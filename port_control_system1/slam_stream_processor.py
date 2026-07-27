"""
slam_stream_processor.py

SLAM 스트림(HTTP MJPEG)을 백그라운드에서 수신하고, 객체를 탐지한 뒤
탐지 결과(라벨, 신뢰도, 좌표)를 클래스 변수로 공유하는 싱글톤 프로세서입니다.

다른 화면(레이더뷰, SLAM 모니터)에서는 별도로 스트림을 열 필요 없이
SlamStreamProcessor.DETECTED_OBJECTS / SHARED_SLAM_FRAME을 읽기만 하면 됩니다.

좌표 변환 흐름:
    SLAM 영상 프레임에서 객체 탐지
    → 탐지 박스 중심의 이미지 픽셀 좌표 (cx, cy)
    → SlamMap.pixel_to_map() → map 좌표(미터)
    → SlamMap.map_to_pixel()의 역으로 map_image_pixel 좌표도 제공
      (레이더뷰 캔버스에 그릴 때 사용)

사용법:
    processor = SlamStreamProcessor.get_instance()
    processor.start("http://192.168.4.13:8000/slam/video")
    ...
    processor.stop()
"""

import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from slam_map_pixel_to_world import SlamMap

# SLAM 스트림 기본 URL
DEFAULT_SLAM_URL = os.environ.get(
    "PORT_CONTROL_SLAM_URL",
    "http://192.168.0.60:8000/slam/video",
)
DEFAULT_SLAM_MAP_YAML = "current_map.yaml"




class SlamStreamProcessor:
    """
    SLAM 스트림 수신 + 객체 탐지 + 좌표 변환을 수행하는 싱글톤 프로세서.

    다른 모듈에서 아래 클래스 변수를 읽어 최신 데이터를 사용합니다:
        SlamStreamProcessor.DETECTED_OBJECTS  - 탐지된 객체 리스트
        SlamStreamProcessor.SHARED_SLAM_FRAME - 최신 SLAM 프레임 (BGR numpy)
        SlamStreamProcessor.ANNOTATED_FRAME   - 탐지 박스가 그려진 프레임
    """

    # 다른 화면과 공유하는 클래스 변수
    SHARED_SLAM_FRAME: Optional[np.ndarray] = None

    _instance: Optional["SlamStreamProcessor"] = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "SlamStreamProcessor":
        """싱글톤 인스턴스를 반환합니다."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        self.url: Optional[str] = None
        self.running = False
        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._slam_map: Optional[SlamMap] = None

        # SLAM 지도 좌표 변환기 로드
        try:
            self._slam_map = SlamMap(DEFAULT_SLAM_MAP_YAML)
        except Exception as e:
            print(f"[SLAM 프로세서] SLAM 지도 로드 실패: {e}")
            print("[SLAM 프로세서] 좌표 변환 없이 픽셀 좌표만 사용합니다.")



    # ------------------------------------------------------------------
    # 시작 / 중지
    # ------------------------------------------------------------------
    def start(self, url: Optional[str] = None) -> None:
        """SLAM 스트림 수신 + 탐지를 시작합니다."""
        self.stop()

        import json
        config_url = DEFAULT_SLAM_URL
        config_path = "stream_config.json"
        if os.path.exists(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    config = json.load(f)
                    if config.get("slam_url"):
                        config_url = config["slam_url"]
            except Exception as e:
                print(f"[SLAM 프로세서] 설정 로드 실패: {e}")
        config_url = os.environ.get(
            "PORT_CONTROL_SLAM_URL",
            config_url,
        )

        self.url = (url or config_url).strip()
        if not self.url:
            return

        self.running = True
        self._thread = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()
        print(f"[SLAM 프로세서] 스트림 수신 시작: {self.url}")

    def stop(self) -> None:
        """스트림 수신을 중단하고 자원을 정리합니다."""
        self.running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3)
        if self._cap is not None:
            self._cap.release()
            self._cap = None

        SlamStreamProcessor.SHARED_SLAM_FRAME = None
        print("[SLAM 프로세서] 스트림 수신 중단")

    @property
    def is_running(self) -> bool:
        return self.running and self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------
    # 메인 처리 루프
    # ------------------------------------------------------------------
    def _process_loop(self) -> None:
        """백그라운드 스레드: 프레임 수신"""
        self._cap = cv2.VideoCapture(self.url)
        if not self._cap.isOpened():
            print(f"[SLAM 프로세서] 연결 실패: {self.url}")
            self.running = False
            return

        retry_count = 0
        while self.running:
            ret, frame = self._cap.read()
            if not ret:
                retry_count += 1
                if retry_count > 30:
                    self._cap.release()
                    time.sleep(1)
                    self._cap = cv2.VideoCapture(self.url)
                    retry_count = 0
                continue

            retry_count = 0
            SlamStreamProcessor.SHARED_SLAM_FRAME = frame.copy()
