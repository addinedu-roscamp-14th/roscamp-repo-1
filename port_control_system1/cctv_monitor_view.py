"""
cctv_monitor_view.py

로컬 웹캠 또는 RTSP 스트림을 실시간으로 보여주는 CCTV 모니터링 화면입니다.
팀원이 만든 cctv_view.py의 핵심 아이디어(스레드로 영상 캡처 + 클래스 변수로
최신 프레임을 다른 화면과 공유)를 그대로 가져오되, 우리 프로젝트 스타일
(폰트/색상/"🗣️ 명령" 버튼)에 맞춰 다시 정리했습니다.

CCTVMonitorView.SHARED_FRAME 클래스 변수에 최신 프레임(BGR, numpy 배열)을
계속 저장해두기 때문에, dashboard_view.py 같은 다른 화면에서 이 화면을 굳이
열지 않고도 "지금 카메라가 뭘 보고 있는지" 최신 프레임을 그대로 가져다 쓸 수 있습니다.
"""

import threading
from typing import Optional

import customtkinter as ctk
import cv2
from PIL import Image, ImageTk

import json
import os

# 카메라 선택지 - 실제 환경에 맞게 주소를 추가/수정하세요.
CAMERA_SOURCES = {
    "맵 스트림 (API)": "http://192.168.0.60:8000/video",
    "Local Cam 0": 0,
}

DEFAULT_SOURCE = "맵 스트림 (API)"


class CCTVMonitorView(ctk.CTkFrame):
    # 다른 화면(대시보드 등)이 최신 프레임을 가져다 쓸 수 있도록 클래스 변수에 저장해둡니다.
    SHARED_FRAME = None

    # ── 백그라운드 캡처 (탭 전환과 무관하게 앱 전체에서 1개만 유지) ──
    _bg_running = False
    _bg_thread: Optional[threading.Thread] = None
    _bg_cap: Optional[cv2.VideoCapture] = None
    _bg_source = None
    _bg_lock = threading.Lock()

    @classmethod
    def _load_source_from_config(cls):
        """stream_config.json에서 cctv_url을 읽어 CAMERA_SOURCES를 갱신합니다."""
        config_path = "stream_config.json"
        if os.path.exists(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    config = json.load(f)
                    cctv_url = config.get("cctv_url")
                    if cctv_url:
                        CAMERA_SOURCES["맵 스트림 (API)"] = cctv_url
            except Exception as e:
                print(f"CCTV 설정 로드 실패: {e}")

    @classmethod
    def ensure_capture_running(cls, source=None):
        """백그라운드 캡처 스레드가 실행 중이 아니면 시작합니다.
        이미 실행 중이면 아무것도 하지 않습니다."""
        with cls._bg_lock:
            if cls._bg_running and cls._bg_thread is not None and cls._bg_thread.is_alive():
                return  # 이미 돌고 있음
            cls._load_source_from_config()
            cls._bg_source = source or CAMERA_SOURCES[DEFAULT_SOURCE]
            cls._bg_running = True
            cls._bg_cap = None  # bg_capture_loop 스레드 안에서 초기화 (UI 블로킹 방지)
            cls._bg_thread = threading.Thread(target=cls._bg_capture_loop, daemon=True)
            cls._bg_thread.start()
            print(f"[CCTV 백그라운드] 캡처 시작: {cls._bg_source}")

    @classmethod
    def switch_capture_source(cls, new_source):
        """캡처 소스를 전환합니다 (기존 캡처를 중단하고 새 소스로 재시작)."""
        with cls._bg_lock:
            if cls._bg_source == new_source and cls._bg_running:
                return
            cls._bg_running = False
        if cls._bg_thread is not None and cls._bg_thread.is_alive():
            cls._bg_thread.join(timeout=2)
        with cls._bg_lock:
            if cls._bg_cap is not None:
                cls._bg_cap.release()
            cls._bg_source = new_source
            cls._bg_cap = None # bg_capture_loop 스레드 안에서 초기화 (UI 블로킹 방지)
            cls._bg_running = True
            cls._bg_thread = threading.Thread(target=cls._bg_capture_loop, daemon=True)
            cls._bg_thread.start()
            print(f"[CCTV 백그라운드] 소스 전환: {new_source}")

    @classmethod
    def stop_capture(cls):
        """앱 종료 시 호출 - 백그라운드 캡처를 완전히 중단합니다."""
        cls._bg_running = False
        if cls._bg_thread is not None and cls._bg_thread.is_alive():
            cls._bg_thread.join(timeout=2)
        if cls._bg_cap is not None:
            cls._bg_cap.release()
            cls._bg_cap = None
        cls.SHARED_FRAME = None

    @classmethod
    def _bg_capture_loop(cls):
        """백그라운드 스레드: 프레임을 계속 읽어 SHARED_FRAME에 저장합니다."""
        while cls._bg_running:
            # bg_lock 안에서 cap 가져오기 및 필요시 초기화
            cap = None
            with cls._bg_lock:
                cap = cls._bg_cap
                source_url = cls._bg_source
            
            if cap is None or not cap.isOpened():
                import time
                time.sleep(0.5)
                # cv2.VideoCapture는 시간이 오래 걸릴 수 있으므로 락 밖에서 수행하여 메인 스레드 블로킹 방지
                new_cap = cv2.VideoCapture(source_url)
                
                with cls._bg_lock:
                    if not cls._bg_running or cls._bg_source != source_url:
                        new_cap.release()
                        continue
                    if cls._bg_cap is not None:
                        cls._bg_cap.release()
                    cls._bg_cap = new_cap
                continue
                
            ret, frame = cap.read()
            if ret:
                cls.SHARED_FRAME = frame
            else:
                # 연결이 끊기면 재연결 시도
                with cls._bg_lock:
                    if cls._bg_cap is not None:
                        cls._bg_cap.release()
                        cls._bg_cap = None

    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)

        # 백그라운드 캡처가 아직 안 돌고 있으면 시작
        CCTVMonitorView.ensure_capture_running()

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=24, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=16, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=14)

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # Header
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 20))
        ctk.CTkLabel(header, text="📷 CCTV 실시간 모니터링", font=self.font_title, text_color="#e5e2e3").pack(side="left")

        # Main Container (bg-surface-container-high #2a2a2b)
        main_container = ctk.CTkFrame(self, corner_radius=8, fg_color="#2a2a2b", border_width=1, border_color="#404850")
        main_container.grid(row=1, column=0, sticky="nsew", pady=(0, 20))
        main_container.grid_columnconfigure(0, weight=1)
        main_container.grid_rowconfigure(2, weight=1)

        # Controls Bar
        control_row = ctk.CTkFrame(main_container, fg_color="transparent")
        control_row.grid(row=0, column=0, sticky="ew", padx=20, pady=15)
        
        ctk.CTkLabel(control_row, text="카메라 선택:", font=self.font_body, text_color="#c0c7d1").pack(side="left", padx=(0, 10))
        self.camera_selector = ctk.CTkOptionMenu(
            control_row, values=list(CAMERA_SOURCES.keys()), command=self.change_camera,
            font=self.font_body, fg_color="#353436", button_color="#353436", button_hover_color="#404850", text_color="#e5e2e3", height=36
        )
        self.camera_selector.set(DEFAULT_SOURCE)
        self.camera_selector.pack(side="left")

        # Separator
        sep = ctk.CTkFrame(main_container, fg_color="#404850", height=1)
        sep.grid(row=1, column=0, sticky="ew")

        # Video Frame
        video_frame = ctk.CTkFrame(main_container, fg_color="black", corner_radius=0)
        video_frame.grid(row=2, column=0, sticky="nsew")
        video_frame.grid_columnconfigure(0, weight=1)
        video_frame.grid_rowconfigure(0, weight=1)

        self.video_label = ctk.CTkLabel(video_frame, text="⏳ 카메라 연결 중...", font=self.font_title, text_color="#8a919b", fg_color="black")
        self.video_label.grid(row=0, column=0, sticky="nsew")

        # Floating Actions (Bottom Right)
        bottom_row = ctk.CTkFrame(self, fg_color="transparent")
        bottom_row.grid(row=2, column=0, sticky="e", pady=(0, 0))

        ctk.CTkButton(bottom_row, text="🗣️ 명령", font=self.font_subtitle, fg_color="#92ccff", text_color="#003351",
                      hover_color="#cce5ff", height=40, width=100, command=self.open_command_popup).pack(side="right")

        # CCTV 탭이 보이는 동안 UI를 갱신하는 루프
        self._ui_alive = True
        self._update_ui_loop()

    # ------------------------------------------------------------------
    def _update_ui_loop(self) -> None:
        """CCTV 탭이 보이는 동안 SHARED_FRAME을 읽어 UI에 표시합니다."""
        if not self._ui_alive:
            return
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        frame = CCTVMonitorView.SHARED_FRAME
        if frame is not None:
            try:
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb_frame)

                MAX_W, MAX_H = 960, 700
                available_w = max(self.video_label.winfo_width(), 320)
                available_h = max(self.video_label.winfo_height(), 240)
                w = min(available_w, MAX_W)
                h = min(available_h, MAX_H)

                ctk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(w, h))
                self.video_label.configure(image=ctk_img, text="")
                self.video_label.image = ctk_img
            except Exception:
                pass

        self.after(30, self._update_ui_loop)

    def change_camera(self, choice: str) -> None:
        """드롭다운에서 다른 카메라를 선택하면 백그라운드 캡처 소스를 전환합니다."""
        new_source = CAMERA_SOURCES.get(choice, 0)
        CCTVMonitorView.switch_capture_source(new_source)

    def stop(self) -> None:
        """CCTV 뷰의 UI 갱신만 중단합니다 (백그라운드 캡처는 계속 유지)."""
        self._ui_alive = False

    def destroy(self) -> None:
        self.stop()
        super().destroy()

    # ------------------------------------------------------------------
    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        open_command_popup(self)
