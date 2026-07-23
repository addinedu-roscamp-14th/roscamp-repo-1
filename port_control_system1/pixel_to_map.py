"""
pixel_to_map.py

관제(탑뷰) 카메라의 픽셀 좌표를 AGV가 인식하는 라이다 SLAM 맵 좌표(미터 단위)로
변환하기 위한 호모그래피(Homography) 캘리브레이션/변환 모듈.

전제 조건
- 관제 카메라가 바닥(평면)을 내려다보는 고정 카메라여야 정확도가 보장됩니다.
- 카메라 위치를 바꾸면 캘리브레이션을 다시 해야 합니다.

사용 흐름
1. 실세계에서 좌표가 이미 알려진 지점(마커 등) 최소 4곳에 대해
   - 그 지점의 픽셀 좌표 (관제 카메라 영상에서 확인)
   - 그 지점의 맵 좌표 (AGV를 직접 그 지점까지 주행시켜 SLAM/AMCL이 인식한
     로봇 pose를 읽어서 얻음, 단위: 미터)
   두 가지를 확보해서 add_correspondence()로 등록합니다.
2. compute()로 호모그래피 행렬을 구합니다 (4개면 정확해, 여러 개면 RANSAC으로 보정).
3. save()/load()로 계산 결과를 파일에 저장해두고 재사용합니다.
4. 이후에는 pixel_to_map()에 임의의 픽셀 좌표를 넣으면 맵 좌표가 나옵니다.
"""

import json
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

PointXY = Tuple[float, float]


class PixelToMapCalibrator:
    def __init__(self) -> None:
        self._pixel_points: List[PointXY] = []   # 등록된 픽셀 좌표들 (관제 카메라 화면 기준)
        self._map_points: List[PointXY] = []      # 위 픽셀과 1:1 대응하는 실제 map 좌표들 (미터)
        self._homography: Optional[np.ndarray] = None  # compute() 후 채워지는 3x3 변환 행렬

    # ------------------------------------------------------------------
    # 대응점 등록
    # ------------------------------------------------------------------
    def add_correspondence(self, pixel_xy: Sequence[float], map_xy: Sequence[float]) -> None:
        """픽셀 좌표 <-> 맵 좌표 대응점 한 쌍을 등록합니다."""
        self._pixel_points.append((float(pixel_xy[0]), float(pixel_xy[1])))
        self._map_points.append((float(map_xy[0]), float(map_xy[1])))
        self._homography = None  # 새 점이 추가됐으니 재계산 필요

    @property
    def num_points(self) -> int:
        return len(self._pixel_points)

    # ------------------------------------------------------------------
    # 호모그래피 계산
    # ------------------------------------------------------------------
    def compute(self, use_ransac: bool = True) -> np.ndarray:
        if self.num_points < 4:
            # 호모그래피(2D 평면 -> 2D 평면 변환)는 수학적으로 최소 4개의 점이 있어야
            # 8개의 미지수(3x3 행렬에서 마지막 원소 1 고정)를 풀 수 있습니다.
            raise ValueError(
                f"호모그래피 계산에는 최소 4개의 대응점이 필요합니다 (현재 {self.num_points}개)."
            )

        src = np.array(self._pixel_points, dtype=np.float64)  # 변환 전(픽셀) 좌표 배열
        dst = np.array(self._map_points, dtype=np.float64)    # 변환 후(map, 미터) 좌표 배열

        if self.num_points == 4:
            # 점이 정확히 4개면 방정식이 딱 떨어지므로(정확해) getPerspectiveTransform 사용
            H = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
        else:
            # 점이 5개 이상이면 측정 오차가 섞여 있을 수 있으므로,
            # RANSAC으로 이상치(잘못 찍은 점)를 걸러내며 최적의 행렬을 추정합니다.
            method = cv2.RANSAC if use_ransac else 0
            H, _mask = cv2.findHomography(src, dst, method=method)
            if H is None:
                raise RuntimeError("호모그래피 계산에 실패했습니다. 대응점 배치를 확인해주세요.")

        self._homography = H  # 이후 pixel_to_map()에서 재사용
        return H

    # ------------------------------------------------------------------
    # 변환
    # ------------------------------------------------------------------
    def pixel_to_map(self, pixel_xy: Sequence[float]) -> PointXY:
        """픽셀 좌표 -> 맵 좌표(미터) 변환."""
        if self._homography is None:
            self.compute()  # 아직 계산 안 됐으면 등록된 점들로 자동 계산

        # cv2.perspectiveTransform은 (N, 1, 2) 형태의 배열을 요구해서 이렇게 감싸줍니다.
        pt = np.array([[[float(pixel_xy[0]), float(pixel_xy[1])]]], dtype=np.float64)
        mapped = cv2.perspectiveTransform(pt, self._homography)  # 실제 행렬 곱 연산 수행
        mx, my = mapped[0][0]  # 감싼 배열에서 결과값(x, y) 꺼내기
        return float(mx), float(my)

    # ------------------------------------------------------------------
    # 저장 / 불러오기
    # ------------------------------------------------------------------
    def save(self, path: str) -> None:
        if self._homography is None:
            self.compute()

        data = {
            "homography": self._homography.tolist(),  # numpy 배열은 JSON에 못 넣으니 리스트로 변환
            "pixel_points": self._pixel_points,        # 나중에 재검증/재계산할 때 쓰려고 원본 점도 같이 저장
            "map_points": self._map_points,
        }
        Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: str) -> "PixelToMapCalibrator":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        calib = cls()
        calib._pixel_points = [tuple(p) for p in data["pixel_points"]]
        calib._map_points = [tuple(p) for p in data["map_points"]]
        calib._homography = np.array(data["homography"], dtype=np.float64)  # 저장해둔 행렬 그대로 복원 (재계산 불필요)
        return calib

    # ------------------------------------------------------------------
    # 정확도 검증
    # ------------------------------------------------------------------
    def reprojection_error(self) -> List[float]:
        """등록된 대응점 자체를 되돌려 변환해서 오차(미터)를 계산합니다.
        캘리브레이션 품질을 확인하는 용도입니다 - 값이 크면 대응점 좌표를 재확인하세요."""
        if self._homography is None:
            self.compute()

        errors = []
        for pixel, map_pt in zip(self._pixel_points, self._map_points):
            mx, my = self.pixel_to_map(pixel)
            err = ((mx - map_pt[0]) ** 2 + (my - map_pt[1]) ** 2) ** 0.5
            errors.append(err)
        return errors


if __name__ == "__main__":
    # 간단한 자체 검증: 실제로는 미리 알려진 4개 이상의 (픽셀, 맵) 좌표 쌍을 사용합니다.
    calib = PixelToMapCalibrator()
    calib.add_correspondence((100, 100), (0.0, 0.0))
    calib.add_correspondence((900, 100), (10.0, 0.0))
    calib.add_correspondence((900, 700), (10.0, 8.0))
    calib.add_correspondence((100, 700), (0.0, 8.0))

    calib.compute()
    print("재투영 오차(m):", [f"{e:.4f}" for e in calib.reprojection_error()])

    test_pixel = (500, 400)
    mx, my = calib.pixel_to_map(test_pixel)
    print(f"테스트 픽셀 {test_pixel} -> 맵 좌표 ({mx:.3f}, {my:.3f})")
