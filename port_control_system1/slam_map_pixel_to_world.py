"""
slam_map_pixel_to_world.py

ROS2 map_server가 생성한 SLAM 점유격자 지도(예: current_map.pgm + current_map.yaml)
위에서 클릭한 픽셀 좌표를, 캘리브레이션 없이 정확한 수식으로 맵(map) 좌표계 좌표로
변환합니다.

CCTV 관제 카메라 화면(pixel_to_map.py, 호모그래피 필요)과는 다른 케이스입니다:
- 이 pgm 지도는 SLAM이 만든 결과물이라서 이미 map 좌표계 그 자체입니다.
- yaml에 있는 resolution/origin만으로 바로 변환 가능 -> 캘리브레이션/오차 없음.

ROS map_server 규격
- resolution: 픽셀당 실제 거리(m/pixel)
- origin: [origin_x, origin_y, yaw] = 이미지의 "좌하단(lower-left) 픽셀"이
  map 좌표계에서 위치하는 (x, y) 좌표
- 이미지 좌표는 (0,0)이 좌상단(top-left), row가 아래로 갈수록 증가
  -> map의 y축(위로 갈수록 증가)과 방향이 반대이므로 뒤집어줘야 함
"""

import json
from pathlib import Path
from typing import Dict, Tuple

import yaml
from PIL import Image

PointXY = Tuple[float, float]


class SlamMap:
    def __init__(self, yaml_path: str):
        yaml_path = Path(yaml_path)
        with yaml_path.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)  # resolution, origin, image 파일명 등을 담은 설정 읽기

        self.resolution: float = float(meta["resolution"])       # 픽셀 1칸이 실제 몇 미터인지 (예: 0.01 = 1cm)
        self.origin_x: float = float(meta["origin"][0])          # 이미지 좌하단 픽셀의 실제 map x좌표
        self.origin_y: float = float(meta["origin"][1])          # 이미지 좌하단 픽셀의 실제 map y좌표
        self.origin_yaw: float = float(meta["origin"][2]) if len(meta["origin"]) > 2 else 0.0

        # yaml이 가리키는 지도 이미지(pgm 등)를 열어서 가로/세로 픽셀 크기만 확인
        # (좌표 변환 시 이미지 높이가 필요해서 - y축 방향이 이미지와 map이 반대이기 때문)
        image_path = yaml_path.parent / meta["image"]
        with Image.open(image_path) as img:
            self.image_width, self.image_height = img.size

    # ------------------------------------------------------------------
    def pixel_to_map(self, px: float, py: float) -> PointXY:
        """지도 이미지 픽셀 좌표(px: 열, py: 행, 좌상단이 0,0) -> map 좌표(m)."""
        # x는 방향이 같으므로 그냥 픽셀 수 * 해상도를 더하면 됨
        map_x = self.origin_x + px * self.resolution
        # y는 이미지 기준(아래로 갈수록 증가) vs map 기준(위로 갈수록 증가)이 반대라서
        # "이미지 전체 높이 - 1 - py"로 뒤집어준 다음 원점을 더함
        map_y = self.origin_y + (self.image_height - 1 - py) * self.resolution
        return map_x, map_y

    def map_to_pixel(self, map_x: float, map_y: float) -> PointXY:
        """map 좌표(m) -> 지도 이미지 픽셀 좌표. (지도 위에 위치를 다시 표시할 때 사용)"""
        # pixel_to_map()의 역연산: 미터 단위 차이를 다시 픽셀 수로 나누고, y는 다시 뒤집음
        px = (map_x - self.origin_x) / self.resolution
        py = (self.image_height - 1) - (map_y - self.origin_y) / self.resolution
        return px, py


def build_location_lookup_from_map_clicks(
    yaml_path: str,
    pixel_locations: Dict[str, PointXY],
    output_path: str,
) -> Dict[str, PointXY]:
    """
    pixel_locations 예:
        {"항구": (85, 40), "A창고": (150, 90), "대기장소": (20, 100)}
    (SLAM 지도 이미지를 화면에 띄워두고, 사용자가 마우스로 클릭한 지점의
    이미지 픽셀 좌표를 그대로 넣으면 됩니다.)
    """
    slam_map = SlamMap(yaml_path)

    lookup: Dict[str, PointXY] = {}
    for name, (px, py) in pixel_locations.items():
        lookup[name] = slam_map.pixel_to_map(px, py)

    Path(output_path).write_text(
        json.dumps({k: list(v) for k, v in lookup.items()}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return lookup


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("사용법: python slam_map_pixel_to_world.py <current_map.yaml 경로>")
        sys.exit(1)

    slam_map = SlamMap(sys.argv[1])
    print(f"지도 크기: {slam_map.image_width} x {slam_map.image_height} px, "
          f"해상도: {slam_map.resolution} m/px, origin: ({slam_map.origin_x}, {slam_map.origin_y})")

    # 예시: 이미지 좌상단(0,0), 정중앙, 좌하단 픽셀을 각각 변환해서 확인
    for label, (px, py) in [
        ("좌상단", (0, 0)),
        ("정중앙", (slam_map.image_width / 2, slam_map.image_height / 2)),
        ("좌하단", (0, slam_map.image_height - 1)),
    ]:
        mx, my = slam_map.pixel_to_map(px, py)
        print(f"  {label} 픽셀({px:.0f},{py:.0f}) -> map 좌표 ({mx:.3f}, {my:.3f})")
