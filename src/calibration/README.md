# Calibration Package

탑다운 카메라 이미지 좌표와 SLAM PGM/map 좌표를 매칭하기 위한 보조 패키지입니다.

권장 방식은 `direct_calibrator` 하나로 카메라 이미지와 PGM 맵을 둘 다 OpenCV 창에 띄운 뒤, 같은 실제 위치를 클릭해 매칭하는 것입니다. 4쌍 이상 모이면 `camera pixel -> map xy` homography도 함께 계산합니다.

## Nodes

### `direct_calibrator`

카메라 이미지와 SLAM PGM 파일을 각각 OpenCV 창으로 띄우고, 두 창의 마우스 왼쪽 클릭 좌표를 한 쌍으로 저장합니다. 외부 mouse 토픽에 의존하지 않는 권장 노드입니다.

기본 입력:

```text
/image_rect/compressed
config/SLAM/current_map.yaml
config/SLAM/current_map.pgm
```

기본 출력 파일:

```text
config/central/camera_map_calibration.yaml
```

### `calibration_verifier`

생성된 calibration YAML을 읽고, 카메라 클릭점이 PGM 맵의 어디로 투영되는지 확인합니다. 카메라 창에서 검증점을 클릭한 뒤 PGM 창에서 실제 대응점을 클릭하면 오차를 m/px 단위로 출력합니다.

기본 출력 파일:

```text
config/central/camera_map_calibration.yaml
```

## Build

워크스페이스 루트에서 실행합니다.

```bash
cd ~/poter_ws
colcon build --packages-select calibration
source install/setup.bash
```

이미 빌드한 뒤 코드를 수정했다면 다시 `colcon build` 후 `source install/setup.bash`를 실행하세요.

## Run

권장 방식:

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run calibration direct_calibrator
```

카메라 이미지 토픽은 압축된 왜곡 보정 이미지인 `/image_rect/compressed`를 기본으로 사용합니다. 다른 토픽을 임시로 쓰려면:

```bash
ros2 run calibration direct_calibrator --ros-args \
  -p camera_topic:=/image_rect/compressed
```

## Click Workflow

1. PGM 창에서 기준점을 클릭합니다.
2. 카메라 창에서 같은 실제 위치를 클릭합니다.
3. 두 클릭이 한 쌍으로 저장됩니다.
4. 순서는 반대로 해도 됩니다.
5. 최소 4쌍 이상 찍으면 homography가 계산됩니다.

## Verify

캘리브레이션에 사용하지 않은 새로운 점들로 검증합니다.

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run calibration calibration_verifier
```

검증 순서:

1. 카메라 창에서 검증하고 싶은 점을 클릭합니다.
2. PGM 창에 빨간 십자 표시가 생깁니다.
3. PGM 창에서 실제 대응 위치를 클릭합니다.
4. 터미널에 오차가 출력됩니다.

예시 로그:

```text
Validation #1: actual_pgm=[...], projected_pgm=[...], error=0.0230 m (2.30 px)
```

주의: 캘리브레이션에 사용한 4개 점은 보통 오차가 0에 가깝게 나옵니다. 정확도 검증은 반드시 새 점으로 해야 합니다.

클릭 예시:

```text
PGM에서 차선 모서리 클릭
카메라에서 같은 차선 모서리 클릭
```

저장되는 정보:

```yaml
points:
  - index: 1
    camera_pixel: [u, v]
    pgm_pixel: [x, y]
    map_xy: [x_meter, y_meter]
homography:
  camera_pixel_to_map_xy: [...]
  reprojection_error_m:
    mean: ...
    max: ...
```

## Coordinate Notes

OpenCV 이미지 픽셀 좌표는 보통 아래 기준입니다.

```text
x: 오른쪽 증가
y: 아래쪽 증가
```

SLAM map 좌표 변환은 `current_map.yaml`의 `resolution`, `origin`, PGM 이미지 높이를 사용합니다.

```text
map_x = origin_x + pgm_x * resolution
map_y = origin_y + (map_height - pgm_y) * resolution
```

현재 기본 맵:

```text
resolution: 0.010
origin: [-1.366, -1.903, 0]
image size: 205 x 231
```

카메라 각도 때문에 PGM의 좌측 상단이 카메라에서는 좌측 하단처럼 보일 수 있습니다. 이 반전/회전/원근 왜곡은 클릭한 대응점들을 기반으로 homography가 흡수합니다.

## Parameters

PGM 클릭 창의 맵 YAML 경로 변경:

```bash
ros2 run calibration direct_calibrator --ros-args \
  -p map_yaml:=config/SLAM/current_map.yaml
```

출력 YAML 경로 변경:

```bash
ros2 run calibration direct_calibrator --ros-args \
  -p output_yaml:=config/central/camera_map_calibration.yaml
```

## Troubleshooting

### `Map yaml not found`

워크스페이스 루트에서 실행했는지 확인하세요.

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run calibration direct_calibrator
```

기본 경로는 다음입니다.

```text
config/SLAM/current_map.yaml
```

### `Package 'calibration' not found`

빌드 또는 source가 빠진 상태입니다.

```bash
cd ~/poter_ws
colcon build --packages-select calibration
source install/setup.bash
```

### 카메라 영상이 안 보임

```bash
ros2 topic list | grep image_rect
ros2 topic hz /image_rect/compressed
```

`/image_rect/compressed`는 압축된 왜곡 보정 이미지입니다. 이 토픽이 발행되지 않으면 `direct_calibrator`와 `calibration_verifier`의 카메라 창은 비어 있습니다.
