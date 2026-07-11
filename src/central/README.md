# Central Package

중앙 관제에서 카메라 픽셀 좌표를 SLAM `/map` 좌표로 변환하고, 차량/브릿지에 전달 가능한 형태로 발행하는 패키지입니다.

현재 핵심 노드는 `rqt_click_to_target`와 `camera_to_map_bridge`입니다.

## `rqt_click_to_target`

rqt image view에서 마우스로 클릭한 픽셀 좌표를 `/central/target_pixel`로 변환해 발행합니다.

입력:

```text
/central/yolo/image_annotated_mouse_left
geometry_msgs/Point
```

출력:

```text
/central/target_pixel
geometry_msgs/PointStamped
```

## `camera_to_map_bridge`

캘리브레이션 결과 YAML을 읽어서 카메라 픽셀 좌표 `(u, v)`를 SLAM map 좌표 `(x, y)`로 변환합니다.

입력:

```text
/central/target_pixel
geometry_msgs/PointStamped
```

출력:

```text
/central/target_map_pose
geometry_msgs/PoseStamped

/central/target_map_json
std_msgs/String
```

기본 calibration 파일:

```text
config/central/camera_map_calibration.yaml
```

이 파일은 `calibration` 패키지의 `direct_calibrator`로 생성합니다.

## Build

워크스페이스 루트에서 실행합니다.

```bash
cd ~/poter_ws
colcon build
source install/setup.bash
```

## Run

전체 실행 흐름:

```text
rqt click
→ /central/yolo/image_annotated_mouse_left
→ rqt_click_to_target
→ /central/target_pixel
→ camera_to_map_bridge
→ /central/target_map_pose
→ /central/target_map_json
```

---

# 노트북에서 실행


### rqt 클릭 좌표 발행

먼저 rqt image view에서 `/central/yolo/image_annotated`를 열고 마우스로 클릭합니다.

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run central rqt_click_to_target
```

클릭 좌표 확인:

```bash
ros2 topic echo /central/target_pixel
```

### map 좌표 변환

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run central camera_to_map_bridge
```
### Nav2 
```bash
ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

### 목표 위치와 방향 클릭

rqt 영상에서 두 번 클릭합니다.

1. 첫 번째 클릭: 차량이 도착할 목표 위치
2. 두 번째 클릭: 목표 위치에서 차량이 바라볼 방향

두 번째 클릭 지점은 차량이 이동할 위치가 아니라 방향을 계산하기 위한 점입니다.
브릿지는 두 점을 모두 map 좌표로 변환하고 첫 번째 점에서 두 번째 점을 향하는 yaw를
계산한 뒤 `/central/target_map_pose`를 한 번 발행합니다.

# 차에서 실행

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```
```bash
ros2 launch pinky_navigation bringup_launch.xml map:=/home/pinky/current_map.yaml
```





기본 설정:

```text
calibration_yaml: config/central/camera_map_calibration.yaml
input_pixel_topic: /central/target_pixel
output_pose_topic: /central/target_map_pose
output_json_topic: /central/target_map_json
frame_id: map
minimum_direction_distance: 0.02
```

## Test

목표 위치와 방향점을 순서대로 직접 발행합니다.

```bash
ros2 topic pub --once /central/target_pixel geometry_msgs/msg/PointStamped \
"{header: {frame_id: camera}, point: {x: 160.0, y: 355.0, z: 0.0}}"

ros2 topic pub --once /central/target_pixel geometry_msgs/msg/PointStamped \
"{header: {frame_id: camera}, point: {x: 260.0, y: 355.0, z: 0.0}}"
```

PoseStamped 출력 확인:

```bash
ros2 topic echo /central/target_map_pose
```

JSON 출력 확인:

```bash
ros2 topic echo /central/target_map_json
```

## JSON Format

`/central/target_map_json`은 `std_msgs/String` 안에 JSON 문자열로 발행됩니다.

예시:

```json
{
  "frame_id": "map",
  "target_id": "target",
  "source_frame_id": "camera",
  "stamp": {
    "sec": 0,
    "nanosec": 0
  },
  "target_camera_pixel": {
    "u": 160.0,
    "v": 355.0
  },
  "direction_camera_pixel": {
    "u": 260.0,
    "v": 355.0
  },
  "direction_map_point": {
    "x": 0.531234,
    "y": 1.278
  },
  "map_pose": {
    "x": 0.065333,
    "y": 1.278,
    "z": 0.0,
    "yaw": 1.570796,
    "heading_deg": 90.0
  }
}
```

## PoseStamped Format

`/central/target_map_pose`는 Nav2 또는 차량 브릿지에서 바로 읽기 좋은 `geometry_msgs/PoseStamped`입니다.

```text
header.frame_id: map
pose.position.x: map x
pose.position.y: map y
pose.position.z: 0.0
pose.orientation: 첫 번째 map 점에서 두 번째 map 점을 향하는 yaw quaternion
```

차량 라즈베리파이 쪽에서 이 좌표를 Nav2 `NavigateToPose` goal로 넘기면 됩니다.

## Parameters

다른 calibration YAML 사용:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p calibration_yaml:=config/central/camera_map_calibration.yaml
```

입력/출력 토픽 변경:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p input_pixel_topic:=/central/target_pixel \
  -p output_pose_topic:=/central/target_map_pose \
  -p output_json_topic:=/central/target_map_json
```

방향점 최소 거리 변경:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p minimum_direction_distance:=0.05
```

target id 변경:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p target_id:=AGV_goal_1
```

rqt mouse 입력 토픽 변경:

```bash
ros2 run central rqt_click_to_target --ros-args \
  -p mouse_topic:=/central/yolo/image_annotated_mouse_left \
  -p target_pixel_topic:=/central/target_pixel
```

## Important Notes

차량 Nav2에서 사용하는 map과 calibration에 사용한 map은 같아야 합니다.

확인해야 할 항목:

```text
resolution
origin
PGM image size
map frame id
```

현재 calibration YAML은 아래 파일을 기준으로 합니다.

```text
config/SLAM/current_map.yaml
config/SLAM/current_map.pgm
```

차량이 다른 map을 쓰면 변환 좌표가 맞지 않습니다.

## Troubleshooting

### `Calibration yaml not found`

`config/central/camera_map_calibration.yaml`이 있는지 확인합니다.

```bash
ls config/central/camera_map_calibration.yaml
```

없으면 먼저 calibration 패키지에서 생성해야 합니다.

```bash
ros2 run calibration direct_calibrator
```

### 출력이 안 나옴

입력 픽셀 토픽이 들어오는지 확인합니다.

```bash
ros2 topic echo /central/target_pixel
```

출력 토픽 목록 확인:

```bash
ros2 topic list | grep central
```
