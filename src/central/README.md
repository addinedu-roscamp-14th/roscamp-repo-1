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

/central/target_map_waypoints
nav_msgs/Path

/central/target_map_waypoints_preview
nav_msgs/Path
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

## 여러 웨이포인트 전송

웨이포인트 모드에서는 중간 지점과 최종 목적지를 목록에 누적한 후 한 번에 차량으로
보냅니다. 중간 지점의 헤딩은 다음 지점을 향하도록 자동 계산합니다.

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p waypoint_mode:=true
```

영상에서 다음 순서로 클릭합니다.

1. 중간 웨이포인트: 각각 위치만 한 번 클릭
2. 최종 목적지: 위치를 한 번 클릭
3. 최종 방향: 최종 목적지에서 차량이 바라볼 방향점을 한 번 클릭

예를 들어 중간 웨이포인트가 2개라면 `중간1 -> 중간2 -> 최종 위치 -> 최종 방향`으로
총 네 번 클릭합니다. 마지막 방향점은 차량이 이동할 웨이포인트에 포함되지 않습니다.

필요한 웨이포인트를 모두 찍은 다음 `camera_to_map_bridge`를 실행한 터미널에 포커스를
두고 **스페이스바**를 누르면 전체 경로가 차량으로 전송됩니다.

스페이스바 입력은 브릿지 터미널에 포커스가 있을 때만 동작합니다. 터미널 입력을 사용할
수 없는 환경에서는 기존 서비스를 호출합니다.

```bash
ros2 service call /central/commit_waypoints std_srvs/srv/Trigger "{}"
```

확정하면 `/central/target_map_waypoints`에 `nav_msgs/Path`가 한 번 발행됩니다. 작업 중인
전체 클릭은 `/central/target_map_waypoints_preview`에서 확인할 수 있으며, 마지막 클릭은
확정 시 최종 방향점으로 해석됩니다.

잘못 찍었거나 처음부터 다시 찍으려면 전체 목록과 진행 중인 첫 클릭을 초기화합니다.

```bash
ros2 service call /central/clear_waypoints std_srvs/srv/Trigger "{}"
```

차량 측 Nav2 웨이포인트 브릿지:

```bash
ros2 launch drive target_map_waypoints_nav.launch.xml start_nav2:=false
```

단일 목적지를 사용할 때는 `waypoint_mode`를 켜지 않고 기존
`target_map_pose_nav.launch.xml`을 사용합니다. 두 drive 브릿지를 동시에 실행하지 않습니다.

# 차량에서 실행

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

차량에서는 센서, odometry와 모터 제어만 실행합니다. Nav2와 목표 브릿지는 노트북에서
실행합니다.

```bash
ros2 launch drive bringup_launch.xml \
  map:=/home/jio/poter_ws/config/SLAM/current_map.yaml

ros2 launch drive nav2_view.launch.xml

ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

RViz에서 `2D Pose Estimate`로 실제 차량의 초기 위치와 방향을 지정해야 합니다.



기본 설정:

```text
calibration_yaml: config/central/camera_map_calibration.yaml
input_pixel_topic: /central/target_pixel
output_pose_topic: /central/target_map_pose
output_json_topic: /central/target_map_json
frame_id: map
minimum_direction_distance: 0.02
waypoint_mode: false
enable_spacebar_commit: true
output_waypoints_topic: /central/target_map_waypoints
output_waypoints_preview_topic: /central/target_map_waypoints_preview
commit_waypoints_service: /central/commit_waypoints
clear_waypoints_service: /central/clear_waypoints
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
    "x": -0.124279,
    "y": -1.164190
  },
  "map_pose": {
    "x": -0.330455,
    "y": -1.495575,
    "z": 0.0,
    "yaw": 1.014239,
    "heading_deg": 58.112
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

노트북의 `drive target_map_pose_to_nav_goal` 노드가 이 좌표를 Nav2 `NavigateToPose`
goal로 넘깁니다.

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
