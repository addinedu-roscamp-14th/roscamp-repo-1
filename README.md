# Port-ER Workspace

스마트 항만 관제를 위한 ROS2 워크스페이스입니다. 각 패키지는 역할별로 분리되어 있으며, 자세한 실행 방법은 각 패키지의 `README.md`에 정리합니다.

## Quick Start: 카메라부터 차량 주행까지

차량과 노트북은 같은 네트워크와 같은 `ROS_DOMAIN_ID`를 사용해야 합니다. 아래 노트북
명령은 각각 새 터미널에서 실행하며, 모든 터미널에서 먼저 다음 환경을 불러옵니다.

### 0. 최초 빌드

노트북:

```bash
cd ~/poter_ws
colcon build
source install/setup.bash
```

새 터미널 공통:

```bash
source ~/poter_ws/install/setup.bash
```

### 1. 차량 하드웨어 실행

핑키에서 센서, odometry와 모터 제어만 실행합니다. 차량에서는 Nav2를 실행하지 않습니다.

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

```bash
ros2 run pinky_led led_server
```

```bash
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'fill', r: 255, g: 0,
b: 0}"
```

노트북에서 차량 데이터가 들어오는지 확인합니다.

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 2. 원본 카메라 실행

노트북 터미널 1:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video2 \
  -p image_size:="[640, 480]" \
  -p time_per_frame:="[1, 30]" \
  -p camera_info_url:=file:///home/jio/poter_ws/config/main_camera/camera_info.yaml \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

### 3. 카메라 왜곡 보정

노트북 터미널 2:

```bash
ros2 run image_proc rectify_node --ros-args \
  -r image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  -r image_rect:=/camera/image_rect
```

확인:

```bash
ros2 topic hz /camera/image_rect
ros2 topic hz /image_rect/compressed
```

### 4. YOLO 실행

노트북 터미널 3:

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run yolo yolo_node --ros-args \
  -p weights_path:=config/weights/best.pt
```

출력 확인:

```bash
ros2 topic hz /central/yolo/image_annotated
```

### 5. 카메라 클릭 좌표를 map 좌표로 변환

노트북 터미널 4:

```bash
ros2 run central rqt_click_to_target
```

노트북 터미널 5:

```bash
ros2 run central camera_to_map_bridge
```

### 6. 노트북에서 Nav2 실행

노트북 터미널 6:

```bash
ros2 launch drive bringup_launch.xml \
  map:=/home/jio/poter_ws/config/SLAM/current_map.yaml
```


### 7. RViz에서 차량 초기 위치 설정

노트북 터미널 7:

```bash
ros2 launch drive nav2_view.launch.xml
```

RViz 상단의 `2D Pose Estimate`로 실제 차량 위치와 방향을 지정합니다. 초기 위치를 설정한
후 AMCL과 `map -> odom` TF를 확인합니다.

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
```

### 8. 중앙 목표를 Nav2에 연결

노트북 터미널 8:

```bash
ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

Nav2 action과 속도 명령 확인:

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /cmd_vel
```

### 9. RQT에서 목표 위치와 방향 클릭

노트북 터미널 9:

```bash
ros2 run rqt_image_view rqt_image_view
```

RQT에서 `/central/yolo/image_annotated`를 선택하고 영상 안을 두 번 클릭합니다.

1. 첫 번째 클릭: 차량이 도착할 위치
2. 두 번째 클릭: 도착 후 차량이 바라볼 방향

두 번째 클릭 후 `/central/target_map_pose`가 발행되고 Nav2가 차량을 이동시킵니다.

```bash
ros2 topic echo /central/target_map_pose
```

### 중요 확인

- `config/SLAM/current_map.yaml`과 `config/central/camera_map_calibration.yaml`은 같은 지도 기준이어야 합니다.
- SLAM 지도를 새로 저장하거나 카메라 위치가 바뀌면 `ros2 run calibration direct_calibrator`로 다시 캘리브레이션합니다.
- SLAM mapping과 Nav2 localization은 동시에 실행하지 않습니다.
- 차량과 노트북 양쪽에서 Nav2를 동시에 실행하지 않습니다.
- `/central/target_map_pose`가 지도 밖이면 Nav2가 `Goal Coordinates ... outside bounds`로 거부합니다.

## Package Structure

```text
poter_ws/
├── config/
│   ├── SLAM/                 # SLAM map yaml/pgm
│   ├── central/              # calibration 결과 yaml
│   ├── main_camera/          # camera_info, calibration yaml
│   └── weights/              # YOLO weight
│
└── src/
    ├── udp/                  # UDP/GStreamer 카메라 입력 fallback
    ├── yolo/                 # YOLO 인식/시각화
    ├── calibration/          # 카메라 픽셀 ↔ SLAM map 캘리브레이션
    ├── central/              # 중앙 좌표 변환/차량 전달용 출력
    ├── drive/                # 노트북 Nav2 실행/차량 goal 브릿지
    ├── slam/                 # LiDAR SLAM 지도 작성/저장
    └── arm/                  # 로봇팔 제어 
```

## Package Summary

| Package | 역할 | 주요 노드/상태 |
| --- | --- | --- |
| `udp` | UDP/GStreamer 방식 카메라 입력 fallback. 로컬 기본 카메라 경로는 `v4l2_camera + image_proc` 사용 | `udp_camera_node` |
| `yolo` | 카메라 이미지에서 객체/영역을 인식하고 annotated image와 detection JSON 발행 | `yolo_node` |
| `calibration` | `/image_rect/compressed` 픽셀 좌표와 SLAM `/map` 좌표를 homography로 캘리브레이션 | `direct_calibrator`, `calibration_verifier` |
| `central` | 캘리브레이션 결과를 사용해 카메라 픽셀 좌표를 `/map` 기준 좌표로 변환하고 JSON/PoseStamped 발행 | `camera_to_map_bridge` |
| `drive` | `/central/target_map_pose`를 차량 Nav2 `NavigateToPose` goal로 전달하고 직접 goal 테스트 지원 | `target_map_pose_to_nav_goal`, `send_nav_goal` |
| `slam` | 차량의 `/scan`, `/odom`, TF를 사용해 노트북에서 SLAM 지도를 작성 | `slam_toolbox`, mapping RViz |
| `arm` | 로봇팔/크레인 제어 담당 예정 | 뼈대 패키지 |

## Data Flow

```text
v4l2_camera
  → /camera/image_raw
  → image rectification
  → /camera/image_rect, /image_rect/compressed
  → YOLO / calibration
  → central
  → /central/target_map_pose
  → 노트북 Nav2
  → /cmd_vel
  → 차량
```

## Camera Transport

로컬 카메라는 `v4l2_camera + image_proc`를 기본으로 사용합니다. `udp` 패키지는 영상을
다른 네트워크 장비로 직접 전송해야 할 때만 사용하는 fallback입니다.

## Important Files

```text
config/SLAM/current_map.yaml
config/SLAM/current_map.pgm
config/central/camera_map_calibration.yaml
config/main_camera/camera_info.yaml
config/weights/best.pt
```

## Rule

각 패키지에는 `README.md`를 두고, 해당 패키지의 노드 기능, 실행 방법, 주요 토픽을 간단하게 정리합니다.

