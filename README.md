# Port-ER Workspace

스마트 항만 관제를 위한 ROS2 워크스페이스입니다. 각 패키지는 역할별로 분리되어 있으며, 자세한 실행 방법은 각 패키지의 `README.md`에 정리합니다.

별도 설명이 없는 명령은 모두 `poter_ws/` 워크스페이스 루트에서 실행하며, 프로젝트
파일 경로는 워크스페이스 기준 상대경로를 사용합니다.

## Quick Start: 카메라부터 차량 주행까지

차량과 노트북은 같은 네트워크와 같은 `ROS_DOMAIN_ID`를 사용해야 합니다. 아래 노트북
명령은 각각 새 터미널에서 실행하며, 모든 터미널에서 먼저 다음 환경을 불러옵니다.

### 0. 시각 동기화 (차량 실행 전 필수)

AGV는 RTC가 없고 로봇 LAN에 인터넷이 없어서, 부팅할 때 마지막 종료 시각을 복원한 채
멈춰 있습니다. 두 차량의 시계가 어긋나면 `map`을 공통 부모로 쓰는 하나의 TF 버퍼 안에서
tf2가 "최신"을 앞선 차량 기준으로 잡고, 뒤처진 차량은 `extrapolation into the past`로
탈락합니다. RViz에서 RobotModel이 빨갛게 깜빡이다 사라지는 증상이 이것입니다.

중앙 노트북에서 최초 1회만:

```bash
sudo ./scripts/setup_ntp_server.sh    # chrony를 로봇 LAN 시간 서버로 설치
```

차량을 켤 때마다, **차량 스택을 실행하기 전에**:

```bash
./scripts/check_fleet_clocks.sh       # 편차 확인 (exit 0이면 진행 가능)
./scripts/sync_vehicle_clocks.sh      # FAIL이면 실행 후 다시 확인
```

### 1. 중앙 관제 노트북

``` bash 
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo ip link set lo multicast on
ros2 daemon stop

zenoh-bridge-ros2dds \
  -c config/network/zenoh_central.json5
```

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PORT_CONTROL_API_TOKEN='porter1234'

ros2 launch porter_bringup fleet_central_laptop.launch.py \
  control_host:=0.0.0.0 \
  start_discovery_server:=false

```

### 2. AGV1

``` bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo ip link set lo multicast on
ros2 daemon stop

zenoh-bridge-ros2dds \
  -c config/network/zenoh_agv1.json5 \
  -e tcp/192.168.5.6:7447
```

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv1 \
  discovery_server:= \
  start_nav2:=true
```

### 3. AGV2

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo ip link set lo multicast on
ros2 daemon stop

zenoh-bridge-ros2dds \
  -c config/network/zenoh_agv2.json5 \
  -e tcp/192.168.5.6:7447
```

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv2 \
  discovery_server:= \
  start_nav2:=true
```

### 중앙제어 노트북 

```bash
cd ~/poter_ws 
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PORT_CONTROL_API_TOKEN='porter1234'

ros2 launch porter_bringup dashboard_laptop.launch.py \
  central_ip:=192.168.5.6 \
  ollama_host:=http://agent.sds.codes \
  llm_model:=gemma4:31b
```
### 잠금해제 

``` bash
cd ~/poter_ws 
./scripts/clear_all_holds.sh --cancel-goals
```

### 0. 최초 빌드

노트북:

```bash
colcon build
source install/setup.bash
```

새 터미널 공통:

```bash
source install/setup.bash
```

### 1. 차량 하드웨어와 Nav2 실행

다중 차량 모드에서는 각 핑키에서 센서, odometry, 모터 제어, AMCL과 Nav2를
함께 실행합니다. 중앙 노트북은 차량별 Nav2 action으로 목표만 전달합니다.

```bash
# AGV1
ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv1 \
  start_nav2:=true
# AGV2
ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv2 \
  start_nav2:=true
```

```bash
ros2 run pinky led_server
```

```bash
ros2 service call /set_led pinky/srv/SetLed \
  "{command: 'fill', pixels: [], r: 0, g: 0, b: 255}"

ros2 service call /set_led pinky/srv/SetLed \
  "{command: 'fill', pixels: [], r: 255, g: 255, b: 0}"
```

노트북에서 차량 데이터가 들어오는지 확인합니다.

```bash
ros2 topic hz /agv1/scan
ros2 topic hz /agv1/odom
ros2 run tf2_ros tf2_echo agv1/odom agv1/base_footprint

ros2 topic hz /agv2/scan
ros2 topic hz /agv2/odom
ros2 run tf2_ros tf2_echo agv2/odom agv2/base_footprint
```

### 2. 원본 카메라 실행

노트북 터미널 1:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video2 \
  -p image_size:="[640, 480]" \
  -p time_per_frame:="[1, 30]" \
  -p camera_info_url:="file://$(realpath config/main_camera/camera_info.yaml)" \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

원본 영상과 보정값을 먼저 확인합니다. `camera_info`의 `k`, `d`가 비어 있거나 모두
0이면 다음 단계로 진행하지 않습니다.

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
```

`camera_name ... does not match ...`는 장치 이름 비교 경고이며 CameraInfo 로드
실패를 의미하지는 않습니다. 위 명령으로 실제 `k`, `d` 값을 확인합니다. 특정
V4L2 control의 `Permission denied`와 YUYV→RGB 변환 메시지도 영상이 정상 발행되면
치명적인 오류가 아닙니다.

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
```

### 4. 왜곡 보정 영상 compressed 발행

노트북 터미널 3:

```bash
ros2 run image_transport republish raw compressed --ros-args \
  -r in:=/camera/image_rect \
  -r out:=/image_rect
```

YOLO와 calibration이 구독하는 토픽을 확인합니다.

```bash
ros2 topic hz /image_rect/compressed
```

### 5. YOLO 실행

노트북 터미널 4:

```bash
source install/setup.bash
ros2 run yolo yolo_node --ros-args \
  -p weights_path:=config/weights/best.pt
```

출력 확인:

```bash
ros2 topic hz /central/yolo/image_annotated
```

### 5-1. 카메라·SLAM API 서버 실행

추가 터미널:

```bash
source install/setup.bash
ros2 launch slam slam_bringup.launch.xml
```

차량에서 실행되는 SLAM Toolbox의 `/map`, `/scan`, `/odom`, `/tf`, `/tf_static`을
노트북 API 서버가 받아 지도, 차량 pose와 LiDAR 데이터를 중계합니다. 노트북에서는
SLAM Toolbox, AMCL 또는 map server를 실행하지 않습니다.

로컬 또는 같은 네트워크의 다른 장치에서 접속합니다.

```text
http://localhost:8000/video
http://<노트북-IP>:8000/video
http://localhost:8000/slam/view
http://<노트북-IP>:8000/slam/view
http://localhost:8000/slam/video
http://<노트북-IP>:8000/slam/video
http://localhost:8000/slam/map.png
http://<노트북-IP>:8000/slam/map.png
```

상태 확인:

```bash
curl http://localhost:8000/health
curl http://localhost:8000/slam/health
```

`/slam/video`는 지도, 차량 위치·헤딩과 LiDAR를 서버에서 합성한 HTTP MJPEG
스트림입니다. `/slam/view`는 이 스트림을 브라우저에서 보여줍니다.

### 6. 카메라 클릭 좌표를 map 좌표로 변환

노트북 터미널 5:

```bash
ros2 run central rqt_click_to_target
```

노트북 터미널 6:

```bash
ros2 run central camera_to_map_bridge
```

### 7. 단일 차량 호환 모드에서 노트북 Nav2 실행

아래 명령은 기존 단일 차량 호환 모드에서만 사용합니다. `agv_vehicle.launch.py`를
`start_nav2:=true`로 실행한 다중 차량 모드에서는 실행하지 않습니다.

```bash
ros2 launch drive bringup_launch.xml \
  map:="$(realpath config/SLAM/current_map.yaml)"
```


### 8. RViz에서 차량 초기 위치 설정

노트북 터미널 8:

```bash
ros2 launch drive nav2_view.launch.xml
```

RViz 상단의 `2D Pose Estimate`로 실제 차량 위치와 방향을 지정합니다. 초기 위치를 설정한
후 AMCL과 `map -> odom` TF를 확인합니다.

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
```

### 9. 중앙 목표를 Nav2에 연결

노트북 터미널 9:

```bash
ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

Nav2 action과 속도 명령 확인:

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /cmd_vel
```

### 10. RQT에서 목표 위치와 방향 클릭

노트북 터미널 10:

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

### 여러 웨이포인트를 찍어서 순서대로 주행

단일 목표 대신 여러 중간 지점을 사용할 때는 터미널 6과 9의 명령을 다음과 같이
바꿉니다.

터미널 6:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p waypoint_mode:=true
```

터미널 9:

```bash
ros2 launch drive target_waypoints_nav.launch.xml start_nav2:=false
```

RQT에서 중간 웨이포인트는 위치만 한 번씩 클릭합니다. 마지막에는 `최종 위치 -> 최종
방향` 순서로 두 번 클릭합니다. 중간 지점의 헤딩은 다음 지점을 향하도록 자동 계산되며,
마지막 방향점은 주행 지점에 포함되지 않습니다. 모든 클릭을 마친 후
`camera_to_map_bridge` 실행 터미널에 포커스를 두고 **스페이스바**를 누르면 전체 경로를
차량으로 보냅니다.

터미널 키 입력을 사용할 수 없으면 기존 서비스 명령으로도 전송할 수 있습니다.

```bash
ros2 service call /central/commit_waypoints std_srvs/srv/Trigger "{}"
```

잘못 찍어서 전체 목록을 지울 때:

```bash
ros2 service call /central/clear_waypoints std_srvs/srv/Trigger "{}"
```

```bash
ros2 topic echo /central/target_map_waypoints_preview
ros2 topic echo /central/target_map_waypoints
```

### 지정 주차 시퀀스 실행

Nav2로 지정 주차 접근 지점까지 이동한 뒤, 마지막 구간은 `/cmd_vel` 후진 제어로 주차
위치까지 들어갑니다. 주차 위치와 접근 경로는 `src/drive/params/parking_spots.yaml`에서
관리합니다.

터미널 9 대신 지정 주차 액션 서버를 실행합니다.

```bash
ros2 run drive parking_action_server
```

주차 명령:

```bash
ros2 action send_goal /park_in_spot drive/action/ParkInSpot \
  "{spot_id: park_red}" --feedback
```

### 중요 확인

- `config/SLAM/current_map.yaml`과 `config/central/camera_map_calibration.yaml`은 같은 지도 기준이어야 합니다.
- SLAM 지도를 새로 저장하거나 카메라 위치가 바뀌면 `ros2 run calibration direct_calibrator`로 다시 캘리브레이션합니다.
- SLAM mapping과 Nav2 localization은 동시에 실행하지 않습니다.
- 다중 차량에서는 각 차량에서만 Nav2를 실행하며 중앙 노트북에서 Nav2를 중복 실행하지 않습니다.
- `/central/target_map_pose`가 지도 밖이면 Nav2가 `Goal Coordinates ... outside bounds`로 거부합니다.

## Quick Start: 로봇팔 컨테이너 Pick

상세한 캘리브레이션, 설정과 문제 해결은 `src/arm/README.md`를 확인합니다.

```bash
source install/setup.bash

ros2 launch arm container_pick_moveit.launch.py \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  video_device:=/dev/video4 \
  marker_id:=0 \
  marker_size_m:=0.015 \
  serial_port:=/dev/ttyUSB0 \
  trajectory_speed:=100 \
  goal_correction_speed:=50 \
  goal_tolerance_deg:=2.5 \
  goal_timeout_sec:=15.0 \
  use_rviz:=true
```

```bash
ros2 service call /arm/preview_pregrasp std_srvs/srv/Trigger '{}'
ros2 service call /arm/move_to_pregrasp std_srvs/srv/Trigger '{}'
ros2 service call /arm/pick_container std_srvs/srv/Trigger '{}'
```

비상 정지:

```bash
ros2 service call /arm/stop_pick std_srvs/srv/Trigger '{}'
```

## Package Structure

```text
poter_ws/
├── config/
│   ├── SLAM/                 # SLAM map yaml/pgm
│   ├── central/              # calibration 결과 yaml
│   ├── arm/                  # 그리퍼 카메라 내부 보정
│   ├── arm2/                 # 두 번째 로봇팔 전용 보정/파지 설정
│   ├── dashboard/            # 영상 API 설정
│   ├── main_camera/          # camera_info, calibration yaml
│   └── weights/              # YOLO weight
│
└── src/
    ├── udp/                  # UDP/GStreamer 카메라 입력 fallback
    ├── yolo/                 # YOLO 인식/시각화
    ├── calibration/          # 카메라 픽셀 ↔ SLAM map 캘리브레이션
    ├── central/              # 중앙 좌표 변환/차량 전달용 출력
    ├── dashboard/            # YOLO 영상과 SLAM 지도의 FastAPI 전송
    ├── drive/                # 노트북 Nav2 실행/차량 goal 브릿지
    ├── slam/                 # LiDAR SLAM 지도 작성/저장
    ├── arm/                  # 로봇팔 ArUco 추적/파지 동작 조정
    ├── arm2/                 # 두 번째 로봇팔용 분리 노드와 launch
    ├── jetcobot_description/ # JetCobot URDF와 mesh
    └── jetcobot_moveit_config/ # MoveIt2 IK/경로 계획 설정
```

## Package Summary

| Package | 역할 | 주요 노드/상태 |
| --- | --- | --- |
| `udp` | UDP/GStreamer 방식 카메라 입력 fallback. 로컬 기본 카메라 경로는 `v4l2_camera + image_proc` 사용 | `udp_camera_node` |
| `yolo` | 카메라 이미지에서 객체/영역을 인식하고 annotated image와 detection JSON 발행 | `yolo_node` |
| `calibration` | `/image_rect/compressed` 픽셀 좌표와 SLAM `/map` 좌표를 homography로 캘리브레이션 | `direct_calibrator`, `calibration_verifier` |
| `central` | 카메라 픽셀을 `/map` 좌표로 변환하고 단일 Pose 또는 웨이포인트 Path 발행 | `camera_to_map_bridge` |
| `drive` | 단일 목표, 웨이포인트 목록과 지정 주차 시퀀스를 차량 Nav2/action으로 전달 | `target_map_pose_to_nav_goal`, `target_waypoints_to_nav_goal`, `parking_action_server` |
| `slam` | 차량의 `/scan`, `/odom`, TF를 사용해 노트북에서 SLAM 지도를 작성 | `slam_toolbox`, mapping RViz |
| `arm` | ArUco XYZ/yaw 추적, Eye-in-Hand, MoveIt2 정렬, Cartesian 파지와 적응형 상승 | `container_pick_coordinator`, `jetcobot_trajectory_bridge` |
| `arm2` | 두 번째 JetCobot용 `/arm2` 토픽, TF, 캘리브레이션과 파지 제어 | `arm2_container_pick_coordinator`, `arm2_jetcobot_trajectory_bridge` |
| `jetcobot_description` | JetCobot의 관절·링크 구조와 시각/충돌 mesh 제공 | `jetcobot.urdf` |
| `jetcobot_moveit_config` | JetCobot용 KDL IK, 충돌 검사, 경로 계획과 controller action 연결 | `real_planning.launch.py` |

## Data Flow

```text
v4l2_camera
  → /camera/image_raw
  → image rectification
  → /camera/image_rect, /image_rect/compressed
  → YOLO / calibration
  → central
  → /central/target_map_pose
  → fleet dispatcher
  → /agv1 또는 /agv2 Nav2 action
  → 차량 로컬 Nav2
  → /agv1 또는 /agv2/cmd_vel
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
