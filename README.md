# Port-ER Workspace

## 분리 도메인 + Zenoh 실행 순서

장비별 ROS Domain은 중앙 `12`, AMR1 `13`, AMR2 `14`, ARM1 `15`,
ARM2 `16`을 사용합니다. 중앙 노트북의 로봇망 IP는 아래 예시에서
`192.168.5.6`입니다. 실제 IP가 다르면 각 Zenoh endpoint와 `central_ip`를 함께
변경합니다. 각 코드 블록은 별도 터미널에서 실행합니다.

### 1. 중앙 관제 노트북

터미널 1 - Zenoh router/bridge:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo ip link set lo multicast on
ros2 daemon stop

zenoh-bridge-ros2dds -c config/network/zenoh_central.json5
```

터미널 2 - 카메라, YOLO, Fleet, ARM dispatcher와 중앙 API:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PORT_CONTROL_API_TOKEN='porter1234'

export PORT_INVENTORY_DB_HOST='192.168.5.5'
export PORT_INVENTORY_DB_PORT='5432'
export PORT_INVENTORY_DB_NAME='port_db'
export PORT_INVENTORY_DB_USER='postgres'
export PORT_INVENTORY_DB_PASSWORD='1234'

ros2 launch porter_bringup fleet_central_laptop.launch.py \
  control_host:=0.0.0.0 \
  start_discovery_server:=false
```

### 2. AMR1

터미널 1 - Zenoh bridge:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=13
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo ip link set lo multicast on
ros2 daemon stop

zenoh-bridge-ros2dds \
  -c config/network/zenoh_agv1.json5 \
  -e tcp/192.168.5.6:7447
```

터미널 2 - 차량 하드웨어, AMCL과 Nav2:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=13
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv1 \
  start_nav2:=true
```

### 3. AMR2

터미널 1 - Zenoh bridge:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=14
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo ip link set lo multicast on
ros2 daemon stop

zenoh-bridge-ros2dds \
  -c config/network/zenoh_agv2.json5 \
  -e tcp/192.168.5.6:7447
```

터미널 2 - 차량 하드웨어, AMCL과 Nav2:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=14
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv2 \
  start_nav2:=true
```

### 4. ARM1 노트북

터미널 1 - 중앙 관제 연결용 Zenoh bridge:

```bash
cd ~/poter_ws

export ROS_DOMAIN_ID=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

zenoh-bridge-ros2dds \
  -c "$HOME/snap/zenoh-bridge-ros2dds/common/config.json5" \
  -e tcp/192.168.5.6:7447
```

터미널 2 - ARM1 Pick/Place 노드(실제 작업 ID는 launch 인자로 지정):

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=15
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch arm_pick_place container_pick_place.launch.py \
  serial_port:=/dev/ttyUSB0 \
  video_device:=/dev/video2 \
  marker_size_m:=0.020
```

ARM1 로컬 계약 확인:

```bash
ros2 service list | grep '^/arm/pick_place/'
ros2 topic echo /arm/pick_place/work_state \
  --qos-durability transient_local \
  --qos-reliability reliable
```

### 5. ARM2 노트북

터미널 1 - 로봇팔, 그리퍼 카메라와 작업 서비스:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=16
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch arm2 arm2_container_pick_moveit.launch.py \
  camera_info_url:=config/arm2/arm2_gripper_camera_info.yaml \
  video_device:=/dev/arm_camera \
  calibration_name:=arm2_jetcobot_eye_in_hand_charuco_5x5_v4 \
  params_file:=config/arm2/arm2_container_pick.yaml \
  use_node_time_for_pose:=true \
  marker_id:=0 \
  stack_marker_id:=11 \
  marker_size_m:=0.020 \
  serial_port:=/dev/jetcobot \
  trajectory_speed:=50 \
  goal_correction_speed:=35 \
  goal_tolerance_deg:=3.5 \
  goal_timeout_sec:=15.0 \
  use_rviz:=true
```

터미널 2 - 중앙 관제 연결용 Zenoh bridge:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=16
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo ip link set lo multicast on
ros2 daemon stop

zenoh-bridge-ros2dds \
  -c config/network/zenoh_arm2.json5 \
  -e tcp/192.168.5.6:7447
```

ARM2 연결 확인:

```bash
ros2 service list | grep '^/arm2/'
ros2 topic echo /arm2/transfer_events
```

### 6. 대시보드 노트북

`OLLAMA_HOST`는 실제 Ollama 서버 주소로 변경합니다. 현재 사용하는 모델이
`qwen3-vl:8b`가 아니면 `LOCAL_LLM_MODEL`도 변경합니다.

``` bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=12
export PORT_CONTROL_API_TOKEN='porter1234'

export OLLAMA_HOST='http://agent.sds.codes'
export LOCAL_LLM_MODEL='gemma4:31b'

export PORT_INVENTORY_DB_HOST='192.168.5.'
export PORT_INVENTORY_DB_PORT='5432'
export PORT_INVENTORY_DB_NAME='port_db'
export PORT_INVENTORY_DB_USER='postgres'
export PORT_INVENTORY_DB_PASSWORD='1234'

ros2 launch porter_bringup dashboard_laptop.launch.py \
  central_ip:=192.168.5.6 \
  ollama_host:="${OLLAMA_HOST}" \
  llm_model:="${LOCAL_LLM_MODEL}"
```

### 차량 led 켜기

```bash
ros2 run pinky led_server
````

```bash
ros2 service call /set_led pinky/srv/SetLed \
  "{command: 'fill', pixels: [], r: 0, g: 0, b: 255}"

ros2 service call /set_led pinky/srv/SetLed \
  "{command: 'fill', pixels: [], r: 255, g: 255, b: 0}"
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
## Rule

각 패키지에는 `README.md`를 두고, 해당 패키지의 노드 기능, 실행 방법, 주요 토픽을 간단하게 정리합니다.
