# Arm Package

JetCobot의 기존 Pick/Place 동작을 보존하면서 탑다운 카메라의 거친 접근과 그리퍼
카메라의 ArUco 정밀 정렬을 연결하기 위한 ROS 2 패키지입니다.

## 목표 구조

```text
탑다운 카메라
  -> 물체 식별과 영상 내 대략적인 탐색 영역 결정
  -> 고정된 사전 접근 자세로 이동
  -> 그리퍼 카메라에서 컨테이너 ArUco 6D pose 검출
  -> Eye-in-Hand TF로 arm/base_link 기준 pose 변환
  -> XYZ와 회전 정밀 보정
  -> 기존 하강/그리퍼 닫기/상승 동작
```

Hand-Eye 결과가 검증되기 전에는 ArUco 좌표를 자동 로봇 이동 명령으로 사용하지
않습니다.

높이가 계속 달라지는 물체에는 단일 평면 homography를 사용하지 않습니다. 단안
탑다운 영상의 한 픽셀은 3D 공간의 한 광선만 나타내므로, 높이 정보 없이 정확한 XYZ를
결정할 수 없습니다. 현재 설계에서는 탑다운 영상을 물체 선택과 거친 탐색에만 사용하고,
실제 XYZ와 자세는 그리퍼 카메라의 ArUco 6D pose와 Hand-Eye TF로 계산합니다.

## 구성

```text
arm/
├── arm/
│   ├── main.py                         # 기존 검증용 클릭 Pick/Place
│   ├── aruco_pose_publisher.py         # ArUco 6D pose와 TF 발행
│   ├── hardware_joint_state_publisher.py # 실제 관절각 발행
│   ├── manual_jog.py                   # TCP 수동 이동과 관절각 발행
│   ├── container_pick_coordinator.py   # 컨테이너 추적과 보호된 집기 순서
│   ├── _container_task.py              # 기존 Pick/Place 작업 순서
│   ├── _robot_utils.py                 # JetCobot 이동과 그리퍼 제어
│   ├── _vision_utils.py                # 기존 그리퍼뷰 검출
│   └── _config.py                      # 속도, 높이와 기존 보정값
└── launch/
    ├── robot_tf.launch.py              # URDF와 실제 관절 TF
    ├── gripper_aruco.launch.py         # /dev/video4와 ArUco 검출
    ├── handeye_calibration.launch.py   # Eye-in-Hand 샘플 수집
    ├── handeye_publish.launch.py       # 저장한 Hand-Eye TF 발행
    └── container_pick.launch.py        # 추적/TF/집기 코디네이터 통합 실행
```

고정 로봇 자세에서만 유효했던 탑다운 ROI와 그리퍼 영상 사이의 2D homography
캘리브레이터는 제거했습니다.

## 의존성

`pymycobot` 설치:

```bash
/usr/bin/python3 -m pip install --user --break-system-packages pymycobot
```
``` bash
sudo apt update
sudo apt install ros-jazzy-v4l2-camera
```


easy_handeye2`는 Jazzy 바이너리 패키지가 없으므로 소스로 추가합니다.

```bash
cd ~/poter_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git

cd ~/poter_ws
rosdep install --from-paths src --ignore-src -r -y
```

빌드:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  jetcobot_description arm easy_handeye2 easy_handeye2_msgs
source install/setup.bash
```

## 1. 그리퍼 카메라 내부 보정

Hand-Eye 전에 `/dev/video4`의 내부 파라미터를 먼저 구해야 합니다. 실제 운용과 같은
`YUYV 640x480@10 FPS`로 카메라를 실행합니다.

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __ns:=/arm/gripper_camera \
  -p video_device:=/dev/video4 \
  -p image_size:="[640,480]" \
  -p time_per_frame:="[1,10]" \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8 \
  -p camera_frame_id:=arm/gripper_camera_optical_frame
```

체커보드 크기와 한 칸의 실제 길이는 사용하는 출력물에 맞게 변경합니다.

```bash
ros2 run camera_calibration cameracalibrator \
  --size 10x7 \
  --square 0.015 \
  --no-service-check \
  --ros-args \
  -r image:=/arm/gripper_camera/image_raw
```

결과 YAML을 다음 경로에 저장합니다.

```text
config/arm/gripper_camera_info.yaml
```

해상도, 렌즈, 카메라 초점 또는 카메라 장착을 바꾸면 다시 보정합니다.

## 2. ArUco 6D pose 확인

`marker_size_m`는 검은 사각형의 실제 한 변 길이를 meter로 정확하게 입력합니다.

```bash
ros2 launch arm gripper_aruco.launch.py \
  video_device:=/dev/video4 \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  marker_id:=0 \
  marker_size_m:=0.02 \
  dictionary:=DICT_5X5_50
```

확인:

```bash
ros2 topic echo /arm/gripper_camera/aruco_pose
ros2 run tf2_ros tf2_echo \
  arm/gripper_camera_optical_frame arm/handeye_target
ros2 run rqt_image_view rqt_image_view \
  /arm/gripper_camera/aruco_annotated
```

인터페이스:

```text
입력: /arm/gripper_camera/image_raw (sensor_msgs/Image)
입력: /arm/gripper_camera/camera_info (sensor_msgs/CameraInfo)
출력: /arm/gripper_camera/aruco_pose (geometry_msgs/PoseStamped)
출력: /arm/gripper_camera/aruco_annotated (sensor_msgs/Image)
TF: arm/gripper_camera_optical_frame -> arm/handeye_target
```

재투영 오차가 기본 `3 px`보다 크거나 마커 ID가 다르면 pose를 발행하지 않습니다.

## 3. Eye-in-Hand 캘리브레이션

ArUco 마커를 작업대에 움직이지 않도록 고정합니다. 컨테이너 마커를 사용할 경우
캘리브레이션이 끝날 때까지 컨테이너가 절대 움직이면 안 됩니다. 바닥 전체를 덮는
ArUco 보드는 필요하지 않으며, 현재 노드는 고정된 단일 `DICT_5X5_50` 마커를
사용합니다.

터미널 1에서 TCP 수동 조작을 실행합니다. 이 노드가 로봇 시리얼을 단독 점유하면서
`/arm/joint_states`도 발행합니다.

```bash
ros2 run arm manual_jog --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p speed:=10
```

터미널 2에서 URDF, 그리퍼 카메라, ArUco TF와 easy_handeye2 GUI를 실행합니다.

```bash
ros2 launch arm handeye_calibration.launch.py \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  marker_id:=0 \
  marker_size_m:=0.02
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo arm/base_link arm/TCP
ros2 run tf2_ros tf2_echo \
  arm/gripper_camera_optical_frame arm/handeye_target
```

샘플 수집:

1. `manual_jog` 터미널에서 `T`를 눌러 5초 동안 토크를 풉니다.
2. 팔을 계속 손으로 지지하면서 마커가 보이는 다른 자세로 이동합니다.
3. 토크 복구 로그가 나온 뒤 팔이 완전히 멈출 때까지 기다립니다.
4. easy_handeye2 GUI에서 `Take Sample`을 누릅니다.
5. 손목 회전을 여러 축과 양쪽 방향으로 충분히 변경합니다.
6. 서로 다른 자세를 최소 12개, 권장 15~20개 수집합니다.
7. 계산 결과와 평가 오차를 확인한 뒤 저장합니다.

로봇팔, 마커와 주변 장비가 충돌하지 않도록 낮은 속도로 이동합니다. `manual_jog`와
`hardware_joint_state_publisher`는 `/dev/ttyUSB0`을 동시에 열면 안 됩니다.

### ChArUco Board Hand-Eye 보정

현재 보정 보드는 `DICT_4X4_50`, 11x8 squares, checker 15 mm, marker 11 mm이며,
짝수 행 구형 배열에 맞춰 `legacy_pattern:=true`를 사용합니다.
노트북에 로봇팔과 그리퍼 카메라를 직접 연결하고 다음을 실행합니다.

```bash
ros2 launch arm handeye_charuco_calibration.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  dictionary:=DICT_4X4_50 \
  squares_x:=11 \
  squares_y:=8 \
  square_length_m:=0.015 \
  marker_length_m:=0.011 \
  legacy_pattern:=true \
  detection_rate_hz:=5.0 \
  opencv_num_threads:=1 \
  minimum_charuco_corners:=6 \
  max_reprojection_error_px:=3.0 \
  use_node_time_for_pose:=true \
  name:=jetcobot_eye_in_hand_charuco \
  calibration_directory:=config/arm
```

검출 영상은 `/arm/gripper_camera/charuco_annotated`, pose는
`/arm/gripper_camera/charuco_pose`에서 확인합니다. 보드를 고정한 상태에서 서로 다른
위치와 Roll/Pitch/Yaw 자세를 20~30개 수집한 뒤 계산하고 저장합니다. 컨테이너 추적에는
이 보드 설정이 아니라 기존 `DICT_5X5_50`, ID 0, 15 mm 단일 마커를 계속 사용합니다.
annotated 영상의 `DETECT markers=N corners=N`과 `POSE OK`로 상태를 확인합니다.
마커는 검출되지만 `corners=0`이면 출력 보드와 `legacy_pattern` 설정이 서로 다른지
확인합니다. CPU 부하가 크면 `detection_rate_hz:=3.0`으로 낮춰도 샘플 수집에는
충분합니다.
이 launch는 ChArUco 검출 노드에는 사용자 OpenCV 5를 사용하고,
`calibrateHandEye()`를 실행하는 easy_handeye2 서버에는 시스템 OpenCV 4.6을
자동으로 사용합니다. 세 번째 샘플부터 보정값을 계산하며, `Take Sample` 서비스와
TF 상태 확인은 GUI가 멈추지 않도록 비동기로 처리됩니다.

## 4. 저장한 Hand-Eye TF 사용

```bash
ros2 launch arm handeye_publish.launch.py \
  name:=jetcobot_eye_in_hand_charuco \
  calibration_directory:=config/arm
```
발행 확인:

```bash
ros2 run tf2_ros tf2_echo \
  arm/TCP arm/gripper_camera_optical_frame
```

저장된 결과가 발행되면 다음 TF 연결이 완성됩니다.

```text
arm/base_link -> arm/TCP
arm/TCP -> arm/gripper_camera_optical_frame
arm/gripper_camera_optical_frame -> arm/container_marker
```

따라서 다음 명령으로 컨테이너 마커의 `arm/base_link` 기준 6D pose를 확인할 수 있습니다.

```bash
ros2 launch arm gripper_aruco.launch.py \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  marker_id:=0 \
  marker_size_m:=0.015 \
  marker_frame_id:=arm/container_marker

ros2 run tf2_ros tf2_echo arm/base_link arm/container_marker
```

## 5. 컨테이너 추적과 Pick dry-run

컨테이너의 `DICT_5X5_50`, ID `0`, 검은 사각형 한 변 `15 mm` 마커를 추적합니다.
처음에는 로봇을 움직이지 않고 계산된 목표만 확인합니다. 다른 카메라 및 로봇 시리얼
노드는 모두 종료한 뒤 실행합니다.

```bash
ros2 launch arm container_pick.launch.py \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  marker_id:=0 \
  marker_size_m:=0.015 \
  execute_motion:=false
```

추적과 목표 pose 확인:

```bash
ros2 topic echo /arm/container_pick/status
ros2 topic echo /arm/container_pick/marker_pose
ros2 topic echo /arm/container_pick/grasp_pose
ros2 topic echo /arm/container_pick/pregrasp_pose

ros2 run rqt_image_view rqt_image_view \
  /arm/gripper_camera/aruco_annotated
```

마커를 1초 이상 안정적으로 보인 뒤 dry-run 서비스를 호출합니다.

```bash
ros2 service call /arm/pick_container std_srvs/srv/Trigger '{}'
```

성공 응답에 `DRY-RUN`이 표시되고 grasp/pregrasp pose가 발행되어야 합니다. 이
단계에서는 `/dev/ttyUSB0`을 관절 상태 읽기에만 사용하며 이동 명령은 보내지 않습니다.
현재 grasp 오프셋은 수동으로 가르친 파지 자세의 초기 측정값입니다. dry-run에서는
시각화를 위해 작업공간 밖 pose도 발행하지만 실제 실행에서는 반드시 작업공간 검사를
통과해야 합니다.
15 mm 단일 마커의 회전값은 흔들릴 수 있어 dry-run은 회전 spread를 `30도`까지,
실제 실행은 `15도`까지 허용합니다.

인터페이스:

```text
서비스: /arm/pick_container (std_srvs/Trigger)
서비스: /arm/preview_pregrasp (std_srvs/Trigger)
서비스: /arm/move_to_pregrasp (std_srvs/Trigger)
서비스: /arm/move_to_symmetric_pregrasp (std_srvs/Trigger)
서비스: /arm/stop_pick (std_srvs/Trigger)
출력: /arm/container_pick/status (std_msgs/String)
출력: /arm/container_pick/marker_pose (geometry_msgs/PoseStamped)
출력: /arm/container_pick/grasp_pose (geometry_msgs/PoseStamped)
출력: /arm/container_pick/pregrasp_pose (geometry_msgs/PoseStamped)
```

## 6. 실제 이동과 전체 Pick

설정 파일은 다음 경로에 있습니다.

```text
config/arm/container_pick.yaml
```

현재 15 mm 단일 마커는 위치는 안정적이지만 평면 회전값이 흔들리므로, 정렬된
컨테이너를 전제로 마커 XYZ에 base 좌표계 오프셋을 더하고 수동으로 가르친 고정 TCP
방향을 사용합니다. 실제 모터 명령은 `execute_motion:=true`를 명시해야만 활성화됩니다.

```yaml
execute_motion: false
allow_full_pick: true
offsets_configured: true
use_marker_rotation_for_grasp: false
grasp_offset_xyz_m: [-0.014261, -0.010785, -0.042962]
grasp_offset_rpy_deg: [-170.530, 8.370, 129.248]
pregrasp_test_keep_current_orientation: true
```

이 모드에서는 컨테이너를 평행 이동할 수 있지만 회전시키면 안 됩니다. 회전된
컨테이너까지 집으려면 더 큰 ArUco 마커나 ArUco 보드로 회전 정확도를 확보한 후
`use_marker_rotation_for_grasp: true`를 사용하고 오프셋을 다시 측정해야 합니다.

첫 실제 테스트는 속도 `5`로 pregrasp까지만 이동하고 정지합니다. 다른 카메라 및
`manual_jog`를 종료하고 실행합니다.

```bash
colcon build --symlink-install --packages-select arm
source install/setup.bash

ros2 launch arm container_pick.launch.py \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  execute_motion:=true
```

마커가 안정적으로 보이면 먼저 이동 없는 preview를 호출해 좌표와 수평거리를
확인합니다.

```bash
ros2 service call /arm/preview_pregrasp std_srvs/srv/Trigger '{}'
```

preview가 작업공간 안이고 로봇 도달 범위에 충분한 여유가 있을 때만 이동 서비스를
호출합니다.

```bash
ros2 service call /arm/move_to_pregrasp std_srvs/srv/Trigger '{}'
```

첫 위치 이동에서는 IK 실패와 갑작스러운 손목 회전을 줄이기 위해 현재 TCP 자세를
유지하고 pregrasp XYZ만 적용합니다. 이 서비스는 그리퍼를 열거나 닫지 않고, grasp로
하강하지 않으며, pregrasp에 도달하면 정지합니다. 즉시 정지:

현재 TCP 자세로 pregrasp IK를 구할 수 없으면 수동으로 가르친 고정 파지 방향으로
한 번 더 IK를 계산합니다. 두 자세가 모두 실패하면 모터 명령 없이 중단합니다.

평행 그리퍼의 180도 대칭 자세를 비교하는 별도 테스트 서비스:

```bash
ros2 service call /arm/move_to_symmetric_pregrasp \
  std_srvs/srv/Trigger '{}'
```

이 서비스는 마커 yaw 전체를 사용해 동일한 pregrasp 위치를 계산하고, 그리퍼 yaw만
180도 차이 나는 두 후보를 MoveIt plan-only로 검사합니다. 도달 가능한 후보 중 현재
관절에서 종점까지의 이동량, 전체 관절 경로 길이, planning time 순으로 선택한 궤적
하나만 실행합니다. 그리퍼 개폐와 grasp 하강은 실행하지 않습니다. 선택 과정은
`/arm/container_pick/status`에 발행됩니다.

```bash
ros2 service call /arm/stop_pick std_srvs/srv/Trigger '{}'
```

전체 Pick은 현재 자세를 유지한 pregrasp 위치 이동, pregrasp에서 파지 방향 정렬,
마커 재측정, grasp 이동, 그리퍼 닫기, 상승 순서로 동작합니다.

```bash
ros2 service call /arm/pick_container std_srvs/srv/Trigger '{}'
```

작업공간, 마커 freshness, 위치/회전 안정성, IK 관절 제한과 관절 해 변화량 검사를
통과하지 못하면 이동하지 않습니다. 탑다운 카메라를 이용한 사전 접근은 아직 이 launch에
연결하지 않았습니다.

## MoveIt2 기반 실행

직접 `pymycobot.solve_inv_kinematics()`를 호출하는 기존 launch 대신 MoveIt2의 KDL IK,
충돌 검사와 trajectory planning을 사용합니다. 최초 한 번 MoveIt을 설치합니다.

```bash
sudo apt-get install -y \
  ros-jazzy-moveit
```

빌드:

```bash
cd ~/poter_ws
colcon build --symlink-install --packages-select \
  jetcobot_description jetcobot_moveit_config arm
source install/setup.bash
```

다른 `manual_jog`, 카메라 노드와 `/dev/ttyUSB0` 사용 노드를 모두 종료한 뒤 통합
launch를 실행합니다.

```bash
ros2 launch arm container_pick_moveit.launch.py \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  video_device:=/dev/video2 \
  calibration_name:=jetcobot_eye_in_hand \
  use_node_time_for_pose:=true \
  marker_id:=0 \
  marker_size_m:=0.015 \
  serial_port:=/dev/ttyUSB0 \
  trajectory_speed:=100 \
  goal_correction_speed:=100 \
  goal_tolerance_deg:=2.5 \
  goal_timeout_sec:=15.0 \
  use_rviz:=true
```

`trajectory_speed`는 JetCobot 내부 위치 제어기가 MoveIt의 시간 기반 중간 목표를
따라가는 속도입니다. 전체 이동 속도는 MoveIt의 velocity scaling으로 제한합니다.

이 launch는 다음을 한 번에 실행합니다.

```texttext
MoveIt move_group + RViz
JetCobot FollowJointTrajectory bridge
실제 /joint_states 발행
그리퍼 서비스
그리퍼 카메라와 ArUco
저장된 Hand-Eye TF
container_pick_coordinator (motion_backend=moveit)
```

### 라즈베리파이 하드웨어와 노트북 MoveIt 분산 실행

로봇팔과 그리퍼 카메라가 라즈베리파이에 연결되어 있으면 위 일체형 launch 대신
하드웨어와 계획 노드를 나눠 실행합니다. 두 컴퓨터에서 같은 `ROS_DOMAIN_ID`를 사용하고
`ROS_LOCALHOST_ONLY=0`으로 설정합니다.

라즈베리파이에서 실제 장치 포트를 지정해 실행합니다.

```bash
cd ~/poter_ws
source install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0

ros2 launch arm container_pick_hardware.launch.py \
  camera_info_url:=config/arm/gripper_camera_info.yaml \
  video_device:=/dev/video4 \
  use_node_time_for_pose:=true \
  marker_id:=0 \
  marker_size_m:=0.015 \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=1000000 \
  trajectory_speed:=100 \
  goal_correction_speed:=100 \
  goal_tolerance_deg:=2.5 \
  goal_timeout_sec:=15.0
```

노트북에서는 로컬 카메라와 시리얼 포트를 열지 않는 원격 MoveIt launch를 실행합니다.
Hand-Eye 보정 파일과 `container_pick.yaml`은 노트북의 `config/arm`에 있어야 합니다.

```bash
cd ~/poter_ws
source install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0

ros2 launch arm container_pick_remote.launch.py \
  calibration_name:=jetcobot_eye_in_hand \
  calibration_directory:=config/arm \
  params_file:=config/arm/container_pick.yaml \
  use_rviz:=true
```

지정한 이름의 `.calib` 파일이 없으면 Hand-Eye TF가 끊어져 마커를 검출해도 안정화
샘플은 수집되지 않습니다.

서비스 호출도 노트북에서 그대로 실행합니다.

```bash
ros2 service call /arm/preview_pregrasp std_srvs/srv/Trigger '{}'
ros2 service call /arm/move_to_pregrasp std_srvs/srv/Trigger '{}'
ros2 service call /arm/pick_container std_srvs/srv/Trigger '{}'
ros2 service call /arm/stop_pick std_srvs/srv/Trigger '{}'
```

실행 전에 노트북에서 원격 하드웨어 인터페이스가 보이는지 확인합니다.

```bash
ros2 topic echo /joint_states --once
ros2 action info /arm_group_controller/follow_joint_trajectory
ros2 service type /arm/gripper/open
ros2 topic hz /arm/gripper_camera/aruco_pose
```

이동 없는 목표 확인, pregrasp 단독 이동을 차례로 검증한 후 전체 Pick을 호출합니다.

```bash
ros2 service call /arm/preview_pregrasp std_srvs/srv/Trigger '{}'
ros2 service call /arm/move_to_pregrasp std_srvs/srv/Trigger '{}'
ros2 service call /arm/pick_container std_srvs/srv/Trigger '{}'
```

`/arm/pick_container`는 요청 직후 기존 마커 샘플을 버리고 새 샘플 5개가 안정화될
때까지 최대 20초 기다립니다. 호출 순간 마지막 샘플이 3초보다 오래됐더라도 새 프레임이
들어오면 목표를 잠근 뒤 그리퍼 열기, pregrasp 이동과 정렬, 하강, 그리퍼 닫기와
상승을 실행합니다. 첫 실기기 테스트에서는 항상 `/arm/move_to_pregrasp`의 위치와
자세를 먼저 확인합니다.

완화된 실행 조건은 다음과 같습니다.

```text
마커 yaw spread: 12도 (전체 회전 spread: 10도)
컨테이너 기준 yaw 변화: 180도 (모든 방향 허용)
MoveIt 자세 허용오차: 5도
Cartesian 관절 점프: 15도
실기기 목표 관절 오차: 2.5도
실기기 최종 수렴 대기: 15초
MoveIt 실행 여유: 계획 시간 x 2 + 20초
```

물리 관절 한계, 작업공간, 충돌 검사, 위치 안정성 5 mm와 Cartesian 경로 완성도
98% 조건은 완화하지 않습니다.

Cartesian 경로가 충돌 검사에서 중단되면 코디네이터는 충돌 검사를 끈 경로를
실행하지 않고 진단에만 사용합니다. `/check_state_validity`로 해당 경로를 검사하여
최초 무효 상태, 충돌 링크/물체 쌍, 접촉 위치와 침투 깊이를 실패 로그에 출력합니다.
이 결과를 기준으로 URDF collision geometry나 접근 자세를 수정하며 목표 하강 깊이와
전체 충돌 검사를 우회하지 않습니다.

Place 수직 하강에서 자기 충돌이 검출되면 `/compute_ik`에 서로 다른 관절 seed를
전달하여 동일한 TCP XYZ, orientation과 최종 깊이에 대한 대체 IK branch를 찾습니다.
각 후보는 가상 preplace 상태에서 최종 place까지의 Cartesian 경로를 충돌 검사하고,
`place_branch_min_fraction` 이상인 후보만 현재 자세에서 접근 계획을 세워 실행합니다.
실제 preplace 도착 후에는 현재 관절 상태로 수직 경로를 다시 검사하며, 검사를 다시
통과한 경우에만 하강합니다. `place_ik_branch_attempts`는 seed 개수,
`place_ik_timeout_sec`는 후보 하나의 IK 제한 시간입니다.

즉시 정지:

```bash
ros2 service call /arm/stop_pick std_srvs/srv/Trigger '{}'
```

MoveIt trajectory 실행 인터페이스:

```text
Action: /arm_group_controller/follow_joint_trajectory
Topic: /joint_states
Service: /arm/gripper/open
Service: /arm/gripper/close
Service: /arm/stop_robot
```

## 두 위치 순차 ArUco Pick & Place

`container_pick_place_moveit.launch.py`는 pick/place 마커가 한 화면에 같이 보일 필요가
없는 순차 인식 방식입니다. `pick_marker_id`로 지정한 ID가 pick이고,
`place_marker_id`는 나머지 place 마커입니다. 첫 촬영 위치에서는 두 ID 중 현재
안정적으로 검출되는 하나를 base 좌표로 저장합니다. 그다음 설정된 두 번째 촬영
자세로 이동하고, 반대 ID의 새 영상 샘플을 다시 안정화한 뒤 기존 MoveIt pick/place를
실행합니다. 첫 화면에 두 마커가 우연히 함께 보여도 하나를 먼저 저장하고 두 번째
촬영 자세에서 나머지를 다시 측정합니다.

실제 장비에서 안전한 두 촬영 자세의 J1~J6 관절각(degree)을 측정해
`config/arm/container_pick_place.yaml`에 입력해야 합니다.

```yaml
first_observation_pose_configured: true
first_observation_joint_angles_deg: [J1, J2, J3, J4, J5, J6]
second_observation_pose_configured: true
second_observation_joint_angles_deg: [J1, J2, J3, J4, J5, J6]
observation_joint_tolerance_deg: 1.0
startup_joint_state_timeout_sec: 20.0
```

두 `*_configured` 값 중 하나라도 `false`이면 로봇 이동 서비스가 거부됩니다.
launch만 실행했을 때는 로봇이 움직이지 않습니다. `/arm/pick_and_place` 서비스를
호출하면 완전한 `/joint_states`를 기다린 다음, MoveIt이 관절 목표 경로의 관절
한계와 충돌을 검사하고 `first_observation_joint_angles_deg`로 이동합니다.

```bash
ros2 launch arm container_pick_place_moveit.launch.py \
  pick_marker_id:=1 \
  place_marker_id:=0 \
  params_file:=config/arm/container_pick_place.yaml

ros2 service call /arm/pick_and_place std_srvs/srv/Trigger '{}'
```

동작 순서는 다음과 같습니다.

```text
launch 시작 → 이동 없이 서비스 대기
→ /arm/pick_and_place 호출
→ first_observation_joint_angles_deg로 이동
→ 첫 위치에서 ID 1 또는 ID 0 안정화
→ 해당 base-frame 목표 좌표 저장
→ second_observation_joint_angles_deg로 이동
→ 반대 ID의 새 좌표 안정화
→ pick_marker_id 좌표에서 집기
→ 다른 ID 좌표에 놓기
```

각 촬영 단계에서는 마커 검출과 자세 안정성만 판단합니다. 두 마커를 모두 저장한 뒤
pick, pregrasp, place, preplace 목표 전체의 작업공간을 검사하고, 하나라도 범위를
벗어나면 실제 Pick & Place 이동 전에 중단합니다.

`target_radial_offset_m`는 marker-yaw와 XYZ offset 계산이 끝난 Pick/Place 목표를
base origin 기준 XY 반경 방향으로 이동합니다. `0.020`이면 방향과 Z를 유지하면서
Pick과 Place 목표를 각각 base origin에서 정확히 20 mm 더 멀리 배치합니다.

상태는 `/arm/container_pick/status`, 즉시 정지는 `/arm/stop_pick`을 사용합니다.

## 수동 TCP Jog

```text
W/S: X +/-       A/D: Y +/-       R/F: Z +/-
U/O: RX +/-      I/K: RY +/-      J/L: RZ +/-
1/2/3: 이동 단위 1/3/5 mm 또는 deg
T: 5초간 전체 관절 토크 해제 후 현재 자세에서 자동 복구
E: 서보 상태 재확인
P: 현재 TCP와 관절각 출력
Space: 즉시 정지
Q 또는 ESC: 종료
```

각 목표는 IK 해, 관절 제한과 이전 해와의 최대 관절 변화량을 검사한 후 실행합니다.

## 기존 고정 높이 Pick/Place 검증 노드

이 노드는 고정 촬영 자세와 고정 높이 평면의 2D homography를 사용하는 과거 검증
코드입니다. 높이가 달라지는 새 자동 집기 경로에는 사용하지 않습니다.

```bash
ros2 run arm click_pick_place --ros-args \
  -p camera_path:=/dev/video4 \
  -p serial_port:=/dev/ttyUSB0
```

이 노드는 기존 고정 사진 자세와 2D 보정값을 사용하므로 새 탑다운/Hand-Eye 자동
파이프라인과 동시에 실행하지 않습니다.

## 다음 구현 단계

Hand-Eye 결과의 반복 오차를 검증한 후 다음 노드를 추가합니다.

1. 탑다운 영상에서 대상과 그리퍼 카메라 탐색 영역 결정
2. 대상별로 정의된 안전한 사전 접근 자세로 이동
3. ArUco 오차를 이용한 제한 속도 폐루프 XYZ/회전 보정
4. 정렬 허용 오차를 만족한 경우에만 기존 Pick 동작 실행
5. 마커 ID별 Pick/Place, 적층 높이와 작업 결과 검증

이 구조는 깊이 보정뿐 아니라 컨테이너 방향 정렬, 여러 마커 ID 구분, 적층 높이 판단,
Pick 이후 마커 소실 확인과 실패 복구까지 확장할 수 있습니다.

## 안전 사항

- Hand-Eye 결과를 검증하기 전에는 자동 이동에 사용하지 않습니다.
- 카메라 내부 보정과 `marker_size_m`가 틀리면 깊이 값도 틀립니다.
- 시리얼 포트를 직접 사용하는 arm 노드는 한 번에 하나만 실행합니다.
- 카메라 프레임이 끊기거나 TF가 오래되면 이동을 즉시 중단해야 합니다.
- 작업 범위, Z 최저 높이와 1회 보정 이동량에 제한을 둬야 합니다.
