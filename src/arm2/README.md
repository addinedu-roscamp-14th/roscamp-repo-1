# arm2

두 번째 JetCobot용 ROS 2 패키지입니다. 기존 `arm` 패키지는 변경하지 않고,
노드와 launch 파일, ROS 토픽 및 TF 프레임을 `arm2` 이름으로 분리했습니다.

## 이름 규칙

- 패키지: `arm2`
- 실행 파일: `arm2_*`
- launch 파일: `arm2_*.launch.py`
- ROS 네임스페이스: `/arm2`
- TF 프레임: `arm2/base_link`, `arm2/TCP`
- 설정 경로: `config/arm2`

`package.xml`, `setup.py`, `setup.cfg`, `README.md`, `__init__.py`는 ROS와
Python이 요구하는 표준 파일명이므로 접두사를 붙이지 않습니다.

## 빌드

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm2
source install/setup.bash
```

## 현재 검증 설정

아래 값은 2번 로봇팔에서 현재 가장 안정적으로 동작한 기준 설정입니다.

| 구분 | 현재 기준값 |
| --- | --- |
| 카메라 내부 보정 | `config/arm2/arm2_gripper_camera_info_v2.yaml` |
| Hand-Eye 보정 | `config/arm2/arm2_jetcobot_eye_in_hand_charuco_5x5_v2.calib` |
| Hand-Eye 보드 | `DICT_4X4_50`, 5x5, checker 20mm, marker 15mm |
| 파지 마커 | `DICT_5X5_50`, ID 0, 26mm |
| 파지 위치 오프셋 | `[0.006879, -0.002075, -0.016814]` m |
| 파지 자세 | `[-175.0, 0.0, -94.071]` deg |
| 기준 마커 yaw | `2.349` deg |
| 최종 정지 높이 | 측정 파지점보다 20mm 위 |

`config/arm2/arm2_container_pick.yaml`에서는 다음 값이 현재 기준입니다.

```yaml
allow_full_pick: true
offsets_configured: true
grasp_orientation_mode: marker_yaw
rotate_grasp_offset_with_marker_yaw: true
grasp_offset_xyz_m: [0.006879, -0.002075, -0.016814]
grasp_offset_rpy_deg: [-175.0, 0.0, -94.071]
reference_marker_yaw_deg: 2.349
grasp_stop_above_m: 0.02
```

`marker_yaw` 모드는 학습한 roll/pitch를 유지해 그리퍼를 수직에 가깝게 두고,
컨테이너 마커의 yaw만 따라가도록 합니다.

## 1. 장치 확인

2번 로봇팔의 카메라와 시리얼 장치 경로를 먼저 확인합니다. 아래 예시는 카메라
`/dev/video2`, 로봇팔 `/dev/ttyUSB0`을 사용하지만 실제 연결 결과에 맞게 launch
인자로 변경할 수 있습니다.

```bash
v4l2-ctl --list-devices
ls -l /dev/video* /dev/ttyUSB*
```

1번 팔과 2번 팔을 동시에 사용할 때는 반드시 서로 다른 장치 경로를 지정합니다.

## 2. 그리퍼 카메라 내부 보정

Hand-Eye 보정 전에 2번 팔 카메라의 내부 파라미터를 별도로 구합니다. 1번 팔의
`config/arm/gripper_camera_info.yaml`은 사용하지 않습니다.

터미널 1:

```bash
source install/setup.bash

ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __ns:=/arm2/gripper_camera \
  -p video_device:=/dev/video2 \
  -p image_size:="[640,480]" \
  -p time_per_frame:="[1,10]" \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8 \
  -p camera_frame_id:=arm2/gripper_camera_optical_frame
```

터미널 2에서 사용하는 체커보드의 내부 코너 수와 한 칸 길이에 맞춰 실행합니다.

```bash
source install/setup.bash

ros2 run camera_calibration cameracalibrator \
  --size 10x7 \
  --square 0.015 \
  --no-service-check \
  --ros-args \
  -r image:=/arm2/gripper_camera/image_raw
```

저장한 결과는 다음 파일로 관리합니다.

```text
config/arm2/arm2_gripper_camera_info_v2.yaml
```

`cameracalibrator`가 만든 `/tmp/calibrationdata.tar.gz`의 `ost.yaml`을 위 이름으로
옮겨 사용합니다.

## 3. 두 번째 로봇 설정

두 번째 팔에 연결된 장치 경로를 실행 시 지정합니다.

```bash
ros2 launch arm2 arm2_container_pick_hardware.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=config/arm2/arm2_gripper_camera_info_v2.yaml \
  serial_port:=/dev/ttyUSB0 \
  trajectory_speed:=100
```

수동 조작:

```bash
ros2 run arm2 arm2_manual_jog --ros-args \
  -p serial_port:=/dev/ttyUSB0
```

## 4. Hand-Eye 캘리브레이션

두 번째 팔은 카메라 장착 위치와 파지 오프셋이 첫 번째 팔과 다르므로 별도로
캘리브레이션해야 합니다. 현재 기준은 ChArUco 보드를 이용한 수동 샘플 수집입니다.

```bash
ros2 launch arm2 arm2_handeye_charuco_calibration.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=config/arm2/arm2_gripper_camera_info_v2.yaml \
  dictionary:=DICT_4X4_50 \
  squares_x:=5 \
  squares_y:=5 \
  square_length_m:=0.020 \
  marker_length_m:=0.015 \
  legacy_pattern:=true \
  minimum_charuco_corners:=6 \
  max_reprojection_error_px:=3.0 \
  detection_rate_hz:=5.0 \
  use_node_time_for_pose:=true \
  name:=arm2_jetcobot_eye_in_hand_charuco_5x5_v2 \
  calibration_directory:=config/arm2
```

별도 터미널에서 2번 팔만 수동 조작합니다. 보드는 고정하고 로봇팔만 움직이며,
위치와 자세가 겹치지 않도록 15~25개 샘플을 `Take Sample`로 수집합니다.

```bash
source install/setup.bash

ros2 run arm2 arm2_manual_jog --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p speed:=10
```

검출 영상과 TF를 확인합니다.

```bash
ros2 run rqt_image_view rqt_image_view \
  /arm2/gripper_camera/charuco_annotated
ros2 run tf2_ros tf2_echo \
  arm2/gripper_camera_optical_frame arm2/handeye_target
```

캘리브레이션 완료 후 생성되는 파일은 다음 이름으로 관리합니다.

```text
config/arm2/arm2_jetcobot_eye_in_hand_charuco_5x5_v2.calib
```

현재 arm2 기준에서는 자동 MoveIt 샘플러를 사용하지 않습니다. ChArUco 보드를
고정한 상태에서 `arm2_manual_jog`로 자세를 바꾸고, 로봇이 완전히 정지한 뒤
easy_handeye2의 `Take Sample`을 직접 누르는 절차를 사용합니다.

파지 오프셋까지 측정한 뒤
`config/arm2/arm2_container_pick.yaml`의 값을 갱신하고
`offsets_configured: true`로 변경해야 실제 파지가 허용됩니다.

### 파지 오프셋 측정

터미널 1에서 측정 전용 launch를 실행합니다. 이 launch는 로봇 시리얼 포트를 열지
않으므로 `arm2_manual_jog`와 충돌하지 않습니다.

```bash
source install/setup.bash

ros2 launch arm2 arm2_grasp_offset_calibration.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=config/arm2/arm2_gripper_camera_info_v2.yaml \
  marker_id:=0 \
  marker_size_m:=0.026 \
  dictionary:=DICT_5X5_50 \
  calibration_name:=arm2_jetcobot_eye_in_hand_charuco_5x5_v2 \
  calibration_directory:=config/arm2 \
  output_yaml:=config/arm2/arm2_container_pick.yaml \
  max_offset_std_m:=0.006
```

터미널 2에서만 로봇 시리얼 포트를 열고 수동 조작합니다.

```bash
source install/setup.bash

ros2 run arm2 arm2_manual_jog --ros-args \
  -p serial_port:=/dev/ttyUSB0
```

컨테이너를 움직이지 않도록 고정하고 ID 0 마커가 보이는 자세에서 2초 이상
정지한 뒤 마커의 `base_link` 위치를 먼저 고정합니다.

```bash
ros2 service call /arm2/capture_grasp_marker std_srvs/srv/Trigger '{}'
```

이후 컨테이너는 절대 움직이지 않고, RViz의 MotionPlanning으로 TCP를 컨테이너를
정확히 잡을 자세에 놓습니다. 이 단계에서는 마커가 카메라 밖으로 나가도 됩니다.
TCP가 2초 이상 정지한 다음 오프셋을 저장합니다.

```bash
ros2 service call /arm2/capture_grasp_offset std_srvs/srv/Trigger '{}'
```

이 서비스는 `grasp_offset_xyz_m`, `grasp_offset_rpy_deg`,
`reference_marker_yaw_deg`를 저장합니다. 새로 측정한 경우에는 통합 launch를 다시
시작하고 pregrasp를 먼저 검증한 뒤 `allow_full_pick: true`로 설정합니다.

현재 검증값인 `rotate_grasp_offset_with_marker_yaw: true`는 컨테이너가 회전하면
측정한 XY 오프셋도 마커 yaw에 맞춰 함께 회전시킵니다. 컨테이너 방향이 달라져도
같은 상대 파지점을 유지하기 위한 설정입니다.

```bash
ros2 service call /arm2/move_to_pregrasp std_srvs/srv/Trigger '{}'
```

pregrasp가 컨테이너 중심 위의 안전한 높이에 도달하는 것을 확인한 후에만
`config/arm2/arm2_container_pick.yaml`의 `allow_full_pick`을 `true`로 변경하고 launch를
다시 시작합니다.

최종 하강 정지 높이만 올릴 때는 `grasp_stop_above_m`을 사용합니다. 예를 들어
`0.02`는 측정한 파지 자세보다 20mm 위에서 멈추며, 검증한 pregrasp 위치는 바꾸지
않습니다.

## 5. 파지와 적재 실행

```bash
source install/setup.bash

ros2 launch arm2 arm2_container_pick_moveit.launch.py \
  camera_info_url:=config/arm2/arm2_gripper_camera_info_v2.yaml \
  video_device:=/dev/video2 \
  calibration_name:=arm2_jetcobot_eye_in_hand_charuco_5x5_v2 \
  params_file:=config/arm2/arm2_container_pick.yaml \
  use_node_time_for_pose:=true \
  marker_id:=0 \
  marker_size_m:=0.026 \
  serial_port:=/dev/ttyUSB0 \
  trajectory_speed:=100 \
  goal_correction_speed:=100 \
  goal_tolerance_deg:=2.5 \
  goal_timeout_sec:=15.0 \
  use_rviz:=true
```

```bash
ros2 service call /arm2/pick_container std_srvs/srv/Trigger '{}'
```

ID 0 컨테이너를 ID 1 위에 적재할 때는 이동 전 preview를 먼저 확인합니다.

```bash
ros2 service call /arm2/preview_stack std_srvs/srv/Trigger '{}'
ros2 service call /arm2/stack_container std_srvs/srv/Trigger '{}'
ros2 topic echo /arm2/container_pick/status
```

`arm2_container_pick_moveit.launch.py`는 ID 0과 ID 1을 동시에 검출합니다. 실제 동작
전에는 `config/arm2/arm2_container_pick.yaml`에서 2번 팔의 작업공간, 파지 오프셋,
기준 마커 yaw를 측정하고 `offsets_configured: true`로 변경해야 합니다.

## MoveIt namespace

`arm2_moveit.launch.py`는 공용 JetCobot 설정을 불러온 뒤 두 번째 로봇의 링크를
`arm2/`로 접두사 처리하고 MoveIt 전체를 `/arm2` namespace에서 실행합니다.

| 구분 | arm | arm2 |
| --- | --- | --- |
| MoveIt goal | `/move_action` | `/arm2/move_action` |
| Cartesian path | `/compute_cartesian_path` | `/arm2/compute_cartesian_path` |
| Trajectory execution | `/execute_trajectory` | `/arm2/execute_trajectory` |
| Controller | `/arm_group_controller/follow_joint_trajectory` | `/arm2/arm_group_controller/follow_joint_trajectory` |
| Joint state | `/joint_states` | `/arm2/joint_states` |
| Base/TCP | `base_link`, `TCP` | `arm2/base_link`, `arm2/TCP` |

namespace 확인:

```bash
ros2 action list | grep arm2
ros2 service list | grep arm2
ros2 topic info /arm2/joint_states -v
ros2 run tf2_ros tf2_echo arm2/base_link arm2/TCP
```

`arm`과 `arm2`는 같은 `ROS_DOMAIN_ID`에서 동시에 실행할 수 있습니다. 실제 장치가
서로 다른 `/dev/video*`, `/dev/ttyUSB*`를 사용하도록 실행 인자를 지정해야 합니다.
