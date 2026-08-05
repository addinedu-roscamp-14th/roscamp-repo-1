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

## 1. 장치 확인

2번 로봇팔의 카메라와 시리얼 장치 경로를 먼저 확인합니다. 아래 예시는 카메라
`/dev/video2`, 로봇팔 `/dev/ttyUSB1`을 사용하지만 실제 연결 결과에 맞게 launch
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
config/arm2/arm2_gripper_camera_info.yaml
```

## 3. 두 번째 로봇 설정

두 번째 팔에 연결된 장치 경로를 실행 시 지정합니다.

```bash
ros2 launch arm2 arm2_container_pick_hardware.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=config/arm2/arm2_gripper_camera_info.yaml \
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
캘리브레이션해야 합니다. 기존 `config/arm/*.calib` 파일은 복사하지 않았습니다.

```bash
source install/setup.bash

ros2 launch arm2 arm2_handeye_charuco_calibration.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=config/arm2/arm2_gripper_camera_info.yaml \
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
  name:=arm2_jetcobot_eye_in_hand_charuco_5x5_v4 \
  calibration_directory:=config/arm2
```

별도 터미널에서 2번 팔만 수동 조작하고, 보드가 고정된 상태에서 위치와 자세가
겹치지 않도록 샘플을 수집합니다.

```bash
source install/setup.bash

ros2 run arm2 arm2_manual_jog --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p speed:=10
```

검출 영상과 TF를 확인합니다.

```bash
ros2 run rqt_image_view rqt_image_view \
  /arm2/gripper_camera/aruco_annotated
ros2 run tf2_ros tf2_echo \
  arm2/gripper_camera_optical_frame arm2/handeye_target
```

캘리브레이션 완료 후 생성되는 파일은 다음 이름으로 관리합니다.

```text
config/arm2/arm2_jetcobot_eye_in_hand.calib
```

파지 오프셋까지 측정한 뒤
`config/arm2/arm2_container_pick.yaml`의 값을 갱신하고
`offsets_configured: true`로 변경해야 실제 파지가 허용됩니다.

### 파지 오프셋 측정

통합 MoveIt launch를 실행한 상태에서 별도 터미널에 측정 노드를 실행합니다.

```bash
source install/setup.bash

ros2 run arm2 arm2_grasp_offset_calibrator --ros-args \
  -p output_yaml:=config/arm2/arm2_container_pick.yaml
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
`reference_marker_yaw_deg`를 저장하고 `offsets_configured`만 `true`로 변경합니다.
`allow_full_pick`은 `false`로 유지됩니다. 통합 launch를 다시 시작한 뒤 pregrasp만
먼저 검증합니다.

```bash
ros2 service call /arm2/move_to_pregrasp std_srvs/srv/Trigger '{}'
```

pregrasp가 컨테이너 중심 위의 안전한 높이에 도달하는 것을 확인한 후에만
`config/arm2/arm2_container_pick.yaml`의 `allow_full_pick`을 `true`로 변경하고 launch를
다시 시작합니다.

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
    goal_tolerance_deg:=3.5 \
    goal_timeout_sec:=15.0 \
    use_rviz:=true






ros2 launch arm2 arm2_container_pick_moveit.launch.py \
    camera_info_url:=config/arm2/arm2_gripper_camera_info.yaml \
    video_device:=/dev/video2 \
    calibration_name:=arm2_jetcobot_eye_in_hand_charuco_5x5_v4 \
    params_file:=config/arm2/arm2_container_pick.yaml \
    use_node_time_for_pose:=true \
    marker_id:=0 \
    marker_size_m:=0.020 \
    serial_port:=/dev/ttyUSB0 \
    trajectory_speed:=100 \
    goal_correction_speed:=100 \
    goal_tolerance_deg:=3.5 \
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

### 베이스 좌표계 고정 보정

`config/arm2/arm2_container_pick.yaml`에서 파지점 보정은 두 종류로 나뉩니다.

```yaml
grasp_offset_xyz_m: [0.006879, -0.002075, -0.036814]
pick_correction_xyz_m: [0.0, -0.007925, 0.0]
id_transfer_pick_correction_xyz_m: [-0.02, -0.01, -0.02]
place_correction_xyz_m: [0.0, 0.0, 0.0]
saved_destination_correction_xyz_m: [-0.02, -0.01, -0.005]
id_transfer_correction_xyz_m: [0.02, 0.0, 0.0]
trailer_correction_xyz_m: [-0.05, 0.0, 0.0]
```

- `grasp_offset_xyz_m`: 마커 기준 자세에서 가르친 파지 오프셋입니다.
  현재 arm2 설정은 `rotate_grasp_offset_with_marker_yaw: false`이므로 그리퍼 yaw만
  컨테이너를 추종하고 XYZ 오프셋 방향은 `arm2/base_link`에 고정됩니다.
- `pick_correction_xyz_m`: `marker_yaw`/`marker_full` 집기 목표에 마커 축
  기준으로 더하며 마커 yaw와 함께 회전합니다. X는 빨간 축, Y는 초록 축입니다.
  `fixed` 모드에서만 `arm2/base_link` 축 기준으로 적용됩니다.
- `id_transfer_pick_correction_xyz_m`: `/arm2/transfer_by_id`의 집기에만
  적용됩니다. 현재 값은 초록 축 반대 10 mm이며 기존 X/Z 집기 보정을
  유지합니다.
- `place_correction_xyz_m`: 놓기 접근점과 최종 놓기점에만 목적지 마커 축
  기준으로 더합니다. X는 빨간 축, Y는 초록 축이며 마커 yaw와 함께 회전합니다.
- `saved_destination_correction_xyz_m`: `/arm2/transfer_to_a1_1`부터
  `/arm2/transfer_to_a3_2`까지 저장 목적지 이송에만 적용됩니다.
  `[-0.02, -0.01, -0.005]`는 빨간 축 반대 20 mm, 초록 축 반대 10 mm,
  Z 아래 5 mm입니다.
- `id_transfer_correction_xyz_m`: `/arm2/transfer_by_id`로 컨테이너 사이를
  옮길 때만 적용됩니다. `[-0.03, 0.0, 0.0]`은 목적지 마커의 빨간 축
  반대 방향으로 30 mm 이동합니다.
- `trailer_correction_xyz_m`: `/arm2/load_id0_to_trailer`부터
  `/arm2/load_id8_to_trailer`까지의 트레일러 놓기에만 적용됩니다. 따라서
  `[-0.05, 0.0, 0.0]`은 ID 9/10 마커가 회전해도 항상 빨간 축 반대 방향으로
  50 mm 이동합니다.
- `container_yaw_symmetry_deg: 180.0`: 직사각형 컨테이너의 0도와 180도를
  동일한 파지 자세와 동일한 XYZ 보정 방향으로 처리합니다. 45도와 90도
  회전은 그대로 추종합니다.

예를 들어 모든 컨테이너 자세에서 로봇 베이스의 `-Y` 방향으로 10 mm 보정하려면:

```yaml
pick_correction_xyz_m: [0.0, -0.01, 0.0]
```

카메라 또는 ArUco 검출 프로세스가 종료되면 launch가 2초 후 자동으로 다시
실행합니다. 카메라가 분리된 동안에는 재실행을 반복하며, 같은 `video_device`로
다시 연결되면 영상과 마커 인식을 재개합니다.

YAML을 수정했다면 launch를 완전히 종료한 뒤 다시 실행해야 합니다. launch는
`params_file`을 절대경로로 확인하며, 시작 로그의 `Loaded grasp tuning`에서 실제
적용값을 출력합니다.

RQt Parameter Reconfigure에서 위 파라미터를 바꾸는 경우에는 로봇이 정지한
상태에서만 즉시 적용됩니다. 실행 중인 동작이 있거나 오프셋 절댓값이 허용 범위를
넘으면 변경을 거부합니다. RQt 변경은 YAML 파일에 저장되지 않으므로 검증이 끝난
값은 `config/arm2/arm2_container_pick.yaml`에도 직접 반영해야 합니다.
`allow_full_pick`과 `offsets_configured` 체크 상태도 실행 잠금에 즉시 반영됩니다.

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

## A-1/A-2/A-3 세부 목적지 저장 후 개별 이송

터미널 1에서 통합 launch를 계속 실행합니다. 컨테이너 ID는 0~8이고,
트레일러는 ID 9 또는 ID 10입니다. 세부 목적지는 다음과 같습니다.

- A-1-1 = ID 11
- A-1-2 = ID 12
- A-2-1 = ID 13
- A-2-2 = ID 14
- A-3-1 = ID 15
- A-3-2 = ID 16

```bash
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
    trajectory_speed:=100 \
    goal_correction_speed:=100 \
    goal_tolerance_deg:=3.5 \
    goal_timeout_sec:=15.0 \
    use_rviz:=true
```

launch 직후 한 번만 목적지 스캔을 실행합니다. 로봇은
`HOME → A-1 → A-2 → A-3 → A-2 → A-1 → HOME` 순서로 이동합니다.
목적지 스캔과 저장은 먼저 `HOME → A-1 → A-2 → A-3` 구간에서 실행합니다.
ID 11~16이 모두 저장됐으면 `A-3 → A-2 → A-1 → HOME` 복귀 구간에서는
다시 스캔하지 않습니다. 누락된 ID가 있을 때만 복귀 자세에서 추가 스캔하며,
복귀 도중 모든 필수 ID가 저장되면 이후 자세부터 스캔을 중지합니다.
각 자세에서 이동 중 샘플을 버리고 정지 상태로 ID 11, 12, 13, 14, 15, 16을
1초 이상 안정화한 뒤 최초 자세를 저장합니다. 이동 중 ID 9 또는 ID 10
트레일러가 안정적으로 보이면 해당 자세도 저장합니다.
ID 11~16이 모두 저장되면 트레일러를 찾지 못했더라도 홈으로 복귀합니다.
저장값은 같은 launch가 실행되는 동안 다시 갱신되지 않습니다.

```bash
ros2 service call /arm2/scan_destinations std_srvs/srv/Trigger "{}"
```

이후 각 터미널에서 필요한 목적지 서비스를 호출합니다. 각 호출은 컨테이너 ID
0~8 중 하나를 찾을 때까지 J1을 스캔하고, 발견 시 1초 정지해 현재 컨테이너
위치를 저장한 뒤 지정 목적지로 옮기고 홈으로 복귀합니다.

세부구역별 서비스:

```bash
ros2 service call /arm2/transfer_to_a1_1 std_srvs/srv/Trigger "{}"
ros2 service call /arm2/transfer_to_a1_2 std_srvs/srv/Trigger "{}"
ros2 service call /arm2/transfer_to_a2_1 std_srvs/srv/Trigger "{}"
ros2 service call /arm2/transfer_to_a2_2 std_srvs/srv/Trigger "{}"
ros2 service call /arm2/transfer_to_a3_1 std_srvs/srv/Trigger "{}"
ros2 service call /arm2/transfer_to_a3_2 std_srvs/srv/Trigger "{}"
```

### 컨테이너 ID 0~8을 트레일러 ID 9 또는 ID 10에 다시 적재

아래 서비스는 호출할 때마다 선택한 컨테이너 ID와 트레일러 ID 9·10을
J1 스캔으로 찾습니다. 선택한 컨테이너와 트레일러 둘 중 하나가 안정적으로
저장되면 두 좌표를 잠그고, 컨테이너를 집어 선택된 트레일러에 적재한 뒤
홈으로 복귀합니다. 두 트레일러가 동시에 보이면 ID 9를 우선합니다. 이 스캔은
기존 ID 11~16 목적지 저장값을 초기화하거나 갱신하지 않습니다.

```bash
ros2 service call /arm2/load_id0_to_trailer std_srvs/srv/Trigger "{}"
ros2 service call /arm2/load_id1_to_trailer std_srvs/srv/Trigger "{}"
ros2 service call /arm2/load_id2_to_trailer std_srvs/srv/Trigger "{}"
ros2 service call /arm2/load_id3_to_trailer std_srvs/srv/Trigger "{}"
ros2 service call /arm2/load_id4_to_trailer std_srvs/srv/Trigger "{}"
ros2 service call /arm2/load_id5_to_trailer std_srvs/srv/Trigger "{}"
ros2 service call /arm2/load_id6_to_trailer std_srvs/srv/Trigger "{}"
ros2 service call /arm2/load_id7_to_trailer std_srvs/srv/Trigger "{}"
ros2 service call /arm2/load_id8_to_trailer std_srvs/srv/Trigger "{}"
```

각 호출은 별도의 터미널에서 실행할 수 있지만 로봇 동작은 한 번에 하나만
허용되므로 이전 적재와 홈 복귀가 끝난 뒤 다음 서비스를 호출해야 합니다.

### 외부 연동용 ID-to-ID 이송

외부 프로그램은 `/arm2/transfer_by_id` 요청의 두 필드만 채우면 됩니다.
`source_id` 컨테이너를 집어 `destination_id` 컨테이너 위에 놓습니다. 두 ID는
0~8 범위여야 하고 서로 달라야 합니다. 호출 후 두 ID만 스캔하고, 각각 0.5초
안정 위치를 저장·잠근 다음 기존과 같은 집기와 적재 동작을 수행합니다.

예를 들어 ID 8을 ID 5 위에 놓으려면:

```bash
ros2 service call /arm2/transfer_by_id \
  arm2_interfaces/srv/TransferById \
  "{source_id: 8, destination_id: 5}"
```

로봇 동작은 한 번에 하나만 실행할 수 있으므로 한 세부구역 작업이 끝나기 전에
다른 이송 서비스를 호출하면 요청이 거부됩니다. launch를 재시작하면 저장 pose가
초기화되므로 `/arm2/scan_destinations`를 다시 호출해야 합니다.

같은 세부 목적지 서비스를 반복 호출하면 세부구역별로 최대 3층까지 쌓습니다.
컨테이너 높이는 35mm이며, 여섯 세부구역은 서로 독립적으로 1층,
2층(+35mm), 3층(+70mm)을 기억합니다. 네 번째 요청은 거부됩니다. 실제
적재물을 치운 뒤 카운터만 다시 1층으로 초기화하려면 다음 서비스를 호출합니다.

```bash
ros2 service call /arm2/reset_stack_level std_srvs/srv/Trigger "{}"
```

### Pregrasp 폐루프 미세 보정

저장 목적지 이송은 MoveIt으로 pregrasp에 도착한 뒤, 선택된 컨테이너 ID의
마커를 다시 측정해 XY를 작은 Cartesian 단계로 보정합니다. 손목을 최종 yaw로
정렬하면 상단 마커가 시야에서 사라지므로, 보정 중에는 현재 손목 자세를 유지하고
마지막 관측의 yaw로 한 번만 정렬합니다. Z는 보정하지 않고 마지막으로 잠근 마커
pose에서 계산한 파지점까지 기존 수직 하강을 사용합니다.

기본 설정은 다음과 같습니다.

```yaml
visual_servo_enabled: false
visual_servo_samples: 10
visual_servo_xy_tolerance_m: 0.002
visual_servo_yaw_tolerance_deg: 2.0
visual_servo_xy_gain: 0.6
visual_servo_yaw_gain: 0.6
visual_servo_max_xy_step_m: 0.005
visual_servo_max_yaw_step_deg: 2.0
visual_servo_max_initial_error_m: 0.02
visual_servo_max_iterations: 5
visual_servo_required_consecutive_successes: 3
visual_servo_timeout_sec: 10.0
visual_servo_marker_loss_timeout_sec: 2.0
visual_servo_settle_sec: 0.6
```

현재 손목 카메라는 pregrasp에서 상단 마커를 잃으므로 기본값은 비활성화되어
있습니다. 이 상태에서는 초기 위치의 안정 pose를 잠근 뒤 마커가 사라져도 저장
좌표로 기존 파지를 계속합니다. 마커가 pregrasp에서도 보이는 배치에서만
`visual_servo_enabled: true`로 활성화합니다.

활성화한 경우 각 반복은 이동 전 샘플을 폐기하고 새로운 10개 샘플만 사용합니다. XY 오차가
20mm를 넘거나, 마커를 2초 동안 잃거나, 정규화한 오차가 두 번 연속 증가하거나,
5회 안에 연속 3회 허용 오차를 만족하지 못하면 하강하지 않고 실패 복귀합니다.
초기 위치에서 ID 0~8 중 하나를 잠근 경우 해당 ID의 TF만 pregrasp 보정에
사용합니다.

## 카메라 재시작 반복성 확인

2026-07-30에 동일 자세에서 3회 측정한 보정 영상의 세션 중심 중앙값
`U=300.350 px`, `V=293.686 px`를 기준으로 사용합니다. 이 값은 CameraInfo나
영상을 이동시키는 보정값이 아니라 재시작 전후 변화를 판정하는 고정 기준입니다.

로봇팔을 움직이지 않고 카메라와 진단 노드만 실행합니다.

```bash
ros2 launch arm2 arm2_camera_repeatability.launch.py
```

200개 검출 표본마다 `/arm2/gripper_camera/repeatability`에 JSON 결과를
발행합니다.

```bash
ros2 topic echo /arm2/gripper_camera/repeatability
```

결과에는 보정 좌표계 중심, 기준 대비 `delta_u_px`, `delta_v_px`, 유클리드
`delta_px`, 프레임 내부 RMS, 마커 크기로 계산한 `mm_per_px`와 `delta_mm`가
포함됩니다. `delta_px <= 2`는 `stable`, `2 < delta_px <= 5`는 `warning`,
5px 초과는 `unstable`로 판정합니다. CameraInfo가 실행 중 변경되면 현재 표본
묶음을 폐기하고 경고합니다.

## Arm2 home 관절 명령 보상

2026-07-31에 기존 home 명령을 반복했을 때 실제 J4가 목표보다 약 `-2.02°`에
고정되는 현상을 확인했습니다. 같은 명령을 10회 재전송해도 줄어들지 않았으며,
관측 바이어스를 역보상한 명령으로 A-1에서 5회 동일 방향 접근했을 때 J1~J5의
목표 오차가 최대 `0.09°`, J6 반복 범위가 `0.35°`로 측정됐습니다.

따라서 `config/arm2/arm2_container_pick.yaml`의
`home_joint_angles_deg`는 실제로 기존 물리 home 자세에 도착하도록 다음 보상
명령을 사용합니다.

```yaml
home_joint_angles_deg: [93.86, 13.27, -25.91, -59.85, 3.15, -43.33]
```

이 값은 startup/shutdown home에만 적용합니다. 작업공간 전체에서 일정한 관절
오프셋이라고 검증되지 않았으므로 MoveIt 궤적 목표에는 전역으로 더하지 않습니다.

### MoveIt 최종 J4 제한 폐루프 보정

A-1/A-2/A-3에서 각 3회 측정한 J4의 `목표-실제` 평균 오차는 각각
`1.11°`, `1.23°`, `1.06°`였고 반복 범위는 최대 `0.08°`였습니다. 이 결과에
따라 계획 경로는 변경하지 않고 MoveIt 궤적의 마지막 목표점에서만 J4를
폐루프 보정합니다.

```yaml
adaptive_goal_correction_enabled: true
adaptive_goal_correction_joints: [4_Joint]
adaptive_goal_correction_tolerance_deg: 0.5
adaptive_goal_correction_gain: 1.0
adaptive_goal_correction_max_total_deg: 3.0
adaptive_goal_correction_max_attempts: 5
```

매 보정 시 실제 `목표-실제` 잔차를 직전 명령에 더하되 원래 MoveIt 목표에서
최대 ±3°까지만 허용합니다. 네 번 안에 J4 잔차가 0.5° 이하로 수렴하지 않거나
보정 명령이 관절 한계를 벗어나면 trajectory를 성공 처리하지 않습니다. 다른
관절은 측정 위치에 따라 오차 방향과 크기가 달라 J4 보정 대상에 포함하지 않습니다.
