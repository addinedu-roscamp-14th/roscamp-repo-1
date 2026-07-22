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

## 두 번째 로봇 설정

두 번째 팔에 연결된 장치 경로를 실행 시 지정합니다.

```bash
ros2 launch arm2 arm2_container_pick_hardware.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=config/arm2/arm2_gripper_camera_info.yaml \
  serial_port:=/dev/ttyUSB1 \
  trajectory_speed:=100
```

수동 조작:

```bash
ros2 run arm2 arm2_manual_jog --ros-args \
  -p serial_port:=/dev/ttyUSB1
```

## Hand-Eye 캘리브레이션

두 번째 팔은 카메라 장착 위치와 파지 오프셋이 첫 번째 팔과 다르므로 별도로
캘리브레이션해야 합니다. 기존 `config/arm/*.calib` 파일은 복사하지 않았습니다.

```bash
ros2 launch arm2 arm2_handeye_charuco_calibration.launch.py \
  video_device:=/dev/video2 \
  camera_info_url:=config/arm2/arm2_gripper_camera_info.yaml \
  name:=arm2_jetcobot_eye_in_hand_charuco \
  calibration_directory:=config/arm2
```

캘리브레이션 완료 후 생성되는 파일은 다음 이름으로 관리합니다.

```text
config/arm2/arm2_jetcobot_eye_in_hand_charuco.calib
```

파지 오프셋까지 측정한 뒤
`config/arm2/arm2_container_pick.yaml`의 값을 갱신하고
`offsets_configured: true`로 변경해야 실제 파지가 허용됩니다.

## 파지 실행

```bash
ros2 launch arm2 arm2_container_pick_moveit.launch.py \
  camera_info_url:=config/arm2/arm2_gripper_camera_info.yaml \
  video_device:=/dev/video2 \
  calibration_name:=arm2_jetcobot_eye_in_hand_charuco \
  params_file:=config/arm2/arm2_container_pick.yaml \
  serial_port:=/dev/ttyUSB1 \
  trajectory_speed:=100 \
  use_rviz:=true
```

```bash
ros2 service call /arm2/pick_container std_srvs/srv/Trigger '{}'
```

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
