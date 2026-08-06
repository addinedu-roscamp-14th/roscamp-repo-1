# Arm Simple Pick/Place

기존 `arm`, `arm_relocation2` 구현과 분리한 새 패키지입니다. 동작 도중
ArUco를 다시 추적하거나 자세를 재계산하지 않습니다.

## 좌표계 결론

기존 `handeye_charuco_calibration.launch.py`의 기본값은
`robot_effector_frame:=arm/TCP`이므로 TCP 기준입니다. 이 패키지의
`handeye_flange_charuco_calibration.launch.py`는 기본 effector를
`arm/6_Link`로 바꿉니다. 따라서 저장되는 결과는 TCP가 아니라 로봇 모델의
플랜지 측 고정 링크에서 카메라 광학 프레임까지의 6D 변환입니다.

`6_Link`는 현재 URDF에 존재하는 가장 안정적인 손목 기준 프레임입니다.
실제 금속 플랜지 면의 원점을 엄밀히 쓰려면 그 면까지의 고정 XYZ/RPY를 실측해
별도 `flange_face` 링크로 모델링해야 합니다. 하지만 hand-eye 정확도 자체는
어떤 고정 손목 프레임을 선택해도 같으며, 카메라가 플랜지에서 떨어진 거리를
별도로 다시 더하면 오히려 이중 보정이 됩니다. 캘리브레이션 결과의
`transform.translation`이 이미 그 거리 성분을 포함합니다.

## 정확한 시퀀스

`pick`:

1. Pick ArUco 안정화 및 base 좌표 고정
2. 그리퍼 열기
3. `[marker x, marker y, approach_z, -180, 0, marker yaw]`
4. X/Y/RPY를 유지하고 `marker z + pick_z_offset`으로 Z 하강
5. 그리퍼 닫기
6. X/Y/RPY를 유지하고 `pick_lift_z`로 Z 상승

`place`:

1. Place ArUco 안정화 및 base 좌표 고정
2. `[marker x, marker y, approach_z, -180, 0, marker yaw]`
3. X/Y/RPY를 유지하고 `marker z + place_z_offset`으로 Z 하강
4. 그리퍼 열기
5. X/Y/RPY를 유지하고 `retreat_z`로 Z 상승

`pick_and_place`는 실제 Pick/Place 조작 전에 다음 관찰 순서를 한 번
수행합니다.

1. `first_observation_joint_angles_deg`로 이동
2. 로봇 정지 후 필요한 Pick/Place 마커 중 보이는 것을 base 좌표로 고정
3. First에서 아무 마커도 검출되지 않아도 중단하지 않음
4. 항상 `second_observation_joint_angles_deg`로 이동
5. 아직 저장하지 못한 마커를 다시 탐색하여 base 좌표로 고정
6. 두 자세를 모두 확인한 뒤에도 필요한 마커가 없을 때만 실패
7. 저장한 좌표만 사용하여 Pick과 Place 실행

First pose는 보조 관찰이므로 기본 5초 동안만 탐색하고, 누락 마커를 찾는
Second pose에는 기본 15초를 사용합니다. 각각
`first_observation_timeout_sec`, `second_observation_timeout_sec`로 조정할
수 있습니다.

ArUco detector는 평상시와 모든 로봇 이동 중에는 비활성 상태입니다.
관찰 관절 자세 도착, 관절 오차 확인, `observation_settle_sec` 대기가 모두
끝난 뒤에만 활성화됩니다. 관찰 창이 닫히면 즉시 다시 비활성화되며, 활성화
시각보다 오래된 TF와 이동 중 샘플은 사용할 수 없습니다.

Direct 관절 이동이 허용 오차 밖에서 멈추면
`observation_correction_attempts` 횟수만큼 같은 관절 목표를 다시
명령합니다. 보정 후에도 `observation_joint_tolerance_deg`를 넘을 때만
관찰 실패로 처리합니다.

개별 `pick`과 `place` 서비스도 First와 Second를 차례로 확인합니다.
관찰 관절값은 이 패키지의 `config/simple_pick_place.yaml`에만 있으며 기존
`config/arm/container_pick_place.yaml`을 읽지 않습니다.

`pick_z_offset_m`과 `place_z_offset_m`은 일부러 분리했습니다. Pick 상승의
`pick_lift_z_m`는 절대 base Z인 반면, Place 하강은 컨테이너/바닥을 누르지
않도록 marker Z에 clearance를 더하는 상대값이기 때문입니다.

Pick과 Place의 공통 목표 Yaw는 다음처럼 계산합니다.

```text
goal_yaw = wrap(marker_yaw + marker_yaw_offset_deg)
```

현재 YAML 설정값은 `-45.0`입니다. 접근, 하강, 상승 중에는
계산된 동일 Yaw를 계속 유지합니다.

## Direct controller 좌표계

세 개의 정지 자세에서 `get_coords()`를 `arm/6_Link`, `arm/TCP`, 카메라
TF와 비교한 결과, pymycobot의 `send_coords/get_coords`는 TCP나 카메라가
아닌 `arm/6_Link`에 고정된 별도 좌표계를 사용합니다. 실행 런치는 이를
`arm/controller_coords`로 발행합니다.

```text
arm/6_Link -> arm/controller_coords
translation_m: [0.0184, 0.0000, -0.0019]
rpy_deg: [-90.0, -45.0, -90.0]
```

Direct backend의 모든 `send_coords`, `get_coords` 목표와 도착 검증은
`arm/controller_coords` 기준입니다. 이 변경은 기존에 컨트롤러에 보낸
좌표계에 맞춰 수행합니다.

별도 XY offset이나 카메라/TCP 위치 보정은 적용하지 않습니다. TF로 얻은
ArUco marker의 base-frame X/Y를 그대로 Direct `send_coords` 목표로
사용합니다. 따라서 URDF와 hand-eye 변환 결과를 추가 보정 없이 시험할 수
있습니다.

Hand-eye 결과는 계속 `arm/6_Link -> camera`로 유지합니다. TF 트리에서
`controller_coords`, `6_Link`, 카메라 관계가 모두 연결되므로 카메라
오프셋을 Pick/Place 좌표에 다시 더하지 않습니다.

## 캘리브레이션

카메라 내부 파라미터 캘리브레이션을 먼저 완료한 뒤:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select arm arm_simple_pick_place
source install/setup.bash

ros2 launch arm_simple_pick_place \
  handeye_flange_charuco_calibration.launch.py \
  video_device:=/dev/video2
```

여러 방향과 위치에서 충분한 ChArUco 샘플을 수집하고 결과를
`jetcobot_eye_in_hand_charuco_flange` 이름으로 저장합니다. 기존 TCP
캘리브레이션 파일은 덮어쓰지 않습니다.

### MoveIt으로 로봇을 움직이며 캘리브레이션

실기체 trajectory bridge, MoveIt/RViz, 플랜지 기준 ChArUco 캘리브레이션을
한 번에 실행할 수 있습니다.

```bash
ros2 launch arm_simple_pick_place \
  handeye_flange_charuco_moveit.launch.py \
  serial_port:=/dev/ttyUSB0 \
  video_device:=/dev/video2 \
  trajectory_speed:=15
```

실행 후 RViz의 `MotionPlanning` 패널에서:

1. Planning Group을 `arm_group`으로 선택합니다.
2. 카메라에서 ChArUco 보드가 선명하게 보이는 안전한 목표 자세를 정합니다.
3. `Plan`으로 충돌 없는 경로인지 확인하고 `Execute`를 누릅니다.
4. 로봇이 완전히 멈추고 보드 pose가 안정되면 easy_handeye2 GUI에서
   `Take Sample`을 누릅니다.
5. 위치와 손목 방향을 함께 바꿔 최소 12~15개 자세에서 반복합니다.
6. 샘플 계산 후 결과를 저장합니다.

MoveIt의 planning tip은 현재 SRDF의 `TCP`이지만 캘리브레이션 기록 기준은
`arm/6_Link`입니다. MoveIt으로 TCP 목표를 주어 로봇을 움직여도
`base_link -> 6_Link` TF가 같이 갱신되므로 플랜지 기준 캘리브레이션에는
문제가 없습니다.

## 실행

먼저 `config/simple_pick_place.yaml`의 다섯 Z 값을 실제 장비에서 낮은
속도로 교시합니다. 기본값은 예시이므로 검증 없이 실동작에 쓰면 안 됩니다.

Jetcobot 기본 Cartesian 함수:

```bash
ros2 launch arm_simple_pick_place simple_pick_place.launch.py \
  motion_backend:=direct \
  video_device:=/dev/video2 \
  pick_marker_id:=0 \
  place_marker_id:=7
```

MoveIt:

```bash
ros2 launch arm_simple_pick_place simple_pick_place.launch.py \
  motion_backend:=moveit \
  video_device:=/dev/video2 \
  pick_marker_id:=0 \
  place_marker_id:=7
```

서비스:

```bash
ros2 service call /arm/simple_pick std_srvs/srv/Trigger "{}"
ros2 service call /arm/simple_place std_srvs/srv/Trigger "{}"
ros2 service call /arm/simple_pick_and_place std_srvs/srv/Trigger "{}"
ros2 service call /arm/simple_stop std_srvs/srv/Trigger "{}"
```

상태는 `/arm/simple_pick_place/status`에 출력됩니다.

### 조작 없이 ID 2 ArUco TF 연속 발행

다음 standalone 런치는 Pick/Place 노드와 그리퍼 명령을 실행하지 않습니다.
20 mm ArUco ID 2를 계속 검출하여 `arm/target_marker` TF를 발행합니다.
카메라 영상에 마커가 보이는 동안 detector는 항상 활성 상태입니다.

```bash
ros2 launch arm_simple_pick_place continuous_target_aruco_tf.launch.py \
  serial_port:=/dev/ttyUSB0 \
  video_device:=/dev/video2
```

base 기준 결과:

```bash
ros2 run tf2_ros tf2_echo arm/base_link arm/target_marker
```

기본값은 `target_id:=2`, `marker_size_m:=0.020`이며 필요하면 launch 인자로
변경할 수 있습니다. 이 런치는 로봇을 움직이지 않지만 실제 관절값을 읽어
URDF TF를 갱신하므로, 다른 프로그램으로 로봇을 제어하면 `/dev/ttyUSB0`
시리얼 포트를 동시에 열지 않도록 주의해야 합니다.

## 두 백엔드의 차이

- `direct`: `sync_send_coords`를 그대로 사용하므로 구성 요소와 실패 지점이
  적습니다. 다만 충돌 검사나 경로 계획이 없고, pymycobot이 설정한 Cartesian
  tool frame을 명령합니다. 현재처럼 RX/RY 고정 시 X/Y가 플랜지와 일치한다는
  장비 조건을 사용하고 Z offset을 현장에서 교시합니다.
- `moveit`: 접근은 충돌 검사된 pose plan, 하강/상승은 Cartesian Z 경로를
  사용합니다. 장애물과 self-collision을 확인할 수 있지만 IK, planning
  scene, joint state 동기화 때문에 실패 가능 지점이 더 많습니다.

두 백엔드 모두 도착 후 위치와 각도 오차를 한 번 확인하며 초과하면 다음
단계로 진행하지 않습니다. 전체 시퀀스 시험 중에는 위치 ±15 mm, 각도 ±6°를
사용합니다. 설정은 `position_tolerance_m`, `angle_tolerance_deg`에서
확인할 수 있습니다.
