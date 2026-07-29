# Drive Package

## 2대 차량 Nav2

`multi_vehicle_nav.launch.py`는 기존 `nav2_params.yaml`을 실행 시점에 복사하여
차량별 TF 프레임으로 바꿉니다. 원본 파라미터와 단일 차량 명령은 변경하지
않습니다.

```bash
ros2 launch drive multi_vehicle_nav.launch.py vehicle_id:=agv1
```

```bash
ros2 launch drive multi_vehicle_nav.launch.py vehicle_id:=agv2
```

생성되는 주요 이름:

```text
/agv1/navigate_to_pose
/agv1/navigate_through_poses
/agv1/initialpose
map -> agv1/odom -> agv1/base_footprint -> agv1/base_link
```

Nav2의 최종 속도는 `/<vehicle_id>/cmd_vel_safe_input`으로 전달되어 Pinky 안전
게이트를 거칩니다.

Nav2 localization/navigation launch, 파라미터, Behavior Tree와 RViz 설정을
제공하는 패키지입니다. 다중 차량 모드에서는 각 차량 컴퓨터에서 namespaced Nav2를
실행하고 중앙 관제는 차량별 Nav2 action으로 목표만 전달합니다.

## 구성

```text
launch/bringup_launch.xml            # AMCL, map server와 Nav2 전체 실행
launch/localization_launch.xml       # map server와 AMCL
launch/navigation_launch.xml         # planner, controller와 Nav2 서버
launch/nav2_view.launch.xml          # Nav2 RViz 화면
launch/target_map_pose_nav.launch.xml # 중앙 목표를 Nav2 action으로 전달
launch/target_waypoints_nav.launch.xml # 여러 중앙 목표를 Nav2 through-poses action으로 전달
params/nav2_params.yaml              # AMCL, costmap, planner/controller 설정
behavior_trees/                      # NavigateToPose Behavior Tree
rviz/                                # RViz 설정
```

## 노드

### `target_map_pose_to_nav_goal`

중앙에서 발행한 map 기준 목표 위치와 방향을 Nav2 `NavigateToPose` goal로 전달합니다.

```text
입력: /central/target_map_pose (geometry_msgs/PoseStamped)
출력: navigate_to_pose (nav2_msgs/action/NavigateToPose)
```

새 목표가 들어오면 `/bt_navigator/get_state`를 확인합니다. Nav2가 아직
`active(3)`가 아니면 최신 목표 한 개를 보관하고 lifecycle 활성화 후 자동으로
전송합니다. 진행 중인 이전 목표가 있으면 취소하고 새 목표를 전송합니다.

상태 확인:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
```

모두 `active [3]`이어야 경로 계획과 주행이 시작됩니다.

### `target_waypoints_to_nav_goal`

중앙에서 발행한 map 기준 여러 목표 위치를 Nav2 `NavigateThroughPoses` goal로 전달합니다.

```text
입력: /central/target_map_poses (geometry_msgs/PoseArray)
입력: /central/target_map_path (nav_msgs/Path)
출력: navigate_through_poses (nav2_msgs/action/NavigateThroughPoses)
```

두 입력 중 하나만 발행하면 됩니다. 새 waypoint 묶음이 들어오면 진행 중인 이전 waypoint
goal을 취소하고 새 묶음을 전송합니다.

### `send_nav_goal`

Nav2를 중앙 시스템 없이 직접 시험하는 명령입니다. map 좌표 단위는 meter이고 yaw 단위는
radian입니다.

```bash
ros2 run drive send_nav_goal --world -0.5 -1.0 --yaw 1.5708
```

`--world`를 생략하면 입력 `x`, `y`를 720x720 map canvas 픽셀로 해석하므로, map 좌표를
시험할 때는 반드시 `--world`를 사용합니다.

### `reverse_parking`

AMCL의 `/amcl_pose`를 사용해 실행 시점의 현재 자세를 기록합니다. 어느 위치에서 실행해도
먼저 Nav2 `navigate_to_pose` action으로 준비 자세 `(1.365431, 0.176245)`까지 자율주행한 뒤,
목표 자세 `(1.635464, 0.168810)`까지 **후진만**으로 이동합니다. 후진 단계의 선형 속도는
항상 음수(`/cmd_vel.linear.x < 0`)이며, 목표점이 차량 앞쪽으로 판단되면 전진하지 않고
즉시 정지합니다.

주차 순서는 `목표점이 로봇 뒤쪽이 되도록 제자리 회전 → 후진 이동 → 목표 방향 미세 보정`입니다.
회전 완료 판정은 기본 5도이며 필요하면 다음처럼 조정할 수 있습니다.

```bash
ros2 run drive reverse_parking --ros-args -p reverse_yaw_tolerance:=0.05
```

```bash
ros2 run drive reverse_parking
```

준비 위치 이동에는 Nav2가 필요합니다. Nav2 목표 완료 뒤 이 노드가 `/cmd_vel`을 직접
제어하므로, teleop 등 별도의 `/cmd_vel` 발행 노드는 중지해야 합니다. 처음 시험할 때에는
더 낮은 속도를 권장합니다.

```bash
ros2 run drive reverse_parking --ros-args -p max_reverse_speed:=0.03
```

## 빌드

노트북의 워크스페이스에서 실행합니다.

```bash
cd ~/poter_ws
colcon build --packages-select drive
source install/setup.bash
```

새 터미널을 열 때마다 다음 명령이 필요합니다.

```bash
source ~/poter_ws/install/setup.bash
```

## 단일 차량 호환 실행 순서

아래 절차는 기존 단일 차량에서 Nav2를 노트북에 두는 호환 모드입니다. `agv1`,
`agv2` 다중 차량 운용에서는 `porter_bringup agv_vehicle.launch.py`가 각 차량의
Nav2까지 실행하므로 아래 노트북 Nav2 명령을 함께 실행하지 않습니다.

### 1. 차량 하드웨어 실행

핑키에서 센서, odometry와 모터 제어만 실행합니다.

```bash
ros2 launch pinky bringup_robot.launch.xml
```

### 2. 노트북에서 Nav2 실행

기본 지도는 `~/poter_ws/config/SLAM/current_map.yaml`입니다. 워크스페이스 위치가 다르면
`workspace:=/path/to/workspace` 또는 `map:=...`으로 지정합니다.

```bash
ros2 launch drive bringup_launch.xml \
  workspace:=$HOME/poter_ws
```

지도 파일만 별도로 지정하려면 `map:=...`과 `keepout_mask:=...`를 사용합니다.

### 3. 노트북에서 RViz 실행

```bash
ros2 launch drive nav2_view.launch.xml
```

RViz의 RobotModel mesh는 기본적으로 현재 워크스페이스의 `~/poter_ws/install/pinky`에서 찾습니다.
이 mesh lookup 환경은 `drive` 패키지의 `nav2_view.launch.xml`에서 RViz 프로세스에 직접 적용합니다.
워크스페이스 위치가 다르면:

```bash
ros2 launch drive nav2_view.launch.xml \
  workspace:=/path/to/poter_ws
```

RViz 상단의 `2D Pose Estimate`를 선택하고 실제 차량 위치와 방향을 map 위에 지정합니다.
초기 위치를 지정하기 전에는 `map -> odom` TF가 정상적으로 만들어지지 않을 수 있습니다.

확인:

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
```

### 4. 중앙 목표 브릿지 실행

```bash
ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

중앙 노트북에서 `/central/target_map_pose`가 발행되면 Nav2가 경로를 생성하고 `/cmd_vel`을
발행합니다. 같은 ROS domain의 차량 bringup이 이 속도 명령을 받아 모터를 제어합니다.

여러 지점을 한 번에 보내려면 waypoint 브릿지를 실행합니다.

```bash
ros2 launch drive target_waypoints_nav.launch.xml
```

중앙 노트북에서 `/central/target_map_poses` 또는 `/central/target_map_path`를 발행하면
Nav2가 지점 순서대로 통과하는 경로를 생성합니다.

## 한 번에 실행

노트북에서 Nav2와 목표 브릿지를 한 번에 실행할 수 있습니다.

```bash
ros2 launch drive target_map_pose_nav.launch.xml \
  start_nav2:=true \
  workspace:=$HOME/poter_ws
```

RViz는 별도 터미널에서 실행합니다.

```bash
ros2 launch drive nav2_view.launch.xml
```

## 수동 주행 시험

Nav2와 AMCL 초기 위치 설정이 끝난 후 map 좌표를 직접 전송합니다.

```bash
ros2 run drive send_nav_goal \
  --world -0.5 -1.0 \
  --yaw 0.0
```

실제로 goal을 보내지 않고 변환 결과만 확인하려면 `--dry-run`을 사용합니다.

```bash
ros2 run drive send_nav_goal \
  --world -0.5 -1.0 \
  --yaw 0.0 \
  --dry-run
```


## 중요 사항

- `/central/target_map_pose`는 이미 `map` 좌표계의 `PoseStamped`이므로 다시 픽셀 변환하면 안 됩니다.
- Nav2 지도와 카메라 calibration에 사용한 지도는 동일해야 합니다.
- 노트북의 `/cmd_vel`을 차량 bringup이 구독하는지 확인해야 합니다.
- 차량과 노트북에서 Nav2를 동시에 실행하면 노드, action과 `/cmd_vel`이 충돌합니다.
- SLAM mapping 중에는 `drive bringup_launch.xml`을 실행하지 않습니다. SLAM과 AMCL이 동시에 `map -> odom` TF를 발행할 수 있습니다.
- 차량의 `base_link`, `base_footprint`, LiDAR frame TF는 차량 bringup에서 제공해야 합니다.
