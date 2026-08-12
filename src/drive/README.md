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

### 상대 AGV 가상 장애물

다중 차량 launch는 기본적으로 상대 차량의 AMCL 위치를 각 차량의 local 및 global
costmap에 가상 장애물로 추가합니다. local costmap은 근접 충돌을 막고, global
costmap은 planner가 상대 차량을 우회하는 경로를 다시 생성하게 합니다.

```text
AGV1 AMCL -> /agv1/shared_amcl_pose -> AGV2 obstacle node
AGV2 AMCL -> /agv2/shared_amcl_pose -> AGV1 obstacle node
```

`amcl_pose_heartbeat` 노드는 자기 AMCL 위치를 주기적으로 공유하고,
`other_robot_obstacle` 노드는 상대 위치 주변을 원형 PointCloud2로 발행합니다.
기본 반경은 `0.13m`, pose timeout은 `1.0s`입니다. 상대 위치가 이동하거나
timeout되면 별도의 clearing PointCloud2로 이전 위치를 지웁니다. 두 토픽은
local/global costmap의 독립적인 `other_robot_layer`에서 사용하며 다음 항목에는
연결하지 않습니다.

PointCloud2는 상대 위치의 `map` 좌표를 각 차량의 `base_footprint` 좌표로 변환해
발행합니다. 따라서 clearing ray의 센서 원점이 rolling local costmap 내부의 자기
차량 위치가 되고, 지도 원점이 local costmap 밖에 있을 때 발생하는 raytrace 경고를
방지합니다.

- AMCL의 `scan` 입력
- static map/map server

AMCL은 정지 상태에서 `amcl_pose`를 연속 발행하지 않으므로,
`amcl_pose_heartbeat`가 마지막 위치를 `shared_amcl_pose`로 주기적으로
relay합니다. AMCL publisher가 사라지면 relay도 중단되고 상대 차량이 timeout 후
가상 장애물을 제거합니다. 위치 공유 bridge와 costmap 장애물 생성은 서로 독립된
노드입니다.

실행 시 조정:

```bash
ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv1 \
  other_robot_obstacle_radius:=0.13 \
  other_robot_pose_timeout:=1.0
```

기능을 끄고 기존 LiDAR-only local costmap으로 비교할 때:

```bash
ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv1 \
  start_other_robot_obstacle:=false
```

상태 확인:

```bash
ros2 topic hz /agv1/other_robot_obstacle
ros2 topic hz /agv2/other_robot_obstacle
```

차량별 DDS domain이 분리된 Zenoh 구성에서는 AGV1 bridge가
`/agv2/shared_amcl_pose`, AGV2 bridge가 `/agv1/shared_amcl_pose`를
subscriber로 허용해야 합니다. 저장소의 `config/network/zenoh_agv1.json5`와
`zenoh_agv2.json5`에 이 설정이 포함되어 있습니다.

### Collision Monitor와 ToF 확장

현재 실차 속도 경로는 다음과 같습니다.

```text
Nav2 velocity_smoother -> cmd_vel_safe_input -> Pinky safety gate -> cmd_vel
```

`nav2_collision_monitor`는 설치되어 있지만 아직 이 경로에 넣지 않습니다. 적용할
때는 safety gate를 우회하지 않도록 반드시 다음 순서로 연결해야 합니다.

```text
velocity_smoother -> collision_monitor input
collision_monitor output -> cmd_vel_safe_input
cmd_vel_safe_input -> 기존 Pinky safety gate -> cmd_vel
```

Collision Monitor를 켜지 않은 이유는 가상 장애물 검증과 최종 속도 차단 계층 변경을
한 번에 적용하지 않기 위해서입니다. 이후 STOP/SLOWDOWN polygon과 remap을 별도
opt-in launch로 추가할 수 있습니다.

ToF 센서는 AMCL에 연결하지 않고 `other_robot_layer`와 별개의 local costmap
observation source 또는 Collision Monitor source로 추가합니다. 이렇게 하면 ToF가
없어도 현재 구성이 그대로 동작합니다.

Nav2 localization/navigation launch, 파라미터, Behavior Tree와 RViz 설정을
제공하는 패키지입니다. 다중 차량 모드에서는 각 차량 컴퓨터에서 namespaced Nav2를
실행하고 중앙 관제는 차량별 Nav2 action으로 목표만 전달합니다.

## 구성

```text
launch/bringup_launch.xml            # AMCL, map server와 Nav2 전체 실행
launch/localization_launch.xml       # map server와 AMCL
launch/navigation_launch.xml         # planner, controller와 Nav2 서버
launch/nav2_view.launch.xml          # Nav2 RViz 화면
launch/navigation_event.launch.xml   # Nav2 시작/종료 JSON 이벤트 발행
launch/target_map_pose_nav.launch.xml # 중앙 목표를 Nav2 action으로 전달
launch/target_waypoints_nav.launch.xml # 여러 중앙 목표를 Nav2 through-poses action으로 전달
params/nav2_params.yaml              # AMCL, costmap, planner/controller 설정
params/parking_spots.yaml            # 지정 주차 approach와 parked 자세
action/ParkInSpot.action             # 지정 주차 action 인터페이스
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

### `navigation_event_publisher`

Nav2 action status와 `/amcl_pose`를 감시해 자율주행 시작/종료 이벤트를 JSON 문자열로
발행합니다. LLM 또는 외부 자연어 AI 연동 쪽에서는 `std_msgs/String.data`를 JSON으로 파싱하면
됩니다.

```text
입력: /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
입력: /navigate_to_pose/_action/status (action_msgs/GoalStatusArray)
입력: /navigate_through_poses/_action/status (action_msgs/GoalStatusArray)
출력: /drive/navigation_event (std_msgs/String JSON)
```

시작 시 현재 위치가 `start_position_1_x/y` 반경 `start_position_tolerance_m` 안이면
`start_type`을 `position_1`로, 아니면 `other`로 발행합니다. 기본 `position_1`은
AGV2 `parking_yellow`의 최종 주차 좌표인 `x=1.635464`, `y=0.168810`입니다.
시작/종료 시 현재 위치가 `params/navigation_areas.yaml`에 등록된 좌표 반경 안인지 검사해
`current_area`와 `matched_area`를 함께 발행합니다. 기본 area는
`A:0.192099:0.043845`, `B:0.200000:0.100000`,
`parking_yellow:1.635464:0.168810`입니다.

```yaml
area_tolerance_m: 0.25
areas:
  A:
    x: 0.192099
    y: 0.043845
  B:
    x: 0.200000
    y: 0.100000
  parking_yellow:
    x: 1.635463773844374
    y: 0.16880950610511666
```

```bash
ros2 launch drive navigation_event.launch.xml
```

### `parking_action_server`

지정한 주차 ID를 읽어 접근 경로를 Nav2 `NavigateToPose` goal로 순차 실행한 뒤, 마지막
구간은 `/cmd_vel` 후진 제어로 `parked` 위치까지 진입합니다.

```text
입력: /park_in_spot (drive/action/ParkInSpot)
입력: /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
출력: navigate_to_pose (nav2_msgs/action/NavigateToPose)
출력: /cmd_vel (geometry_msgs/Twist)
설정: params/parking_spots.yaml
```

`approach_path`가 있으면 중간 지점은 yaw 허용 오차를 크게 열고 위치 위주로 통과하며,
마지막 `approach` 지점에서는 yaw 허용 오차를 다시 좁힙니다. `approach`에서 `parked`
방향과 크게 어긋나면 후진하지 않고 실패 처리합니다.

### `parking_new`

`park_red` 지정 주차에 맞춘 실행 노드입니다. `parking_spots.yaml`의 `approach`와 `parked`를
읽고, `approach` 기준 자동 pre-approach를 계산해 `pre-approach -> approach` 두 지점을
Nav2 `NavigateToPose` goal로 순차 실행합니다. 마지막 구간은 `parked` 좌표 도착 판정 대신
고정 후진 거리와 속도로 `/cmd_vel`을 발행합니다.

```text
입력: /park_in_spot (drive/action/ParkInSpot)
입력: /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
출력: navigate_to_pose (nav2_msgs/action/NavigateToPose)
출력: /cmd_vel (geometry_msgs/Twist)
설정: params/parking_spots.yaml
```

현재 AGV1 기본 접근값과 AGV2 캘리브레이션값:

```text
approach: x=1.218242, y=0.375254, yaw=-3.095542
auto pre-approach: x=1.392214, y=0.261071, yaw=-3.095542
AGV2 approach: x=1.365431, y=0.176245, yaw=-3.136509
AGV2 parked: x=1.635464, y=0.168810, yaw=-3.098530
reverse_distance_m: 0.476720       # [m] 약 47.7 cm 고정 후진
reverse_speed: 0.095344            # [m/s] 약 5.0초 후진
```

action feedback의 `phase` 문자열에는 현재 향하는 목표 좌표가 같이 표시됩니다.

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

## 지정 주차 실행

Nav2와 AMCL 초기 위치 설정이 끝난 뒤 별도 터미널에서 실행합니다.

```bash
ros2 run drive parking_new
```

기본 주차 설정 파일은 `src/drive/params/parking_spots.yaml`입니다. 다른 파일을 쓰려면:

```bash
ros2 run drive parking_new --ros-args \
  -p parking_spots_yaml:=/absolute/path/to/parking_spots.yaml
```

주차 action 전송:

```bash
ros2 action send_goal /park_in_spot drive/action/ParkInSpot \
  "{spot_id: park_red}" --feedback
```

주요 파라미터:

```text
auto_pre_approach_distance_m: 0.208096       # [m] approach 기준 자동 pre-approach 거리
auto_pre_approach_angle_offset_deg: -35.916590 # [deg] approach 후방 기준 각도 보정
reverse_distance_m: 0.476720       # [m] 마지막 고정 후진 거리
reverse_speed: 0.095344            # [m/s] 마지막 후진 속도
approach_xy_tolerance: 0.05        # [m] approach 위치 확인
strict_yaw_goal_tolerance: 0.05    # [rad] 최종 approach yaw 허용 오차
```

각 주차 위치에는 `auto_pre_approach_distance_m`과 `reverse_distance_m`을
선택적으로 지정할 수 있습니다. 생략하면 위 노드 기본값을 사용합니다.
`parking_yellow`는 AGV2에서 측정한 접근 자세로 바로 이동한 뒤 약 `0.270 m`를
후진하도록 별도로 설정되어 있습니다.

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
- `/scan` timestamp 지연 완화를 위해 Nav2 주요 `transform_tolerance`는 현재 1.5초로 설정되어 있습니다.
