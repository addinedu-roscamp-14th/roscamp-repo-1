# Drive Package

노트북에서 Nav2를 실행하고 중앙 관제의 map 목표 자세를 핑키 차량의 주행으로 연결하는
패키지입니다. Nav2 localization/navigation launch, 파라미터, Behavior Tree와 RViz 설정을
포함합니다.

## 구성

```text
launch/bringup_launch.xml            # AMCL, map server와 Nav2 전체 실행
launch/localization_launch.xml       # map server와 AMCL
launch/navigation_launch.xml         # planner, controller와 Nav2 서버
launch/nav2_view.launch.xml          # Nav2 RViz 화면
launch/target_map_pose_nav.launch.xml # 중앙 목표를 Nav2 action으로 전달
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

새 목표가 들어오면 진행 중인 이전 목표를 취소하고 새 목표를 전송합니다.

### `send_nav_goal`

Nav2를 중앙 시스템 없이 직접 시험하는 명령입니다. map 좌표 단위는 meter이고 yaw 단위는
radian입니다.

```bash
ros2 run drive send_nav_goal --world -0.5 -1.0 --yaw 1.5708
```

`--world`를 생략하면 입력 `x`, `y`를 720x720 map canvas 픽셀로 해석하므로, map 좌표를
시험할 때는 반드시 `--world`를 사용합니다.

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

## 전체 실행 순서

### 1. 차량 하드웨어 실행

핑키에서 센서, odometry와 모터 제어만 실행합니다.

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

### 2. 노트북에서 Nav2 실행

기본 지도는 `/home/jio/poter_ws/config/SLAM/current_map.yaml`입니다.

```bash
ros2 launch drive bringup_launch.xml \
  map:=/home/jio/poter_ws/config/SLAM/current_map.yaml
```

### 3. 노트북에서 RViz 실행

```bash
ros2 launch drive nav2_view.launch.xml
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

## 한 번에 실행

노트북에서 Nav2와 목표 브릿지를 한 번에 실행할 수 있습니다.

```bash
ros2 launch drive target_map_pose_nav.launch.xml \
  start_nav2:=true \
  map:=/home/jio/poter_ws/config/SLAM/current_map.yaml
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
