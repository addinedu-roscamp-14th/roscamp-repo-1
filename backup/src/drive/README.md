# Drive Package

차량 Nav2와 중앙 관제 좌표를 연결하는 패키지입니다.

## Nodes

### `target_map_pose_to_nav_goal`

중앙 관제에서 발행한 map 기준 목표 좌표를 Nav2 `NavigateToPose` action goal로 전달합니다.

입력:

```text
/central/target_map_pose
geometry_msgs/PoseStamped
```

출력:

```text
navigate_to_pose
nav2_msgs/action/NavigateToPose
```

### `send_nav_goal`

차량에서 Nav2 goal을 직접 보내는 테스트 노드입니다.

중앙 관제에서 나온 map 좌표를 테스트할 때는 반드시 `--world`를 붙입니다.

```bash
ros2 run drive send_nav_goal --world 1.767494 0.344350
```

`--world`를 빼면 입력값을 map 픽셀/캔버스 좌표로 해석합니다.

## Build

```bash
cd ~/poter_ws
colcon build --packages-select drive
source install/setup.bash
```

## Run

차량에서 Nav2를 먼저 실행합니다.

```bash
ros2 launch pinky_navigation bringup_launch.xml map:=/home/pinky/current_map.yaml
```

Nav2가 준비된 뒤 중앙 목표 좌표 브릿지를 실행합니다.

```bash
ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

중앙 노트북에서 `/central/target_map_pose`를 발행하면 차량의 Nav2 goal로 전달됩니다.

## Combined Launch

차량에 `pinky_navigation` 패키지가 있고 한 번에 실행하고 싶으면:

```bash
ros2 launch drive target_map_pose_nav.launch.xml \
  start_nav2:=true \
  map:=/home/pinky/current_map.yaml
```

## Check

목표 좌표 수신 확인:

```bash
ros2 topic echo /central/target_map_pose
```

Nav2 action 확인:

```bash
ros2 action list | grep navigate
```

차량 속도 명령 확인:

```bash
ros2 topic echo /cmd_vel
```

AMCL 위치 확인:

```bash
ros2 topic echo /amcl_pose --once
```

## Notes

- `/central/target_map_pose`는 이미 `map` 좌표계의 `PoseStamped`입니다.
- 이 좌표를 픽셀 변환 노드에 다시 넣으면 안 됩니다.
- 차량 Nav2에서 사용하는 map과 중앙 calibration에 사용한 map은 같아야 합니다.
