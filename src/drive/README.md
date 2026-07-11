# Drive Package

Pinky 주행을 위한 Nav2 launch/config 패키지입니다.

## Files

- `launch/bringup_launch.xml`: localization과 navigation을 함께 실행하는 bringup launch
- `launch/localization_launch.xml`: AMCL 기반 localization 실행
- `launch/navigation_launch.xml`: Nav2 navigation 실행
- `launch/nav2_view.launch.xml`: Nav2 확인용 RViz 실행
- `params/nav2_params.yaml`: Nav2 파라미터
- `rviz/nav2_view.rviz`: Nav2 RViz 설정
- `behavior_trees/`: Nav2 behavior tree 설정

## Build

```bash
cd ~/poter_ws
colcon build --packages-select drive
source install/setup.bash
```

## Run

```bash
ros2 launch drive bringup_launch.xml
```

기본 map 경로는 `~/poter_ws/config/SLAM/current_map.yaml`입니다.

RViz 확인

```bash
ros2 launch drive nav2_view.launch.xml
```
