# SLAM Package

Pinky 맵 생성을 위한 SLAM launch/config 패키지입니다.

## Files

- `launch/map_building.launch.xml`: SLAM Toolbox 기반 맵 생성 실행
- `launch/map_view.launch.xml`: 맵 확인용 RViz 실행
- `params/mapper_params.yaml`: SLAM Toolbox 파라미터
- `rviz/map_building.rviz`: 맵 생성 RViz 설정

## Build

```bash
cd ~/poter_ws
colcon build --packages-select slam
source install/setup.bash
```

## Run

```bash
ros2 launch slam map_building.launch.xml
```

RViz 확인

```bash
ros2 launch slam map_view.launch.xml
```

맵 저장

```bash
ros2 run nav2_map_server map_saver_cli -f ~/poter_ws/config/SLAM/current_map
```
