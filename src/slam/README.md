# SLAM Package

핑키 차량의 LiDAR와 odometry를 사용해 노트북에서 지도를 작성하는 패키지입니다.
SLAM Toolbox 설정과 mapping용 RViz 화면만 포함합니다.

## 구성

```text
launch/map_building.launch.xml  # SLAM Toolbox 실행
launch/map_view.launch.xml      # mapping용 RViz 실행
launch/slam_bringup.launch.xml  # SLAM과 RViz 동시 실행
params/mapper_params.yaml       # frame, scan, 해상도와 loop closure 설정
rviz/map_building.rviz          # mapping 화면
```

## 빌드

```bash
cd ~/poter_ws
colcon build --packages-select slam
source install/setup.bash
```

## 실행

차량에서는 하드웨어 bringup을 실행합니다.

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

노트북에서는 SLAM과 RViz를 실행합니다.

```bash
ros2 launch slam slam_bringup.launch.xml
```

RViz 없이 SLAM만 실행하려면:

```bash
ros2 launch slam map_building.launch.xml
```

RViz만 별도로 실행하려면:

```bash
ros2 launch slam map_view.launch.xml
```

## 필요한 토픽과 TF

차량에서 다음 데이터가 노트북으로 전달되어야 합니다.

```text
/scan
/odom
/tf
/tf_static
```

기본 frame 설정:

```text
map_frame: map
odom_frame: odom
base_frame: base_footprint
scan_topic: /scan
```

## 지도 저장

워크스페이스의 `config/SLAM/current_map.yaml`과 `current_map.pgm`으로 저장합니다.

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/jio/poter_ws/config/SLAM/current_map
```

기존 파일을 덮어쓰기 전 필요한 지도인지 확인해야 합니다. 저장 후 calibration에서 사용하는
지도도 같은 파일인지 확인합니다.

## 확인

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /map --once
```

차량과 노트북은 같은 네트워크와 같은 `ROS_DOMAIN_ID`를 사용해야 합니다. Nav2와 SLAM이
동시에 `map -> odom` TF를 발행하지 않도록 mapping 중에는 `drive bringup_launch.xml`을
실행하지 않습니다.
