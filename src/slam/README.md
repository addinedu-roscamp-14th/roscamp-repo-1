# SLAM Package

핑키 차량의 LiDAR와 odometry를 사용해 노트북에서 지도를 작성하는 패키지입니다.
SLAM Toolbox 기반 맵핑 도구와 차량 SLAM 토픽을 웹 API로 중계하는 실행 구성을 포함합니다.

## 구성

```text
launch/map_building.launch.xml  # SLAM Toolbox 실행
launch/map_view.launch.xml      # mapping용 RViz 실행
launch/slam_bringup.launch.xml  # 차량 SLAM 토픽용 웹 API 서버 실행
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

차량에서는 하드웨어 bringup과 SLAM Toolbox를 실행합니다.

```bash
ros2 launch pinky bringup_robot.launch.xml
ros2 launch slam map_building.launch.xml
```

현재 차량 bringup의 `sllidar_node`는 `/scan` 발행 시점을 기준으로 `header.stamp`를 찍도록
수정되어 있습니다. 이전처럼 scan 수집 시작 시각을 stamp로 쓰면 `/scan`이 현재 TF보다
0.5~1초 이상 과거로 들어와 SLAM/Nav2 message filter에서 drop될 수 있습니다.

노트북에서는 SLAM과 RViz를 실행합니다.

```bash
ros2 launch slam slam_bringup.launch.xml
```

이 명령은 노트북에서 SLAM Toolbox, AMCL 또는 map server를 실행하지 않습니다.
dashboard API 서버만 실행해 차량의 지도, pose와 LiDAR scan을 전송합니다.
브라우저에서 다음 주소를 엽니다.

```text
http://localhost:8000/slam/view
```

차량에서 새 지도를 작성할 때 사용하는 명령은 다음과 같습니다.

```bash
ros2 launch slam map_building.launch.xml
```

`~/.bashrc`에 `slam` alias를 등록한 환경에서는 다음처럼 실행할 수 있습니다.

```bash
slam
```

RViz만 별도로 실행하려면:

```bash
ros2 launch slam map_view.launch.xml
```

## 필요한 토픽과 TF

차량에서 다음 데이터가 노트북으로 전달되어야 합니다.

```text
/scan
/map
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
워크스페이스 위치가 다르면 `$HOME/poter_ws` 대신 실제 경로를 사용합니다.

```bash
ros2 run nav2_map_server map_saver_cli \
  -f $HOME/poter_ws/config/SLAM/current_map
```

기존 파일을 덮어쓰기 전 필요한 지도인지 확인해야 합니다. 저장 후 calibration에서 사용하는
지도도 같은 파일인지 확인합니다.

## 확인

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /scan --once --field header
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /map --once
```

`/map`은 점유 지도와 좌표 메타데이터만 포함합니다. 차량 위치·헤딩은
`map -> odom -> base_footprint` TF에서 받고 LiDAR 점은 `/scan`에서 받습니다.

차량과 노트북은 같은 네트워크와 같은 `ROS_DOMAIN_ID`를 사용해야 합니다. Nav2와 SLAM이
동시에 `map -> odom` TF를 발행하지 않도록 mapping 중에는 `drive bringup_launch.xml`을
실행하지 않습니다.

실차 운용 기본 domain은 현재 `ROS_DOMAIN_ID=13`입니다. `/scan` timestamp가 계속 과거로
들어오는지 확인하려면 scan header stamp와 현재 ROS 시간을 비교합니다. 지연이 크면
차량 쪽 `pinky` 패키지를 다시 빌드하고 bringup을 재시작해야 합니다.
