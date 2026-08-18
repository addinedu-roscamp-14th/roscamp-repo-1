# porter_bringup

## 분리 도메인 + Zenoh 권장 구성

현재 권장 도메인은 중앙 `12`, AGV1 `13`, AGV2 `14`, ARM1 `15`, ARM2 `16`입니다.
각 장비는 로컬 DDS만 사용하고 Zenoh가 필요한 토픽, 서비스, 액션만 중앙으로
전달합니다. 이 구성에서는 `ROS_DISCOVERY_SERVER`를 사용하지 않습니다.

중앙 노트북:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PORT_CONTROL_API_TOKEN='porter1234'

zenoh-bridge-ros2dds -c config/network/zenoh_central.json5
```

다른 터미널:

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PORT_CONTROL_API_TOKEN='porter1234'

ros2 launch porter_bringup fleet_central_laptop.launch.py \
  start_discovery_server:=false \
  control_host:=0.0.0.0
```

ARM2 노트북은 먼저 `REAL`에 기록된 ARM2 launch를 실행한 뒤 별도 터미널에서
다음 브리지를 실행합니다. `<중앙_IP>`는 중앙 노트북의 로봇망 IP입니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=16
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

zenoh-bridge-ros2dds \
  -c config/network/zenoh_arm2.json5 \
  -e tcp/<중앙_IP>:7447
```

ARM1은 도메인 15와 `config/network/zenoh_arm1.json5`를 사용합니다. 이 설정은
Pick/Place뿐 아니라 초기 선박 마커 스캔, 입항 컨테이너 스캔, 스캔 결과와
인벤토리 이동 이벤트까지 중앙 관제로 전달합니다.

```bash
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=15
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
zenoh-bridge-ros2dds \
  -c config/network/zenoh_arm1.json5 \
  -e tcp/<중앙_IP>:7447
```

## 레거시 단일 도메인: 2대 Pinky 실행 순서

이 절은 Zenoh를 사용하지 않는 레거시 구성입니다. 이 방식에서만 두 차량과 중앙
노트북이 같은 `ROS_DOMAIN_ID`를 사용합니다. 각 차량에는
동일한 `config/SLAM/current_map.yaml`이 있어야 합니다.

각 차량에서 하드웨어 bringup, AMCL과 Nav2를 실행합니다. 중앙 노트북은
카메라·YOLO, fleet dispatcher, 제어 API와 RViz만 실행하며 Nav2를 중복 실행하지
않습니다.

### 1. 중앙 관제 노트북

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export PORT_CONTROL_API_TOKEN='porter1234'

ros2 launch porter_bringup fleet_central_laptop.launch.py
```

이 launch는 YOLO 기반 `fleet_collision_supervisor`도 기본 실행합니다. 두 차량의
예상 경로가 가까워지면 한 대의 `safety_hold`만 잠그고 위험 해제 후 기존 Nav2
목표를 이어서 수행합니다. 점검 토픽은 다음과 같습니다.

```bash
ros2 topic echo /central/fleet/collision_status
```

충돌 감독 없이 진단할 때만 다음 인자를 사용합니다.

```bash
ros2 launch porter_bringup fleet_central_laptop.launch.py \
  start_collision_supervisor:=false
```

### 2. AGV1 컴퓨터

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DISCOVERY_SERVER=10.121.206.28:11811

ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv1 \
  discovery_server:=$ROS_DISCOVERY_SERVER \
  start_nav2:=true
```

### 3. AGV2 컴퓨터

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DISCOVERY_SERVER=10.121.206.28:11811

ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv2 \
  discovery_server:=$ROS_DISCOVERY_SERVER \
  start_nav2:=true
```

각 차량 launch는 기본적으로 LiDAR 타임스탬프 필터를 사용합니다. Nav2가
활성화되기 전에 생성됐거나 네트워크/CPU 정체로 0.5초 이상 늦어진 스캔은
AMCL과 costmap에 전달하지 않습니다. 진단 시 허용 시간을 조정할 수 있습니다.

```bash
ros2 launch porter_bringup agv_vehicle.launch.py \
  vehicle_id:=agv1 \
  discovery_server:=$ROS_DISCOVERY_SERVER \
  scan_max_age_sec:=0.7 \
  start_nav2:=true
```

필터는 시스템 시계를 보정하지 않으므로 중앙과 두 차량 모두
`timedatectl show -p NTPSynchronized`가 `yes`인지 확인해야 합니다.

중앙 launch는 `0.0.0.0:11811`에서 Fast DDS Discovery Server를 함께 실행합니다.
따라서 중앙 launch를 먼저 실행하고 각 차량의 `discovery_server`에는 중앙
노트북의 실제 Wi-Fi IP를 넣습니다. 이 설정은 여러 Nav2 프로세스의 멀티캐스트
검색 트래픽을 줄이고 무선 연결이 반복해서 끊기는 현상을 완화합니다.

별도 터미널에서 `ros2 topic`, `ros2 action`, `rqt`를 실행할 때도 해당 컴퓨터의
`ROS_DISCOVERY_SERVER`를 위와 같이 먼저 설정해야 합니다. 중앙에서는
`127.0.0.1:11811`, 차량에서는 `<중앙_노트북_IP>:11811`을 사용합니다.

이 launch는 두 차량을 한 화면에 표시하는 저대역폭 fleet RViz를 함께 실행합니다.
지도, 저주기 차량 위치 마커, 경로와 두 RobotModel을 기본 표시하며 원격
`/scan`, `/odom`, `/particle_cloud`는 기본 비활성화합니다. 툴바의
`AGV1 Initial Pose`로 1번 차량을, `AGV2 Initial Pose`로 2번 차량을 실제
위치와 방향에 맞게 각각 설정합니다. 두 도구는 각각 `/agv1/initialpose`,
`/agv2/initialpose`에 발행합니다. RViz가 필요 없으면 `use_rviz:=false`를
추가합니다.

launch가 정상적으로 시작되면 터미널은 종료되지 않고 계속 실행 상태로
유지됩니다. RViz 창과 API 서버가 실행된 뒤에도 프롬프트로 돌아오지 않는 것이
정상입니다.

다중 차량 모드의 SLAM 웹 화면은 동일한 지도를 사용하는 `agv1`의
`/agv1/map`과 `/agv1/amcl_pose`를 표시합니다. LiDAR 오버레이는 무선 트래픽을
줄이기 위해 기본적으로 끕니다. 진단할 때만 다음 인자를 추가합니다.

```bash
ros2 launch porter_bringup fleet_central_laptop.launch.py \
  dashboard_enable_scan:=true
```

RViz의 실제 RobotModel은 두 대 모두 표시하되 5 Hz로만 갱신됩니다. 모델이
겹쳐 보이면 두 차량의 초기 위치가 아직 같은 위치로 설정된 것입니다. 툴바의
차량별 `Initial Pose`를 각각 지정합니다. `Fleet Vehicles`는 RobotModel과
별개인 2 Hz 경량 차량 마커이므로 항상 두 차량 상태를 확인할 수 있습니다.

`AGV1 Scan`, `AGV2 Scan` 체크는 진단할 때만 켭니다. 체크하는 동안 해당
LaserScan이 차량에서 중앙 노트북으로 계속 전송됩니다.
차량별 위치와 상태는 `/central/fleet/agv1/state`,
`/central/fleet/agv2/state`에서 동시에 관리됩니다.

### 4. 초기 위치 설정

각 차량의 실제 시작 위치와 방향을 별도로 발행합니다. RViz를 사용할 때도
차량에 맞는 `initialpose` 토픽을 선택해야 합니다.

```bash
ros2 topic pub --once /agv1/initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}'
```

```bash
ros2 topic pub --once /agv2/initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}'
```

위 예시는 원점이므로 실제 위치의 `x`, `y`, quaternion을 넣어야 합니다.
초기 pose를 받기 전 차량 상태는 `WAITING_FOR_INITIAL_POSE`이며 중앙 배차 대상에
포함되지 않습니다.

### 4.1 RViz에서 차량별 수동 목표 전송

중앙 fleet RViz 툴바에는 `2D Goal Pose` 도구가 두 개 있습니다. 첫 번째는
AGV1의 `/agv1/goal_pose`, 두 번째는 AGV2의 `/agv2/goal_pose`로 발행됩니다.
이 도구를 선택한 뒤 지도에서 목표 위치를 누르고 드래그해 최종 방향을
지정하면 중앙 목표 라우터가 각 차량의 namespaced Nav2 액션으로 전달합니다.

- 첫 번째 `2D Goal Pose`: `/agv1/navigate_to_pose`
- 두 번째 `2D Goal Pose`: `/agv2/navigate_to_pose`

### 5. AUTO 배차

```bash
curl -X POST http://127.0.0.1:8100/api/v1/navigation/pixel-goal \
  -H 'Content-Type: application/json' \
  -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}" \
  -d '{
    "command_id":"fleet-001",
    "vehicle_id":"",
    "target":{"x":320,"y":300},
    "heading":{"x":380,"y":300}
  }'
```

특정 차량은 `"vehicle_id":"agv1"` 또는 `"agv2"`로 지정합니다. B-1은
`"mode":"parking_b1"`을 추가합니다.
일반 새 명령은 해당 차량의 기존 목표를 취소하고 즉시 새 목표로 교체합니다.
LLM이 포괄적인 요청을 여러 단계로 계획하면 후속 명령에
`predecessor_command_id`와 `queue_if_busy:true`가 자동으로 붙고, 앞 단계 성공
후에만 실행됩니다. API에서 의도적으로 기존 작업 뒤에 붙일 때도 같은 두 필드를
사용합니다.

A-1/A-2/A-3(화물 적재 대기 구역)은 `"mode":"parking_a"`를 추가합니다. 셋 다
`camera_to_map_bridge`에 고정된 동일한 map pose(`a_zone_map_x`,
`a_zone_map_y`, `a_zone_map_yaw_deg` 파라미터, 기본값은 카메라 픽셀
`(157, 262)`를 현재 homography로 변환한 map 좌표
`(0.16812885, 0.06234431)`, yaw `90°`)에서 차량 헤딩 반대 방향으로
`a_zone_stop_back_offset_m=0.10m` 이동한 `(0.16812885, -0.03765569)`입니다.
`target`/`heading` 픽셀 값 자체는 무시됩니다(요청 형식을 맞추기 위한 값만 필요).

B-1과 A 구역 모두 한 번에 한 대만 점유할 수 있는 배타 구역이라 다른 차량은
바로 진입하지 않습니다. 요청 순서대로 FIFO 대기열에 등록되고, B-1은 최종
자세 뒤 `0.25m`, A 구역은 뒤 `0.20m` 지점까지 먼저 이동해 대기합니다. 점유
차량이 다른 목적지로 정상 출차하면 첫 번째 대기 차량이 자동으로 최종 위치에
진입합니다. 거리는 launch의 `b1_waiting_distance_m`,
`a_zone_waiting_distance_m`으로 변경할 수 있습니다.
중앙 노드가 재시작된 경우에도 요청 구역의 최종 map 좌표 반경 `0.18m` 안에
있는 차량을 최신 AMCL 위치로 찾아 점유 잠금을 복원합니다.

```bash
ros2 launch porter_bringup fleet_central_laptop.launch.py \
  b1_waiting_distance_m:=0.25 \
  a_zone_waiting_distance_m:=0.20
```

점유하던 차량이 오프라인이 되면 락이 자동으로
풀리지 않는데, 대시보드가 `"zone_visually_empty":true`를 함께 보내면(현재
카메라 프레임에 그 구역을 가리는 차량이 없을 때만 true) 텔레메트리로 이미
오프라인 판정된(`_zone_unknown`) 구역에 한해 자동으로 락이 해제됩니다.
차량이 아직 온라인이면 화면이 일시적으로 비어 보여도 절대 자동 해제되지
않습니다. 그래도 꼬였을 때 수동으로 강제 해제하려면:

```bash
curl -X POST http://127.0.0.1:8100/api/v1/zones/b1/clear \
  -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}"
curl -X POST http://127.0.0.1:8100/api/v1/zones/a/clear \
  -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}"
```

### 상태 확인

```bash
ros2 topic echo /central/fleet/agv1/state
ros2 topic echo /central/fleet/agv2/state
ros2 topic echo /central/fleet/zones
curl -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}" \
  http://127.0.0.1:8100/api/v1/status
```

전체 또는 개별 비상정지:

```bash
curl -X POST http://127.0.0.1:8100/api/v1/emergency-stop \
  -H 'Content-Type: application/json' \
  -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}" \
  -d '{"vehicle_id":"fleet","enabled":true}'
```

`vehicle_id`는 `fleet`, `agv1`, `agv2`이며 해제할 때는 `enabled`를 `false`로
설정합니다.

기존 `central_laptop.launch.py`와 단일 차량 launch는 그대로 사용할 수 있습니다.

## 무선 대역폭 최적화

다중 차량 launch의 기본값은 저대역폭 모드입니다.

- 중앙 dispatcher는 차량별 `/amcl_pose`와 배터리만 지속 구독합니다.
- 중앙 RViz는 지도, 저주기 차량 마커, 경로만 기본 표시합니다.
- 웹 SLAM 화면은 `/agv1/map`과 `/agv1/amcl_pose`를 사용합니다.
- 원격 LaserScan, odom, particle cloud는 기본 구독하지 않습니다.
- 두 RobotModel은 저주기 갱신하고 `Fleet Vehicles` 경량 마커를 함께 사용합니다.
- 차량 내부 Nav2는 기존 `/scan`, `/odom`, TF를 그대로 사용합니다.
- Nav2의 voxel map 발행과 full costmap 반복 발행은 끕니다.
- Fast DDS Discovery Server를 사용해 세 컴퓨터 사이의 검색 트래픽을 줄입니다.
- 물리 차량에서는 중복 `joint_state_publisher`를 실행하지 않습니다.
- LaserScan은 Best Effort, depth 1로 발행해 오래된 프레임 재전송을 막습니다.

변경 후 중앙 노트북과 두 차량에서 최신 코드를 빌드합니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  drive central dashboard porter_bringup
source install/setup.bash
```

중앙 launch 실행 전후의 구독자 수를 비교합니다.

```bash
ros2 topic info /agv1/scan
ros2 topic info /agv2/scan
ros2 topic info /agv1/odom
ros2 topic info /agv2/odom
```

차량 내부 Nav2도 `/scan`과 `/odom`을 구독하므로 구독자 수가 0일 필요는 없습니다.
저대역폭 중앙 launch를 켰을 때 `/scan`, `/odom` 구독자 수가 추가되지 않는 것이
정상입니다. `ros2 topic bw /agv1/scan` 자체도 임시 구독을 생성하므로 측정이
끝나면 `Ctrl+C`로 종료합니다.

Port-ER 시스템을 중앙제어 노트북과 대시보드 노트북으로 나눠 실행하는 launch
패키지입니다. 기존 개별 노드의 토픽과 실행 파일은 변경하지 않습니다.

## 빌드

두 노트북의 워크스페이스에서 실행합니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  porter_interfaces pinky central dashboard drive yolo porter_bringup
source install/setup.bash
```

## 중앙제어 노트북

차량과 같은 `ROS_DOMAIN_ID`를 사용합니다. API 토큰을 설정한 뒤 통합 launch를
실행합니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PORT_CONTROL_API_TOKEN='porter1234'

ros2 launch porter_bringup central_laptop.launch.py
```

다음 기능이 함께 실행됩니다.

- `/dev/video2` 탑다운 카메라
- `image_proc` 왜곡 보정
- `/image_rect/compressed` 발행
- YOLO
- 카메라·SLAM HTTP API (`8000`)
- Nav2와 RViz
- 카메라 픽셀 → map 좌표 변환
- `/central/target_map_pose` → Nav2 브리지
- 중앙제어 HTTP API (`8100`)

RViz가 열리면 차량을 움직이기 전에 `2D Pose Estimate`로 초기 위치와 방향을
설정합니다.

카메라 장치나 RViz 실행 여부를 바꾸는 예:

```bash
ros2 launch porter_bringup central_laptop.launch.py \
  video_device:=/dev/video0 \
  use_rviz:=false
```

이미 카메라 또는 Nav2를 따로 실행 중이면 중복 실행을 끕니다.

```bash
ros2 launch porter_bringup central_laptop.launch.py \
  start_camera:=false \
  start_nav2:=false
```

## 대시보드 노트북

최초 한 번 GUI Python 환경을 준비합니다.

```bash
cd ~/poter_ws
python3 -m venv .venv
.venv/bin/pip install -r port_control_system1/requirements.txt
```

중앙제어 노트북 IP를 인자로 전달해 실행합니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PORT_CONTROL_API_TOKEN='porter1234'

ros2 launch porter_bringup dashboard_laptop.launch.py \
  central_ip:=192.168.0.60
```

이 launch는 기본적으로 대시보드의 실시간 LLM 관제 에이전트도 켭니다. 사용자가
명령 창에서 한 번 실행한 목표를 유지하면서 최신 영상, YOLO 검출, 차량 상태와
구역 점유 변화를 2초 간격으로 확인합니다. 다음 인자로 주기를 조정할 수 있습니다.

```bash
ros2 launch porter_bringup dashboard_laptop.launch.py \
  central_ip:=192.168.0.60 \
  realtime_llm_enabled:=true \
  realtime_llm_interval_sec:=2.0 \
  realtime_llm_heartbeat_sec:=5.0 \
  realtime_llm_initial_delay_sec:=5.0
```

`realtime_llm_enabled:=false`로 시작해도 GUI 사이드바의 `LLM 실시간 관제`
스위치로 다시 켤 수 있습니다. 충돌 정지는 LLM 주기보다 빠른 중앙
`fleet_collision_supervisor`가 별도로 담당합니다.

현재 GUI의 차량 및 로봇팔 제어는 ROS2에 직접 연결되므로 대시보드 노트북도
차량과 같은 `ROS_DOMAIN_ID`를 사용해야 합니다. 카메라와 SLAM 영상 주소는
launch가 각각 다음 주소로 자동 설정합니다.

대시보드 launch는 Fast DDS를 UDP 전용으로 설정합니다. 여러 ROS 프로세스가
실행될 때 발생할 수 있는 `RTPS_TRANSPORT_SHM open_and_lock_file` 충돌을 피하면서
다른 노트북과의 DDS 통신은 유지합니다.

```text
http://<central_ip>:8000/video
http://<central_ip>:8000/slam/video
```

대시보드 launch는 다음 주소도 자동 설정합니다.

```text
중앙제어 API: http://<central_ip>:8100
Ollama: http://agent.sds.codes (팀 공유 서버)
기본 모델: gemma4:31b
```

로컬 모델명이 다르면 실행할 때 지정합니다.

```bash
ros2 launch porter_bringup dashboard_laptop.launch.py \
  central_ip:=192.168.0.60 \
  llm_model:=<설치된_비전_모델명>
```

### VLM 차량 이동 테스트

대시보드의 `명령` 창에서 현재 탑다운 영상을 기준으로 명령합니다.

```text
현재 영상에서 중앙의 빈 공간으로 이동하고 오른쪽을 바라봐
노란 차 상차하러 보내줘
항구에 차량 한 대 배차해
차량 한 대를 A구역에 대기시켜
```

항구·항만·부두·상차·하차·선적·하역은 `B-1`의 포괄 표현으로, A구역·A존·적재
대기는 현재 검출된 `A-1`/`A-2`/`A-3`의 포괄 표현으로 처리합니다. LLM이
`unknown`을 반환하거나 검출 인덱스/접근 방향을 생략해도 현재 YOLO JSON에 대응
구역이 있으면 대시보드가 보완합니다. 검출되지 않은 대상과 이동 금지 명령에는
임의 좌표를 만들지 않습니다.

처리 흐름:

```text
현재 탑다운 JPEG
YOLO 검출 JSON (/central/yolo/detections → /detections)
→ VLM이 detection_index와 접근 방향 선택
→ 대시보드가 bbox 기준 target/heading 계산
→ 중앙제어 API 8100
→ /central/target_pixel
→ /central/target_map_pose
→ Nav2
```

`B-1`은 항구 상차·하차 전용 주차 구역입니다. B-1 명령에서는 구역 중심과
세그멘테이션 헤딩을 사용하고, map 변환 후 카메라 영상의 왼쪽 방향으로 기본
`0.15m`, 화면 아래쪽으로 `0.03m` 이동합니다. 실행 시
`b1_camera_left_offset_m:=0.15`, `b1_camera_down_offset_m:=-0.02`으로 거리를
조정할 수 있습니다.

RViz에서 `2D Pose Estimate`로 초기 위치를 지정한 후 테스트해야 합니다.

검출 객체 접근 명령 예:

```text
영상에서 파란 차량의 왼쪽으로 접근해서 차량을 바라봐
```

정상 실행 로그:

```text
[VLM 영상] 현재 프레임 640x480을 함께 전송합니다.
[VLM YOLO JSON] 2개 검출: 0:car_blue, 1:trailer
[VLM 원본 응답] ... visual_navigation ...
[VLM 객체 접근 좌표 계산] ... target=..., heading=...
```

검출 API 확인:

```bash
curl http://127.0.0.1:8000/detections
```

## 주요 인자

### `central_laptop.launch.py`

| 인자 | 기본값 | 기능 |
| --- | --- | --- |
| `video_device` | `/dev/video2` | 탑다운 카메라 장치 |
| `use_rviz` | `true` | 초기 위치 설정용 RViz |
| `start_camera` | `true` | 카메라·보정·압축 노드 |
| `start_yolo` | `true` | YOLO 노드 |
| `start_dashboard_api` | `true` | 영상 API 서버 |
| `start_nav2` | `true` | Nav2 |
| `start_navigation_control` | `true` | 좌표 변환·목표 브리지·제어 API |
| `nav2_params_file` | `drive/params/nav2_params.yaml` | Nav2 전용 파라미터 |
| `control_host` | `0.0.0.0` | 제어 API bind 주소 |

### `dashboard_laptop.launch.py`

| 인자 | 기본값 | 기능 |
| --- | --- | --- |
| `central_ip` | 필수 | 중앙제어 노트북 IP |
| `video_port` | `8000` | `dashboard_stream_node` HTTP 포트 (중앙 `config/dashboard/dashboard.yaml`의 `port`와 일치해야 함) |
| `python_executable` | `~/.venv/bin/python` | GUI 실행 Python |
| `api_token` | `PORT_CONTROL_API_TOKEN` | 중앙제어 API 토큰 |
| `ollama_host` | `http://agent.sds.codes` | VLM 서버 (팀 공유, 로컬로 바꾸려면 `http://127.0.0.1:11434` 지정) |
| `llm_model` | `gemma4:31b` | 비전 모델 이름 |
| `llm_num_ctx` | `8192` | 이미지·YOLO JSON을 포함한 VLM 요청의 Ollama 컨텍스트 크기 |
