# porter_bringup

## 2대 Pinky 실행 순서

두 차량과 중앙 노트북은 같은 `ROS_DOMAIN_ID`를 사용해야 합니다. 각 차량에는
동일한 `config/SLAM/current_map.yaml`이 있어야 합니다.

### 1. AGV1 컴퓨터

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13

ros2 launch porter_bringup agv_vehicle.launch.py vehicle_id:=agv1
```

### 2. AGV2 컴퓨터

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13

ros2 launch porter_bringup agv_vehicle.launch.py vehicle_id:=agv2
```

### 3. 중앙 관제 노트북

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13
export PORT_CONTROL_API_TOKEN='porter1234'

ros2 launch porter_bringup fleet_central_laptop.launch.py
```

다중 차량 모드의 SLAM 웹 화면은 동일한 지도를 사용하는 `agv1`의
`/agv1/map`, `/agv1/scan`, `agv1/base_footprint`를 기본으로 표시합니다.
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
export ROS_DOMAIN_ID=<차량과_같은_번호>
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
export ROS_DOMAIN_ID=<차량과_같은_번호>
export PORT_CONTROL_API_TOKEN='porter1234'

ros2 launch porter_bringup dashboard_laptop.launch.py \
  central_ip:=192.168.0.60
```

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
로컬 Ollama: http://127.0.0.1:11434
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
```

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
`b1_camera_left_offset_m:=0.15`, `b1_camera_down_offset_m:=0.03`으로 거리를
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
| `python_executable` | `~/.venv/bin/python` | GUI 실행 Python |
| `api_token` | `PORT_CONTROL_API_TOKEN` | 중앙제어 API 토큰 |
| `ollama_host` | `http://127.0.0.1:11434` | 로컬 VLM 서버 |
| `llm_model` | `gemma4:31b` | 비전 모델 이름 |
