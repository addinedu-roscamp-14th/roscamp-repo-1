# Central Package

## 중앙 RQT costmap 조정

`fleet_central_laptop.launch.py`는 Zenoh 너머의 Nav2 파라미터 서비스를 중앙
ROS graph에 노출하는 프록시 노드 4개를 실행합니다. RQT Parameter Reconfigure
목록에서 `agv1_global_costmap_tuning`, `agv1_local_costmap_tuning`,
`agv2_global_costmap_tuning`, `agv2_local_costmap_tuning`을 선택해 inflation
radius와 cost scaling factor를 실시간으로 변경할 수 있습니다. 변경은 원격
costmap에 즉시 전달되지만 Nav2 재시작 후에는 YAML 값으로 돌아갑니다.

중앙 관제에서 카메라 픽셀 좌표를 SLAM `/map` 좌표로 변환하고, 차량/브릿지에 전달 가능한 형태로 발행하는 패키지입니다.

현재 핵심 노드는 `rqt_click_to_target`, `camera_to_map_bridge`,
`control_gateway`, `fleet_dispatcher`, `fleet_collision_supervisor`입니다.

## 로봇팔 중앙 연동

중앙은 `/central/arms/dispatch` 액션으로 ARM 명령을 FIFO 처리합니다. ARM1은
`/arm/pick_place/execute`, `/arm/pick_place/stop` 서비스와
`/arm/pick_place/work_state` 상태 계약을 사용합니다. 중앙은 새 `WORK_STARTED` 이후
`WORK_COMPLETED`, `FAILED`, `STOPPED` 중 하나가 올 때까지 명령을 완료하지 않습니다.
`execute` 요청의 `pick_id/place_id`는 중앙 action의
`source_id/destination_id`에서 매 작업마다 전달됩니다.
차량 연계 작업은 ARM1의 경우 차량 상태가 `READY`이고 `locked_zone=B-1`,
ARM2의 경우 `READY`이고 `locked_zone=A`가 된 뒤에만 시작합니다.
ARM2도 직접 서비스 응답을 작업 완료로 간주하지 않습니다. 반드시
`/arm2/transfer_events`에서 같은 `operation_id`의 최종 `COMPLETED` 또는 `FAILED`
이벤트를 받은 뒤 중앙 결과를 확정합니다.

허용 작업은 다음으로 제한됩니다.

- ARM1: `pick_place`, `stop`
- `scan_destinations`
- `transfer_to_slot`: 차량에서 창고로 이동
- `load_to_trailer`: 창고에서 차량으로 이동
- `transfer_by_id`: 창고 내부 이동
- `go_pose`, `reset_stack_level`, `stop`

차량 출발 승인은 `pick_place`, `transfer_to_slot` 또는 `load_to_trailer`가 최종
성공하고 요청의 `final_for_vehicle`가 참일 때만 허용됩니다.
`transfer_by_id` 성공은 차량 출발 조건이 아닙니다.

### 출발 게이트

같은 조건(`final_for_vehicle` + 차량 지정 + 위 두 이송 작업)을 만족하는 명령이
큐에 들어가는 순간부터 종료될 때까지, `arm_dispatcher`는 해당 차량을 붙잡고
있다고 `/central/arms/vehicle_holds`에 2Hz로 알립니다. `fleet_dispatcher`는 이
스냅샷을 구독해서, 붙잡힌 차량의 주행 명령을 바퀴가 돌기 직전 단계에서
대기시킵니다(피드백 상태 `WAITING_FOR_ARM`). 팔 작업이 끝나면 대기가 풀리고
주행이 이어집니다.

- 스냅샷은 **주기 발행**이며 매번 집합 전체를 교체합니다. 메시지를 놓쳐도 다음
  스냅샷에서 스스로 복구되고, latch로 인해 낡은 hold가 재생되는 일도 없습니다.
- 작업이 **실패해도** hold는 풀립니다. 차가 갇히는 쪽이 더 위험하기 때문이며,
  실패 사실은 작업 결과로 별도 통보됩니다.
- 대기는 `cargo_hold_timeout_sec`(기본 300초)에서 끊기고 명령은 abort 됩니다.
- 대시보드의 수동 목표(`/agvX/goal_pose`)는 `fleet_dispatcher`를 거치지 않으므로
  이 게이트의 적용을 받지 않습니다. 운용자 비상 수단으로 열어둔 경로입니다.

```bash
ros2 topic echo /central/arms/vehicle_holds
```

HTTP API 예시:

```bash
curl -X POST http://127.0.0.1:8100/api/v1/arms/commands \
  -H 'Content-Type: application/json' \
  -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}" \
  -d '{
    "command_id":"arm2-load-001",
    "mission_id":"mission-001",
    "arm_id":"arm2",
    "operation":"load_to_trailer",
    "source_id":3,
    "vehicle_id":"agv1",
    "final_for_vehicle":true
  }'
```

작업 상태 확인:

```bash
ros2 topic echo /central/arms/arm1/state
ros2 topic echo /central/arms/arm2/state
ros2 topic echo /central/arms/results
ros2 topic echo /central/autonomy/vehicle_release
```

## 입항 자동 감지

`port_event_detector`는 대시보드에서 지정한 탑다운 카메라 ROI 안의 YOLO OBB를
검사합니다. 신뢰도 `0.65` 이상, ROI 겹침 `30%` 이상이 최근 5프레임 중 3프레임에
있으면 입항으로 판정합니다. 10초 동안 사라지면 출항으로 판정합니다. 입항 시
`autonomy_orchestrator`가 ARM2 목적지 스캔을 요청하고 화물 정책이 준비될 때까지
`WAITING_FOR_CARGO_POLICY`로 대기합니다.

## 2대 차량 Fleet 제어

`fleet_dispatcher`는 `agv1`, `agv2`의 Nav2 액션과 상태를 통합합니다.

```text
/central/dispatch_navigation
porter_interfaces/action/DispatchNavigation

/central/fleet/agv1/state
/central/fleet/agv2/state
porter_interfaces/msg/VehicleState

/central/fleet/zones
std_msgs/String

/central/fleet/vehicle_markers
visualization_msgs/MarkerArray
```

AUTO 명령은 준비된 유휴 차량 중 목표와 가까운 차량을 선택하며 동률이면
`agv1`을 선택합니다. `vehicle_id`가 지정되면 다른 차량으로 대체하지 않습니다.
기본 새 명령은 선택된 차량의 기존 Nav2 목표와 대기 명령을 취소하고 새 목표로
교체합니다. 포괄적인 사용자 명령의 독립적인 차량 작업은 기본적으로 동시에
전송합니다. 같은 차량의 연속 목표나 물리적 선행 조건이 있는 단계에만
`predecessor_command_id`를 연결하며, 이전 단계가 실패하거나 취소되면 해당 후속
단계를 실행하지 않습니다.
명시적으로 현재 작업 뒤에 대기시킬 명령만 `queue_if_busy: true`를 사용합니다.
B-1과 공용 A 구역은 각각 한 차량만 점유할 수 있습니다. 이미 점유 중이면 새
차량은 FIFO 대기열에 들어간 뒤 최종 자세의 뒤쪽 대기점까지 먼저 이동합니다.
dispatcher는 최신 AMCL 위치를 기본 `2Hz`로 감시합니다. 점유 차량이 A 또는
B-1 목표 반경에 실제로 진입한 뒤 구역 밖으로 나가면, 다음 목적지 도착이나 LLM
재호출을 기다리지 않고 잠금을 해제하여 대기열 첫 차량을 최종 위치로 보냅니다.
AUTO 요청에서는 현재 점유 차량을 후보에서 제외합니다. 중앙 노드 재시작으로
잠금 메모리가 비어 있어도 최종 구역 좌표 반경 `0.18m` 안의 최신 AMCL 차량을
점유자로 복원합니다. 진입 반경은 `zone_occupancy_radius_m`, 경계에서 잠금이
반복 전환되지 않도록 하는 추가 해제 여유는 `zone_release_hysteresis_m`(기본
`0.05m`)으로 조정합니다.
점유 차량의 통신이 끊기면 안전을 위해 `UNKNOWN`으로 잠긴 상태를 유지합니다.
B-1 점유 차량이 B-1 이외의 목적지 명령을 받으면 최종 이동이나 구역 대기점 이동
전에 Nav2 `Spin`으로 왼쪽 `90°` 제자리 회전합니다. 회전 액션이 성공해야 다음
단계로 넘어가며, 회전이 끝나면 `DriveOnHeading`으로 차량 전방 `0.30m`를 직진한
뒤 최종 목적지 경로를 시작합니다. 기본 각도는 `b1_exit_left_turn_deg`, 직진
거리는 `b1_exit_forward_distance_m`, 직진 속도는
`b1_exit_forward_speed_mps`(기본 `0.05m/s`)로 관리합니다. 각 동작 제한 시간은
`b1_exit_behavior_timeout_sec`(기본 `20초`)입니다. 회전 또는 직진이 실패하면 최종
목적지 경로를 시작하지 않습니다. `Spin` 성공 후에도 AMCL map 헤딩을 검사하며,
`b1_exit_turn_tolerance_deg`(기본 `5도`)를 벗어나면 잔여 각도를 최대 두 번
추가 보정합니다. B-1 잠금이 위치 오차로 먼저 해제되는 경우를 위해 B-1 목표
`b1_exit_detection_radius_m`(기본 `0.35m`) 안의 차량에도 같은 이탈 시퀀스를
적용합니다.
중앙 노드를 재시작한 경우에도 `b1_zone_map_x`, `b1_zone_map_y`로 등록된 B-1
기준점과 각 차량의 최신 AMCL 위치를 비교해 B-1 점유 차량을 복원합니다. 따라서
메모리 잠금이 없어도 B-1 반경 안에 있는 명령 대상 차량에는 같은 이탈 시퀀스가
적용됩니다. 현재 지도 기본값은 `(1.294, -0.087)`입니다.
실시간 LLM이 같은 목적지를 새 command ID로 반복 보내더라도, 동일 차량의 진행 중
목표와 최종 위치 `0.12m` 및 헤딩 `20도` 이내이면 기존 작업에 병합합니다. 따라서
B-1 이탈 회전과 전진이 중간에 재시작되지 않으며, 실제로 다른 목적지만 현재 작업을
선점합니다. 허용치는 `duplicate_goal_distance_m`와
`duplicate_goal_yaw_tolerance_deg`로 조정합니다.

## 자동 후진 주차

각 차량은 공유 없이 자기 전용 주차 스팟만 사용합니다: `agv1`은
`park_red`(구역 `PARK1`), `agv2`는 `parking_yellow`(구역 `PARK2`)입니다
(`drive/params/parking_spots.yaml`). 서로 다른 자리라서 B-1/A처럼 FIFO 대기가
필요 없고, 항상 자기 자리로만 갑니다. `parking_yellow`의 approach 좌표와 두
yaw 값은 AGV2에서 측정한 전용 캘리브레이션 값입니다.

`fleet_dispatcher`는 두 가지 경로로 주차를 트리거합니다.

- **유휴 자동 주차**: 차량이 `auto_park_idle_sec`(기본 `20초`) 이상 READY 상태로
  가만히 있으면(바쁘지도, 이미 자기 자리에 주차돼 있지도 않으면) 자동으로
  자기 전용 스팟으로 후진 주차를 시작합니다. `auto_park_check_interval_sec`
  (기본 `3초`)마다 재확인합니다. `auto_park_idle_sec:=0`으로 끌 수 있습니다.
- **명시적 명령**: `/central/fleet/park_request`(`std_msgs/String`, `data`에
  `agv1`/`agv2`/빈 문자열)를 publish하면 즉시 해당(또는 유휴 중 아무) 차량을
  주차시킵니다. `control_gateway`의 `POST /api/v1/navigation/park`가 이 토픽으로
  중계합니다.

실제 후진 동작은 `drive` 패키지의 `parking_new` 노드(`/{vehicle_id}/park_in_spot`
액션)가 수행하며, `multi_vehicle_nav.launch.py`에서 각 차량 네임스페이스로 자동
실행됩니다(`start_parking_supervisor:=false`로 끌 수 있음).

주차가 완료된 차량에 다른 목적지 명령이 들어오면 일반 Nav2 경로를 보내기 전에
`DriveOnHeading`으로 현재 차량 전방을 따라 반드시 `0.20m` 직진합니다. 이 출차가
성공한 뒤에만 주차 구역 잠금을 해제하고 목적지 경로를 시작합니다. 거리와 속도는
각각 `park_exit_forward_distance_m`(기본 `0.20`)와
`park_exit_forward_speed_mps`(기본 `0.05`)로 조정합니다. 중앙 노드를 재시작해도
차량이 자기 주차 기준점 근처에 있으면 같은 출차 절차를 적용합니다.

## YOLO 차량 충돌 감독

`fleet_collision_supervisor`는 `/central/yolo/detections`의 `car_yellow`
(`agv1`)와 `car_blue`/`car_bule`(`agv2`) 중심을 카메라-map Homography로
변환합니다. 최근 검출 속도, `/<vehicle_id>/odom`, `/<vehicle_id>/plan`을 함께
사용해 기본 3초 동안 두 차량의 위치를 예측합니다. 한 차량이 카메라에서 잠시
가려지면 최신 AMCL 기반 `VehicleState.pose`를 대신 사용합니다. 예상 최소 거리가
`0.22m` 이하이면 한 차량의 `/<vehicle_id>/safety_hold`를 잠급니다. 거리가 `0.30m`
이상으로 0.8초간 유지되면 같은 Nav2 목표를 재개합니다.
차량 설치본이 아직 `safety_hold` 서비스를 제공하지 않으면 기존
`/<vehicle_id>/emergency_stop`을 자동 hold 채널로 사용하며, 위험 해제 시 같은
서비스를 통해 정지를 해제합니다. 최종 구성에서는 차량의 `pinky` 패키지도
업데이트하여 `safety_hold`를 사용하는 것이 권장됩니다.

```bash
ros2 topic echo /central/fleet/collision_status
```

감독 기능을 일시적으로 끄면서 자동 hold를 해제하려면 다음 서비스를 사용합니다.

```bash
ros2 service call /central/fleet/collision_supervisor/enabled \
  std_srvs/srv/SetBool '{data: false}'
```

직접 차량 hold를 해제할 때는 감독 기능을 먼저 끈 뒤 실행합니다.

```bash
ros2 service call /agv1/safety_hold std_srvs/srv/SetBool '{data: false}'
```

임계값과 예측 시간은 `config/central/fleet_collision_supervisor.yaml`에서
관리합니다. YOLO와 AMCL Fleet pose가 모두 없거나 오래된 차량이 있으면 새 자동
hold를 걸지 않고 차량 LiDAR와 Nav2가 로컬 충돌 방지를 담당합니다. 이미 hold된
상태에서 위치 입력이 끊기면 입력이 복구되거나 운영자가 명시적으로 해제할 때까지
hold를 유지합니다. 상태 JSON의 `position_source`에서 차량별로 `vision`, `fleet`,
`missing` 중 어느 위치를 사용 중인지 확인할 수 있습니다.

`fleet_dispatcher`는 `/<vehicle_id>/odom`을 온라인 상태 확인용으로 구독하고,
배차 거리 계산에는 `/<vehicle_id>/amcl_pose`를 사용합니다. 초기 위치 설정 전
odom 위치를 임시 배차 좌표로 사용할 때만 `subscribe_odom_fallback:=true`로
실행합니다.

## `control_gateway`

AI/LLM 서버가 계산한 탑다운 카메라 픽셀 목표를 HTTP JSON으로 받고, 기존
`camera_to_map_bridge`가 사용하는 `/central/target_pixel`에 목표점과 방향점을
순서대로 발행합니다. 직접 map 좌표를 생성하지 않으므로 기존 카메라-SLAM
캘리브레이션을 그대로 거칩니다.

입력 API:

```text
POST http://<관제노트북_IP>:8100/api/v1/navigation/pixel-goal
```

ROS 출력:

```text
/central/target_pixel
geometry_msgs/PointStamped

/central/control/status
std_msgs/String
```

관제 상태 API:

```text
GET http://<관제노트북_IP>:8100/api/v1/status
GET http://<관제노트북_IP>:8100/health
```

상태 API에는 `/battery/percent`, `/battery/voltage`, `/odom`,
`/central/target_map_json`의 최신값과 데이터 나이가 포함됩니다.

## `rqt_click_to_target`

rqt image view에서 마우스로 클릭한 픽셀 좌표를 `/central/target_pixel`로 변환해 발행합니다.

입력:

```text
/central/yolo/image_annotated_mouse_left
geometry_msgs/Point
```

출력:

```text
/central/target_pixel
geometry_msgs/PointStamped
```

## `camera_to_map_bridge`

캘리브레이션 결과 YAML을 읽어서 카메라 픽셀 좌표 `(u, v)`를 SLAM map 좌표 `(x, y)`로 변환합니다.

시작할 때 캘리브레이션 YAML에 기록된 지도 크기, 해상도, 원점과 현재 map
YAML/PGM을 비교합니다. `config/SLAM/current_map.yaml` 또는 PGM을 교체했다면 기존
호모그래피를 재사용할 수 없으므로 현재 지도로 다시 캘리브레이션해야 합니다. 지도
정보가 다르면 잘못된 Nav2 목표 발행을 막기 위해 노드가 시작되지 않습니다.

`parking_b1` API 명령은 B-1 중심을 map 좌표로 변환한 뒤, 탑다운 카메라 영상의
왼쪽 방향을 같은 호모그래피로 계산하여 기본 `0.15m`, 화면 아래쪽으로 기본
`0.03m` 이동합니다. 목표와 헤딩점을 같이 이동하므로 B-1 정렬 방향은 유지됩니다.
거리는 `b1_camera_left_offset_m`, `b1_camera_down_offset_m` 파라미터로 조정합니다.

점유 구역 대기점은 최종 헤딩의 반대 방향으로 계산합니다.

- `b1_waiting_distance_m`: B-1 대기 거리, 기본 `0.25m`
- `a_zone_waiting_distance_m`: 공용 A 구역 대기 거리, 기본 `0.20m`
- `a_zone_waiting_camera_down_offset_m`: A 대기점을 카메라 화면 아래 방향으로
  추가 이동하는 거리, 기본 `0.05m`

예를 들어 B-1 차량은 그대로 두고 다음 차량만 미리 대기시키려면 B-1 요청을
먼저 보냅니다. 이후 점유 차량에 B-1이 아닌 목적지 명령을 보내면, 출차 성공
시 잠금이 풀리고 대기 차량이 자동으로 B-1에 진입합니다.

입력:

```text
/central/target_pixel
geometry_msgs/PointStamped
```

출력:

```text
/central/target_map_pose
geometry_msgs/PoseStamped

/central/target_map_json
std_msgs/String

/central/target_map_waypoints
nav_msgs/Path

/central/target_map_waypoints_preview
nav_msgs/Path
```

기본 calibration 파일:

```text
config/central/camera_map_calibration.yaml
```

이 파일은 `calibration` 패키지의 `direct_calibrator`로 생성합니다.

## Build

워크스페이스 루트에서 실행합니다.

```bash
cd ~/poter_ws
colcon build
source install/setup.bash
```

## Run

전체 실행 흐름:

```text
rqt click
→ /central/yolo/image_annotated_mouse_left
→ rqt_click_to_target
→ /central/target_pixel
→ camera_to_map_bridge
→ /central/target_map_pose
→ /central/target_map_json
```

### AI 좌표 자동 전송

단일 목표 모드로 카메라-map 변환 노드와 Nav2 목표 브리지를 먼저 실행합니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run central camera_to_map_bridge
```

```bash
ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

별도 터미널에서 중앙제어 API를 실행합니다. 영상/SLAM API의 기본 포트는
`8000`, 제어 API의 기본 포트는 `8100`입니다.

```bash
cd ~/poter_ws
source install/setup.bash
ros2 launch central control_gateway.launch.py
```

기본값은 관제 노트북 내부의 `127.0.0.1`에서만 접속할 수 있습니다. 외부 AI
서버가 접속해야 할 때는 토큰을 설정하고 전체 네트워크 인터페이스에 엽니다.

```bash
export PORT_CONTROL_API_TOKEN='<충분히_긴_임의의_문자열>'
ros2 launch central control_gateway.launch.py host:=0.0.0.0
```

AI 서버는 탑다운 영상의 목표 픽셀과 차량이 바라볼 방향 픽셀을 함께 보냅니다.

```bash
curl -X POST http://127.0.0.1:8100/api/v1/navigation/pixel-goal \
  -H 'Content-Type: application/json' \
  -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}" \
  -d '{
    "command_id": "dispatch-001",
    "vehicle_id": "",
    "zone_id": "",
    "target": {"x": 320, "y": 300},
    "heading": {"x": 380, "y": 300}
  }'
```

`vehicle_id`는 빈 문자열(AUTO), `agv1`, `agv2` 중 하나입니다. B-1 주차는
`mode: "parking_b1"` 또는 `zone_id: "B-1"`을 사용합니다.

비상정지:

```bash
curl -X POST http://127.0.0.1:8100/api/v1/emergency-stop \
  -H 'Content-Type: application/json' \
  -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}" \
  -d '{"vehicle_id":"fleet","enabled":true}'
```

`fleet` 대신 `agv1` 또는 `agv2`를 지정할 수 있습니다. 해제는
`"enabled":false`로 요청합니다.

B-1 `UNKNOWN` 잠금은 차량 위치를 확인한 운영자만 해제합니다.

```bash
curl -X POST http://127.0.0.1:8100/api/v1/zones/b1/clear \
  -H "X-Control-Token: ${PORT_CONTROL_API_TOKEN}"
```

`command_id`는 재시도 시 같은 명령이 두 번 실행되는 것을 막는 선택값입니다.
같은 ID를 다시 보내면 게이트웨이는 성공 응답을 반환하지만 ROS 목표는 재발행하지
않습니다. 좌표는 `640x480` 영상 범위 안이어야 하고 목표와 방향점은 기본 10픽셀
이상 떨어져야 합니다.

상태 확인:

```bash
curl http://127.0.0.1:8100/api/v1/status
ros2 topic echo /central/control/status
ros2 topic echo /central/target_map_pose
```

`camera_to_map_bridge`가 실행되지 않아 `/central/target_pixel` 구독자가 없으면
제어 API는 명령을 버리지 않고 HTTP `503`으로 거부합니다.
외부 접속 모드에서는 `/api/v1/*` 요청에 동일한 `X-Control-Token` 헤더가
없으면 HTTP `401`로 거부합니다. `/health`는 프로세스 확인용으로 공개됩니다.

---

# 노트북에서 실행


### rqt 클릭 좌표 발행

먼저 rqt image view에서 `/central/yolo/image_annotated`를 열고 마우스로 클릭합니다.

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run central rqt_click_to_target
```

클릭 좌표 확인:

```bash
ros2 topic echo /central/target_pixel
```

### map 좌표 변환

```bash
cd ~/poter_ws
source install/setup.bash
ros2 run central camera_to_map_bridge
```
### Nav2 
```bash
ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

### 목표 위치와 방향 클릭

rqt 영상에서 두 번 클릭합니다.

1. 첫 번째 클릭: 차량이 도착할 목표 위치
2. 두 번째 클릭: 목표 위치에서 차량이 바라볼 방향

두 번째 클릭 지점은 차량이 이동할 위치가 아니라 방향을 계산하기 위한 점입니다.
브릿지는 두 점을 모두 map 좌표로 변환하고 첫 번째 점에서 두 번째 점을 향하는 yaw를
계산한 뒤 `/central/target_map_pose`를 한 번 발행합니다.

## 여러 웨이포인트 전송

웨이포인트 모드에서는 중간 지점과 최종 목적지를 목록에 누적한 후 한 번에 차량으로
보냅니다. 중간 지점의 헤딩은 다음 지점을 향하도록 자동 계산합니다.

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p waypoint_mode:=true
```

영상에서 다음 순서로 클릭합니다.

1. 중간 웨이포인트: 각각 위치만 한 번 클릭
2. 최종 목적지: 위치를 한 번 클릭
3. 최종 방향: 최종 목적지에서 차량이 바라볼 방향점을 한 번 클릭

예를 들어 중간 웨이포인트가 2개라면 `중간1 -> 중간2 -> 최종 위치 -> 최종 방향`으로
총 네 번 클릭합니다. 마지막 방향점은 차량이 이동할 웨이포인트에 포함되지 않습니다.

필요한 웨이포인트를 모두 찍은 다음 `camera_to_map_bridge`를 실행한 터미널에 포커스를
두고 **스페이스바**를 누르면 전체 경로가 차량으로 전송됩니다.

스페이스바 입력은 브릿지 터미널에 포커스가 있을 때만 동작합니다. 터미널 입력을 사용할
수 없는 환경에서는 기존 서비스를 호출합니다.

```bash
ros2 service call /central/commit_waypoints std_srvs/srv/Trigger "{}"
```

확정하면 `/central/target_map_waypoints`에 `nav_msgs/Path`가 한 번 발행됩니다. 작업 중인
전체 클릭은 `/central/target_map_waypoints_preview`에서 확인할 수 있으며, 마지막 클릭은
확정 시 최종 방향점으로 해석됩니다.

잘못 찍었거나 처음부터 다시 찍으려면 전체 목록과 진행 중인 첫 클릭을 초기화합니다.

```bash
ros2 service call /central/clear_waypoints std_srvs/srv/Trigger "{}"
```

차량 측 Nav2 웨이포인트 브릿지:

```bash
ros2 launch drive target_map_waypoints_nav.launch.xml start_nav2:=false
```

단일 목적지를 사용할 때는 `waypoint_mode`를 켜지 않고 기존
`target_map_pose_nav.launch.xml`을 사용합니다. 두 drive 브릿지를 동시에 실행하지 않습니다.

# 차량에서 실행

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

차량에서는 센서, odometry와 모터 제어만 실행합니다. Nav2와 목표 브릿지는 노트북에서
실행합니다.

```bash
ros2 launch drive bringup_launch.xml \
  map:=/home/jio/poter_ws/config/SLAM/current_map.yaml

ros2 launch drive nav2_view.launch.xml

ros2 launch drive target_map_pose_nav.launch.xml start_nav2:=false
```

RViz에서 `2D Pose Estimate`로 실제 차량의 초기 위치와 방향을 지정해야 합니다.



기본 설정:

```text
calibration_yaml: config/central/camera_map_calibration.yaml
input_pixel_topic: /central/target_pixel
output_pose_topic: /central/target_map_pose
output_json_topic: /central/target_map_json
frame_id: map
minimum_direction_distance: 0.02
waypoint_mode: false
enable_spacebar_commit: true
output_waypoints_topic: /central/target_map_waypoints
output_waypoints_preview_topic: /central/target_map_waypoints_preview
commit_waypoints_service: /central/commit_waypoints
clear_waypoints_service: /central/clear_waypoints
```

## Test

목표 위치와 방향점을 순서대로 직접 발행합니다.

```bash
ros2 topic pub --once /central/target_pixel geometry_msgs/msg/PointStamped \
"{header: {frame_id: camera}, point: {x: 160.0, y: 355.0, z: 0.0}}"

ros2 topic pub --once /central/target_pixel geometry_msgs/msg/PointStamped \
"{header: {frame_id: camera}, point: {x: 260.0, y: 355.0, z: 0.0}}"
```

PoseStamped 출력 확인:

```bash
ros2 topic echo /central/target_map_pose
```

JSON 출력 확인:

```bash
ros2 topic echo /central/target_map_json
```

## JSON Format

`/central/target_map_json`은 `std_msgs/String` 안에 JSON 문자열로 발행됩니다.

예시:

```json
{
  "frame_id": "map",
  "target_id": "target",
  "source_frame_id": "camera",
  "stamp": {
    "sec": 0,
    "nanosec": 0
  },
  "target_camera_pixel": {
    "u": 160.0,
    "v": 355.0
  },
  "direction_camera_pixel": {
    "u": 260.0,
    "v": 355.0
  },
  "direction_map_point": {
    "x": -0.124279,
    "y": -1.164190
  },
  "map_pose": {
    "x": -0.330455,
    "y": -1.495575,
    "z": 0.0,
    "yaw": 1.014239,
    "heading_deg": 58.112
  }
}
```

## PoseStamped Format

`/central/target_map_pose`는 Nav2 또는 차량 브릿지에서 바로 읽기 좋은 `geometry_msgs/PoseStamped`입니다.

```text
header.frame_id: map
pose.position.x: map x
pose.position.y: map y
pose.position.z: 0.0
pose.orientation: 첫 번째 map 점에서 두 번째 map 점을 향하는 yaw quaternion
```

노트북의 `drive target_map_pose_to_nav_goal` 노드가 이 좌표를 Nav2 `NavigateToPose`
goal로 넘깁니다.

## Parameters

다른 calibration YAML 사용:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p calibration_yaml:=config/central/camera_map_calibration.yaml
```

입력/출력 토픽 변경:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p input_pixel_topic:=/central/target_pixel \
  -p output_pose_topic:=/central/target_map_pose \
  -p output_json_topic:=/central/target_map_json
```

방향점 최소 거리 변경:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p minimum_direction_distance:=0.05
```

target id 변경:

```bash
ros2 run central camera_to_map_bridge --ros-args \
  -p target_id:=AGV_goal_1
```

rqt mouse 입력 토픽 변경:

```bash
ros2 run central rqt_click_to_target --ros-args \
  -p mouse_topic:=/central/yolo/image_annotated_mouse_left \
  -p target_pixel_topic:=/central/target_pixel
```

## Important Notes

차량 Nav2에서 사용하는 map과 calibration에 사용한 map은 같아야 합니다.

확인해야 할 항목:

```text
resolution
origin
PGM image size
map frame id
```

현재 calibration YAML은 아래 파일을 기준으로 합니다.

```text
config/SLAM/current_map.yaml
config/SLAM/current_map.pgm
```

차량이 다른 map을 쓰면 변환 좌표가 맞지 않습니다.

## Troubleshooting

### `Calibration yaml not found`

`config/central/camera_map_calibration.yaml`이 있는지 확인합니다.

```bash
ls config/central/camera_map_calibration.yaml
```

없으면 먼저 calibration 패키지에서 생성해야 합니다.

```bash
ros2 run calibration direct_calibrator
```

### 출력이 안 나옴

입력 픽셀 토픽이 들어오는지 확인합니다.

```bash
ros2 topic echo /central/target_pixel
```

출력 토픽 목록 확인:

```bash
ros2 topic list | grep central
```

# AMR1/AMR2 LiDAR adaptive recovery

`fleet_dispatcher` applies the fallback to both vehicles after Nav2 exhausts
its normal recovery tree. It checks a fresh vehicle LiDAR scan and reverses when
the rear sector has at least 5 cm clearance, stops at a 1 cm rear safety
margin or after 5 cm, and then compares the LiDAR clearance swept by left and
right turns. Instead of a fixed 45-degree turn, it finds each side's widest
continuous free angular gap and rotates toward the midpoint of that gap. It
then clears the local costmap and retries the original goal. If that route
fails, it rotates toward the midpoint of the opposite-side gap, clears the
local costmap again, and retries once more. Commands use the vehicle's
`cmd_vel_manual`. All
manual recovery velocity still passes through the vehicle emergency and fleet
collision safety gate.
