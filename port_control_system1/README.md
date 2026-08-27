# Port Control System

CustomTkinter 기반 중앙관제 대시보드입니다. 기존 파일 기반 화물·위치 관리 기능을
유지하면서 ROS 2 중앙관제 토픽과 연결합니다.

## 연결 기능

| 방향 | 인터페이스 | 기능 |
| --- | --- | --- |
| ROS → UI | `/battery/percent` (`std_msgs/Float32`) | 차량 1 배터리 표시 |
| ROS → UI | `/battery/voltage` (`std_msgs/Float32`) | 차량 전압 수신 |
| ROS → UI | `/odom` (`nav_msgs/Odometry`) | 차량 1 odom 위치·방향 표시 |
| ROS → UI | `/arm/pick_place/status` | 1번 로봇팔 상태 로그 수신 |
| ROS → UI | `/arm/pick_place/work_state` | ARM1 고정 작업 상태 수신 |
| ROS → UI | `/arm2/container_pick/status` | 2번 로봇팔 상태 수신 |
| ROS → UI | `/central/arms/arm1/state` | ARM1 연결·작업·오류 상태 표시 |
| ROS → UI | `/central/arms/arm2/state` | ARM2 연결·작업·진행률 표시 |
| UI → ROS | `/central/target_map_waypoints` (`nav_msgs/Path`) | 위치·경유지 주행 명령 |
| UI → ROS | `/central/target_map_pose` (`geometry_msgs/PoseStamped`) | 단일 목표용 인터페이스 |
| UI → ROS | `/cmd_vel` (`geometry_msgs/Twist`) | 수동 운전과 비상 정지 |
| UI → 중앙 | `/api/v1/arms/commands` (`arm1/pick_place`) | ARM1 Pick/Place 시작 |
| UI → ROS | `/arm2/pick_container`, `/arm2/stack_container` | 2번 로봇팔 파지·적재 |

비상 정지가 활성화되면 `/cmd_vel`에 정지 명령을 100Hz로 계속 발행합니다.
관제 화면에서 명시적으로 **비상 정지 해제**를 누르기 전에는 수동 속도 명령도
차단됩니다.

## 실행

차량과 관제 노트북의 네트워크, `ROS_DOMAIN_ID`, DDS 설정을 먼저 맞춥니다.

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=<차량과_같은_번호>
```

최초 한 번 대시보드 Python 의존성을 설치합니다.

```bash
cd ~/poter_ws
python3 -m venv .venv
.venv/bin/pip install -r port_control_system1/requirements.txt
```

차량에서 Nav2가 실행 중인 상태에서 관제 노트북의 waypoint 브리지를 실행합니다.

```bash
ros2 launch drive target_waypoints_nav.launch.xml \
  path_topic:=/central/target_map_waypoints
```

이 launch는 Nav2를 시작하지 않습니다. 차량 또는 관제 노트북에서 Nav2가 먼저
실행되어 `navigate_through_poses` 액션 서버가 준비되어 있어야 합니다.

영상 API가 필요하면 별도 터미널에서 실행합니다.

```bash
ros2 launch dashboard dashboard_stream.launch.py
```

대시보드를 실행합니다.

```bash
cd ~/poter_ws/port_control_system1
../.venv/bin/python agv_control_center.py
```

상단에 `ROS 연결됨`이 표시되는지 확인합니다. 통합 관제 화면의 차량 탭에서
배터리와 odom 값이 갱신되어야 합니다.

대시보드의 `로봇팔 (ARM)` 카드에는 ARM1·ARM2의 연결 상태, 현재 작업, 작업 단계,
진행률, 명령·미션 ID와 최근 오류가 표시됩니다. 중앙 ARM dispatcher의 구조화된
상태를 우선 사용하고, 아직 중앙 상태가 없으면 기존 `/arm*/container_pick/status`
문자열을 임시 상태로 표시합니다.

비상상황 대처 화면에서 `JetCobot #01`은 `/arm`, `JetCobot #02`는 `/arm2`
서비스를 사용합니다. 각 로봇팔의 MoveIt 파지 launch가 먼저 실행되어 해당 서비스가
준비되어 있어야 합니다.

## 위치 명령

`location_marks_verified.json`에서 `map_meters`가 등록된 위치만 차량 목표로 보낼 수
있습니다. 대시보드의 **명령** 창에서 다음처럼 입력합니다.

```text
항구로 이동
창고 회차지점을 거쳐 창고 하역장으로 이동
```

명령은 `/central/target_map_waypoints`에 `map` 좌표계의 `nav_msgs/Path`로
발행됩니다. 현재 등록 위치에는 별도 헤딩값이 없으므로 각 waypoint의 기본 yaw는
0도입니다.

## VLM 영상 좌표 주행

명령 창은 최신 탑다운 프레임을 Ollama 비전 모델에 함께 전달할 수 있습니다. 모델이
`pixel_navigation` 액션의 목표 픽셀과 헤딩 픽셀을 반환하면 대시보드가 중앙제어
API로 전송합니다.

검출 객체를 지칭한 명령은 `visual_navigation`을 사용합니다. VLM에는 YOLO
어노테이션 JPEG와 클래스, confidence, bbox, 중심점, heading JSON이 함께 전달됩니다.
VLM은 `detection_index`와 `approach_side`만 선택하고, 실제 목표점은 코드가 선택된
bbox 바깥 50픽셀에서 계산합니다. 직접 픽셀 목표는 영상 범위를 벗어나거나 YOLO
bbox 안에 있으면 전송하지 않습니다.

`B-1`과 `A-1`/`A-2`/`A-3`은 예외적으로 특수 구역으로 처리합니다. 차량/트레일러
클래스에는 이 규칙을 적용하지 않습니다.

- `B-1`(항구 상차·하차 전용 주차 구역): bbox 중심을 목표로 하고 세그멘테이션
  `heading_deg`를 주차 방향으로 사용합니다. 중심에 다른 차량의 검출 bbox가
  겹치면 점유된 구역으로 판단해 명령을 거부합니다. 중앙 ROS 브리지는 B-1
  중심을 map 좌표로 변환한 후 탑다운 카메라 영상의 왼쪽 방향으로 `0.15m`,
  화면 아래쪽으로 `0.03m` 이동한 지점을 최종 주차 위치로 사용합니다.
- `A-1`/`A-2`/`A-3`(화물 적재 대기 구역): 셋 다 동일한 화물 적재 위치를
  공유하므로, 검출된 bbox 위치와 무관하게 항상 같은 고정 map pose로
  이동합니다. 기본 정차점은 카메라 픽셀 `(157, 262)`를 현재 homography로
  변환한 map 좌표 `(0.16812885, 0.06234431)`에서 yaw `90°`의 반대 방향으로
  `0.10m` 물러난 `(0.16812885, -0.03765569)`입니다.
  `camera_to_map_bridge`의 `a_zone_map_x`/`a_zone_map_y`/
  `a_zone_map_yaw_deg`/`a_zone_stop_back_offset_m` 파라미터로 변경할 수
  있습니다. 화면상 목표/헤딩 픽셀은 안전 검증용일 뿐 실제 정지 위치에는
  영향을 주지 않습니다.

`B-1`은 여전히 항구 상하차 구역을 가리키는 말이고("항구", "항만", "부두",
"상차", "하차" 등도 전부 B-1을 뜻함), **"주차해줘"는 이제 별도의 실제 후진
주차 명령**입니다. 목적지를 영상 속 구역으로 특정하지 않고 그냥 "주차해",
"주차시켜"라고만 말하면 `park_command` 액션이 나가고, 대시보드는 중앙제어
API의 `/api/v1/navigation/park`로 요청을 보냅니다. 차량은 각자 전용 스팟(AMR1
`park_red`, AMR2 `parking_yellow`)으로 실제 후진 주차합니다. 명령 없이도 차량이
일정 시간(기본 20초) 이상 가만히 있으면 `fleet_dispatcher`가 자동으로 같은
주차를 시작합니다 (`fleet_dispatcher`의 `auto_park_idle_sec` 참고).

두 구역 모두 한 번에 한 대만 점유할 수 있어 다른 차량은 자동으로 대기합니다.

하나의 포괄적인 사용자 요청에 여러 이동이 필요하면 VLM은 `execution_mode`와
`actions` 배열을 함께 반환합니다. 서로 독립적인 여러 차량 작업은 사용자가
"동시에"라고 말하지 않아도 기본 `parallel`로 전송합니다. 같은 차량의 연속 목표,
"먼저/그 다음/도착 후"가 포함된 작업, 앞 단계 결과가 필요한 작업만
`sequential`로 처리하고 `predecessor_command_id`를 연결합니다. 별도의 새 사용자
명령은 진행 중인 해당 차량 목표를 새 목표로 교체합니다.

`모든 차량`, `두 대 모두` 같은 요청에서 모델이 이동 action 하나만 반환해도
대시보드가 이를 AMR1(내부 ID `agv1`)과 AMR2(내부 ID `agv2`) action으로 확장해 병렬 전송합니다. 두 차량이
같은 독점 구역을 요청하면 동시 명령이어도 Fleet 구역 잠금에 따라 한 대만 진입하고
다른 차량은 대기합니다.

현재 화면의 두 구역 사이에서 화물을 옮기라는 명령은 저장된
`cargo_locations.json`을 갱신하는 재고 명령과 분리됩니다. 예를 들어
`A-3 구역의 컨테이너를 B-1로 옮겨`는 `visual_transfer`로 해석하며, 현재 YOLO
프레임에서 A-3과 B-1을 다시 찾고 현재 Fleet 상태가 정상인 차량 중 A-3에 가장
가까이 보이는 차량을 선택합니다. 선택된 같은 차량에 A-3 도착 명령과 B-1 후속
명령을 순차 전송하며 과거 화물 목록은 변경하지 않습니다. 이 단계는 AMR 이동
계획이며 로봇팔 상차·하차 서비스 호출은 별도 작업입니다.

`amr1을 B-1로 보내고 amr2를 A구역으로 보내`처럼 두 차량이 서로의 목적 구역을
이미 점유한 경우에는 일반 순차 명령으로 연결하지 않습니다. 두 차량을 각 목표
구역의 대기점으로 독립 이동시키고, 대기점에 도착해 기존 구역에서 물리적으로
이탈하면 기존 구역 잠금을 해제합니다. 상대 차량이 실제 정차점에 들어가 구역을
비우면 대기 차량도 자동으로 최종 정차점까지 이어서 이동합니다.

구역 번호를 정확히 말하지 않아도 현재 YOLO 결과에 대응 구역이 실제로 보이면 다음
표현을 이동 명령으로 처리합니다.

```text
노란 차 상차하러 보내줘        # B-1, AMR1
항구에 차량 한 대 배차해       # B-1, 자동 배차
차량 한 대를 A구역에 대기시켜  # 보이는 A-1/A-2/A-3, 자동 배차
주차해                          # B-1이 보일 때 B-1
```

모델이 `unknown`을 반환하거나 `detection_index`/`approach_side`를 빠뜨려도, 명령의
목적지 의미와 현재 YOLO 검출이 일치하면 대시보드가 해당 필드를 복구합니다. 검출되지
않은 구역을 만들거나 임의 좌표를 추측하지 않으며, 이동 금지·정지 명령은 이 복구
대상에서 제외합니다.

기본값은 팀 공유 Ollama 서버(`http://agent.sds.codes`, 모델 `gemma4:31b`)이므로
별도 설정 없이 바로 사용할 수 있습니다. 로컬 Ollama를 쓰려면 `OLLAMA_HOST`를
`http://127.0.0.1:11434`로 덮어씁니다.

```bash
export PORT_CONTROL_API_URL=http://<중앙제어_IP>:8100
export PORT_CONTROL_API_TOKEN='porter1234'
# 로컬 Ollama를 쓸 때만:
# export OLLAMA_HOST=http://127.0.0.1:11434
# export LOCAL_LLM_MODEL=<설치된_모델명>
```

테스트 명령:

```text
현재 영상에서 중앙의 빈 공간으로 이동하고 오른쪽을 바라봐
```

VLM이 이미지를 받지 못했거나 안전한 목표를 특정하지 못하면 좌표를 추측하지 않고
명령을 거부하도록 프롬프트에 지정되어 있습니다. 최종 픽셀 범위는 중앙제어 API가
다시 검증하고 실제 장애물과 경로 가능 여부는 Nav2가 판단합니다.

## LLM 실시간 관제

명령 창에서 정상 실행된 최근 명령은 일회성 문장이 아니라 **지속 목표**로도
등록됩니다. 사이드바의 `LLM 실시간 관제` 스위치가 켜져 있으면 백그라운드
에이전트가 최신 탑다운 영상, YOLO JSON, 두 차량의 Fleet 상태와 구역 점유 상태를
함께 다시 판단합니다.

- 기본 확인 간격은 2초입니다.
- 객체 위치, 차량 상태, 현재 명령, 구역 점유가 의미 있게 변하면 즉시 다시
  판단합니다.
- 화면 변화가 없어도 기본 5초마다 최신 이미지와 상태를 다시 판단합니다.
- 사용자가 방금 보낸 action은 초기 이력으로 등록하고 5초 동안 재평가를 늦춰
  같은 명령이 즉시 중복 전송되는 것을 막습니다.
- `READY` 차량에 필요한 새 이동만 전송하며, 독립 작업은 두 차량에 병렬로
  배정합니다.
- A/B-1 대기열 진입과 점유 해제는 AMCL 기반 Fleet dispatcher가 처리하므로 LLM
  응답 지연과 무관하게 진행됩니다.

실시간 LLM은 작업 계획 계층입니다. 차량 간 충돌처럼 즉시 반응해야 하는 안전
판단은 기존 `fleet_collision_supervisor`가 계속 담당하며, Nav2와 비상 정지 게이트도
그대로 적용됩니다. 아직 사용자 지속 목표가 없을 때는 영상을 감시하더라도 차량
명령을 임의로 만들지 않습니다.

환경변수로 주기를 변경할 수 있습니다.

```bash
export PORT_CONTROL_REALTIME_LLM_ENABLED=true
export PORT_CONTROL_REALTIME_LLM_INTERVAL_SEC=2.0
export PORT_CONTROL_REALTIME_LLM_HEARTBEAT_SEC=5.0
export PORT_CONTROL_REALTIME_LLM_INITIAL_DELAY_SEC=5.0
```

## PostgreSQL DB 기반 컨테이너 이동 판단

명령 창의 **DB 기반 계획** 버튼은 PostgreSQL `port_db.cargos`를 직접 읽고,
목표를 완료하기 위한 컨테이너 이동 순서를 JSON으로 만듭니다.
이 버튼은 기존 **실행** 버튼과 분리되어 있으며 DB 쓰기, ROS 서비스 호출,
중앙제어 API 전송을 수행하지 않습니다. 생성된 목표는 실시간 에이전트의
`inventory` 모드에도 등록되어 `snapshot_id`가 바뀌거나 heartbeat가 도래하면 같은
읽기 전용 판단을 다시 수행합니다.

대시보드 우측 하단의 **자율 관제모드 시작/중지** 버튼으로 지속 판단 루프를 직접
켜고 끌 수 있습니다. 시작했지만 아직 목표가 등록되지 않았으면 버튼에 `목표 대기`,
LLM이 판단 중이면 `판단 중`, 원격 조회나 판단이 실패하면 `오류`가 표시됩니다.
사이드바의 기존 `LLM 실시간 관제` 스위치도 같은 상태로 자동 동기화됩니다.

```bash
pip install psycopg2-binary

# 현재 기본값과 동일하므로 접속 정보가 바뀔 때만 설정합니다.
export PORT_INVENTORY_DB_HOST=10.11.4.249
export PORT_INVENTORY_DB_PORT=5432
export PORT_INVENTORY_DB_NAME=port_db
export PORT_INVENTORY_DB_USER=postgres
export PORT_INVENTORY_DB_PASSWORD=1234
export PORT_INVENTORY_DB_TIMEOUT_SEC=3
```

같은 DB를 터미널에서 확인하는 명령은 다음과 같습니다.

```bash
psql -h 192.168.5.9 -U postgres -d port_db
```

클라이언트는 `name, location, container_id, cargo_type, note, base_aruco_id, floor`만
읽기 전용 트랜잭션으로 조회합니다. 조회 결과 내용으로 `sql-...` 스냅샷 ID를 만들기
때문에 실제 재고가 바뀐 경우에만 지속 관제가 즉시 재판단합니다. 연결·인증·스키마
오류가 발생하면 이전 데이터를 재사용하지 않고 `status=error`, `moves=[]`로
종료합니다. 정상 계획도 LLM 응답을 그대로 사용하지 않고 컨테이너 ID, 현재 출발
위치, 적층 순서와 목적지 기반 관계를 메모리에서 순서대로 검증합니다.

## DB 기반 완전 자율 관제

중앙관제와 대시보드를 실행하기 전에 동일한 DB 접속 환경을 설정합니다.

```bash
export PORT_INVENTORY_DB_HOST=192.168.5.9
export PORT_INVENTORY_DB_PORT=5432
export PORT_INVENTORY_DB_NAME=port_db
export PORT_INVENTORY_DB_USER=postgres
export PORT_INVENTORY_DB_PASSWORD=1234
```

중앙관제의 `inventory_sync` 노드만 PostgreSQL을 갱신합니다. ARM1/ARM2의 완료
결과는 `/central/inventory/movements` 표준 이벤트가 되고, 먼저 로컬 SQLite
보류 큐(`PORT_INVENTORY_OUTBOX_PATH`)에 저장된 뒤 `cargo_movements` 이력과
`cargos` 최신 위치에 트랜잭션으로 반영됩니다. `operation_id`가 같으면 한 번만
적용됩니다. DB가 끊기면 물리 작업은 안전하게 끝내지만 보류 건이 0이 될 때까지
새 LLM 계획은 차단됩니다.

대시보드의 **자율 관제모드 시작**은 실제 자동 실행 승인입니다. 시작 후에는
`WAITING_FOR_INBOUND → SCANNING_INBOUND → UNLOADING_INBOUND →
LOADING_OUTBOUND → WAITING_FOR_CLEAR` 정책으로 한 컨테이너씩 실행하고 DB 반영을
확인한 뒤 다시 계획합니다. **중지**하면 진행 중인 ARM 원자 작업만 끝나며 다음
단계는 전송하지 않습니다. 수동 자연어 명령과 자율 실행은 동시에 허용되지 않습니다.

ARM1은 중앙관제 연결 후 사용 가능한 빈 선박 슬롯 마커 19~23을 캐시합니다. 마커
18(선박-1)은 사용하지 않습니다. 실패하거나 ARM1이
재시작해 캐시가 사라지면 유휴 상태에서 10초 간격으로 다시 스캔합니다. ROI 입항
이벤트가 발생하면 노출된 컨테이너 마커 0~8을 선박 슬롯과 대응시켜 DB 동기화
이벤트로 보냅니다.

판단 결과는 화면 로그와 실시간 에이전트 상태에 다음 형태로만 노출됩니다.

```json
{
  "schema_version": "1.0",
  "plan_id": "plan-...",
  "snapshot_id": "sql-...",
  "objective": "C0를 B-1로 옮겨",
  "status": "ready",
  "moves": [
    {
      "sequence": 1,
      "container_id": "0",
      "container_name": "컨테이너_C0",
      "source_location": "A-1-1",
      "source_floor": 1,
      "destination_location": "B-1",
      "destination_floor": 1,
      "destination_base_aruco_id": "",
      "reason": "운영 목표의 출고 위치로 이동"
    }
  ],
  "summary": "C0 출고 계획",
  "error": ""
}
```

## 영상 설정

`stream_config.json`에서 기존 dashboard API 주소를 지정합니다.

```json
{
  "cctv_url": "http://127.0.0.1:8000/video",
  "slam_url": "http://127.0.0.1:8000/slam/video"
}
```

API 서버가 다른 노트북에서 실행 중이면 `127.0.0.1`을 해당 노트북 IP로
변경합니다.

## 토픽 확인

```bash
ros2 topic hz /battery/percent
ros2 topic hz /odom
ros2 topic echo /central/target_map_waypoints
ros2 topic echo /cmd_vel
ros2 topic info /central/target_map_waypoints -v
```

## 환경변수

필요한 경우 앱 실행 전에 인터페이스 토픽을 변경할 수 있습니다.

```bash
export PORT_CONTROL_CMD_VEL_TOPIC=/cmd_vel
export PORT_CONTROL_TARGET_POSE_TOPIC=/central/target_map_pose
export PORT_CONTROL_TARGET_WAYPOINTS_TOPIC=/central/target_map_waypoints
```

현재 ROS 브리지는 앱을 실행한 `ROS_DOMAIN_ID` 한 개에 연결됩니다. 서로 다른
ROS 도메인의 차량 두 대를 동시에 제어하려면 도메인별 게이트웨이 프로세스를 두고
차량 토픽을 중앙 도메인에 namespace로 중계해야 합니다.
