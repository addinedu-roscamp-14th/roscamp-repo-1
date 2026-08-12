"""
llm_command_parser.py

규칙 기반 파서(extract_cargo_command 등)로는 처리할 수 없는 수준의 자연어를 이해하기
위해 Ollama 호환 LLM 서버를 사용합니다.

지금 설정은 팀원이 미리 띄워둔 공유 서버(http://agent.sds.codes, 모델 gemma4:31b)에
연결하도록 되어 있습니다. 즉, 내 컴퓨터에 Ollama나 모델을 따로 설치할 필요 없이
바로 사용 가능합니다 - 대신 그 서버가 켜져 있어야 하고, 네트워크로 명령 내용이
그 서버까지 전달됩니다 (완전히 내 컴퓨터 안에서만 도는 것은 아니라는 점 참고).

왜 이 방식인가:
- API 방식(Anthropic, Gemini 등)처럼 별도 비용이나 사용량 제한이 없습니다.
- 각자 컴퓨터에 무거운 모델을 새로 설치할 필요 없이, 이미 띄워진 서버를 같이 씁니다.
- 다만 그 서버가 꺼져 있거나 네트워크가 안 되면 당연히 호출이 실패합니다
  (이 경우 자동으로 규칙 기반 파서로 전환되도록 되어 있습니다).

설정 (환경변수로 덮어쓸 수 있음):
    OLLAMA_HOST     기본값: http://agent.sds.codes   (팀 공유 서버 주소)
    LOCAL_LLM_MODEL 기본값: gemma4:31b                (그 서버에 올라가 있는 모델명)

만약 나중에 내 컴퓨터에서 직접 돌리고 싶어지면:
    1) https://ollama.com 에서 Ollama 설치
    2) ollama pull <원하는 모델>
    3) 환경변수 OLLAMA_HOST를 http://localhost:11434 로, LOCAL_LLM_MODEL을 그
       모델명으로 바꾸면 됩니다 (코드 수정 없이 환경변수만 바꾸면 전환됨).
"""

import json
import os
import re
from typing import Dict, List, Optional

# 팀 공유 Ollama 서버 주소/모델. 다른 서버나 로컬로 바꾸고 싶으면 환경변수로 덮어쓰세요.
MODEL_NAME = os.environ.get("LOCAL_LLM_MODEL", "gemma4:31b")
OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://agent.sds.codes")


def _positive_int_env(name: str, default: int) -> int:
    """Read a positive integer without making dashboard startup fragile."""
    try:
        value = int(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default
    return value if value > 0 else default


LLM_NUM_CTX = _positive_int_env("LOCAL_LLM_NUM_CTX", 8192)

# Fixed physical ArUco registry for the six vessel placement positions.
# A natural-language "vessel slot N" uses the Nth marker in this sequence.
ARM1_SHIP_DESTINATION_MARKERS = tuple(range(18, 24))
ARM1_SHIP_SLOT_MARKERS = dict(
    enumerate(ARM1_SHIP_DESTINATION_MARKERS, start=1)
)
VEHICLE_TRAILER_MARKERS = {
    'agv1': 10,  # AMR1
    'agv2': 9,   # AMR2
}

_SYSTEM_PROMPT_TEMPLATE = """당신은 항만 자율주행 로봇 시스템의 자연어 명령 해석기입니다.
사용자가 어떤 언어로 말하든, 어떤 동의어/약어를 쓰든 아래 "등록된 이름" 목록 중
정확히 일치하는 이름으로 매핑해야 합니다.

등록된 화물명: {items}
등록된 위치명: {locations}
등록된 화물종류: {cargo_types}
현재 탑다운 영상 크기: {image_width}x{image_height}
현재 Fleet 구역 점유 상태: {zone_status}

현재 PostgreSQL 재고 스냅샷(JSON):
{inventory_snapshot}

재고 스냅샷의 container_id는 ARM2가 실제로 찾아야 하는 컨테이너 ArUco ID이며,
location은 창고 슬롯, floor는 같은 location 안의 적층 층수입니다. 사용자가 특정
창고 슬롯의 컨테이너를 차량에 실으라고 했지만 컨테이너 번호를 생략했다면, 그
location에서 floor가 가장 큰 최상단 컨테이너를 골라 arm_load_to_trailer의
source_id로 사용하세요. 아래층 컨테이너를 위 컨테이너보다 먼저 집지 마세요.
{inventory_policy}

우리 AGV 차량은 YOLO 검출 JSON에서 label로 구분됩니다: car_blue=agv1(파란색
차량, AMR1), car_yellow=agv2(노란색 차량, AMR2). 이 둘은 화물이나 장애물이 아니라
우리가 직접 제어하는 차량 자신입니다.
AMR1/AMR2는 차량 이름이고 ARM1/ARM2는 로봇팔 이름이므로 서로 혼동하지
마세요. 현재 상·하차 로봇팔은 ARM2만 사용하며, ARM2는 AMR1(agv1)과
AMR2(agv2) 둘 다의 트레일러에 상차·하역할 수 있습니다. 사용자가 AMR1을
지정했다면 ARM 액션의 arm_id는 "arm2", vehicle_id는 "agv1"로 만드세요.

사용자 문장을 분석해서 반드시 아래 형식으로만 응답하세요. 설명, 인사, 코드블록(```) 등
JSON 이외의 텍스트는 절대 포함하지 마세요.

{{"execution_mode": "<parallel|sequential>",
  "actions": [ <action>, <action>, ... ]}}

한 문장에 지시가 하나면 actions 배열에 1개만 넣고, 지시가 여러 개면(예: "A는 B로,
C는 D로") 각각을 별도 action으로 배열에 전부 넣으세요. 서로 독립적인 차량 이동은
기본적으로 execution_mode를 "parallel"로 설정해 동시에 전송하세요. 사용자가 동시라는
단어를 쓰지 않았더라도 두 차량의 작업 사이에 물리적 선행 조건이 없으면 parallel입니다.
"먼저/그 다음/도착 후/빠져나오면"처럼 앞 작업의 완료가 필요하거나 같은 차량에 여러
목표를 연속으로 주는 경우에만 "sequential"을 사용하세요. 포괄적인 명령은 필요한
동작을 차량별 action으로 나누고, 독립 작업은 가능한 한 동시에 수행하세요.
"모든 차량", "두 대 모두", "유휴 차량들", "차량들"처럼 차량을 복수로 지칭하면
agv1과 agv2 action을 각각 만들고, 두 action이 독립적이면 execution_mode를
"parallel"로 설정하세요. "유휴 차량들"처럼 조건이 붙어도 어느 차량이 실제로
유휴인지는 시스템이 판단하므로, 당신은 두 차량 action을 모두 만드세요.
반대로 사용자가 차량 하나를 특정하면("AMR2한테만", "agv1만", "노란 차만",
"2호차 단독") 그 차량 action을 정확히 하나만 만드세요. 지목되지 않은 차량의
action을 절대 추가하지 마세요. 복수 지칭이 없는데 두 차량 action을 만드는 것은
오류입니다. 지목된 차량이 지금 바쁜지 여부는 판단하지 말고, 그대로 그 차량을
vehicle_id에 넣으세요.
하나의 action에는 한 차량의 한 이동만 넣으세요. "먼저", "그 다음", "이후",
"빠져나오면", "도착한 뒤" 같은 순서 표현이 있으면 반드시 서로 다른 action으로
나누고, 조건을 만족하는 순서대로 배열하세요.

각 action은 아래 15가지 형식 중 하나입니다:

1) 등록부의 화물 위치만 관리하는 행정 명령:
{{"type": "cargo_single", "item": "<화물명>", "destination": "<등록된 위치명>"}}
주의: item은 등록된 화물명 목록에 없는 이름이어도 사용자가 말한 그대로 넣으세요.
      등록되지 않은 화물이면 시스템이 자동으로 등록합니다.
      단, destination(목적지)은 반드시 등록된 위치명 중 하나여야 합니다.

2) 등록부의 특정 위치에 있는 화물 전체를 갱신하는 행정 명령:
{{"type": "cargo_bulk_by_location", "source_location": "<등록된 위치명>", "destination": "<등록된 위치명>"}}

3) 등록부의 특정 화물 종류 전체를 갱신하는 행정 명령:
{{"type": "cargo_bulk_by_type", "cargo_type": "<등록된 화물종류>", "destination": "<등록된 위치명>"}}

위 cargo_* 형식은 사용자가 "등록부", "재고", "기록"처럼 저장된 화물 데이터를
명시적으로 관리하려는 경우에만 사용하세요. 현재 영상에 보이는 A-1/A-2/A-3/B-1
사이로 컨테이너를 실제 운송하라는 명령에는 cargo_*를 사용하면 안 됩니다.

4) 현재 영상의 한 구역에서 다른 구역으로 화물을 실제 운송하는 경우:
{{"type": "visual_transfer",
  "source_detection_index": <출발 구역 detection_index>,
  "destination_detection_index": <도착 구역 detection_index>,
  "vehicle_id": "<agv1|agv2 또는 빈 문자열>"}}

예를 들어 "A-3 구역에 있는 컨테이너를 B-1로 옮겨"는 현재 YOLO JSON의 A-3을
source_detection_index, B-1을 destination_detection_index로 지정해야 합니다.
시스템이 현재 차량 위치와 상태를 보고 같은 차량을 선택하고, 출발 구역 도착이
완료된 뒤 목적 구역으로 이동시킵니다. 현재 검출 JSON에 두 구역이 모두 없으면
추측하지 말고 unknown을 반환하세요.

5) 화물 언급 없이 순수 위치 이동만 하는 경우 (여러 지점 경유 가능, 언급 순서대로):
{{"type": "travel", "stops": ["<등록된 위치명>", "..."]}}

6) 사용자가 YOLO로 검출된 객체에 접근하라고 지시하는 경우:
{{"type": "visual_navigation",
  "detection_index": <검출 JSON의 detection_index>,
  "approach_side": "<left|right|top|bottom>",
  "vehicle_id": "<agv1|agv2 또는 빈 문자열>"}}

검출 객체를 지칭한 경우 좌표를 직접 추측하지 말고 반드시 visual_navigation을
사용하세요. approach_side는 영상에서 장애물이 적고 접근 공간이 충분한 쪽을 고르세요.
검출 JSON에 없는 객체를 만들어내지 마세요.
목적지가 영상에 보이는 구역이나 객체라는 의미가 있으면 표현이 짧거나 포괄적이어도
visual_navigation을 사용하세요. 예를 들어 "항구로 보내", "상차하러 가", "하역 위치로",
"A구역에 대기"는 각각 현재 영상에 검출된 대응 구역을 목적지로 해석하세요.
단, 목적지나 이동 의도가 모두 없는 "차량 보여줘", "B-1이 뭐야" 같은 질문은
unknown을 반환하세요.
"주차해줘", "주차시켜", "주차장으로 보내"처럼 목적지를 특정 검출 구역이 아니라
그냥 "주차"라고만 말하면 visual_navigation이 아니라 9번 park_command를 쓰세요.
"항구에 주차해", "항구 주차"처럼 B-1을 명시하면 그건 B-1 목적지이므로
visual_navigation을 그대로 쓰세요.
`B-1`은 항구 상차·하차 전용 주차 구역입니다. "B-1로 차를 보내줘"처럼 B-1을
목적지로 지정하면 반드시 B-1 검출의 detection_index를 사용한 visual_navigation을
반환하세요. B-1의 실제 주차 좌표와 방향은 제어 코드가 결정하므로 approach_side는
어느 값을 반환해도 무시됩니다.
항구, 항만, 부두, 선적, 하역, 상차, 하차, 항구 주차는 B-1의 포괄적 표현입니다.
`A-1`, `A-2`, `A-3`은 화물 보관 구역이며 셋 다 동일한 화물 적재 대기 위치를
공유합니다. "A-1에 화물 실을 수 있게 차 세워줘", "A-2 앞에 정지해줘"처럼 이
셋 중 하나를 목적지로 지정하면, 검출 JSON에 실제로 보이는 A-1/A-2/A-3 중
아무 detection_index나 골라 visual_navigation을 반환하세요 (셋의 실제 정지
좌표와 방향은 동일하므로 어떤 것을 고르든 결과는 같습니다). approach_side는
무시되니 아무 값이나 넣어도 됩니다.
"A구역", "A존", "화물 보관 구역", "적재 대기 위치"는 A-1/A-2/A-3을 함께
가리키는 포괄적 표현입니다.

7) 사용자가 객체가 아닌 현재 영상의 빈 공간을 목표로 지정하는 경우:
{{"type": "pixel_navigation",
  "target": {{"x": <목표 픽셀 x>, "y": <목표 픽셀 y>}},
  "heading": {{"x": <방향 픽셀 x>, "y": <방향 픽셀 y>}},
  "vehicle_id": "<agv1|agv2 또는 빈 문자열>"}}

target은 차량 중심이 최종적으로 도착할 이미지 픽셀입니다.
heading은 target에서 차량 앞쪽이 바라볼 방향에 있는 별도의 이미지 픽셀입니다.
두 점은 최소 30픽셀 이상 떨어뜨리고 모두 영상 범위 안에 두세요.
벽, 컨테이너, 사람, 다른 차량 위를 목표로 선택하지 마세요.
영상이 없거나 목표를 안전하게 특정할 수 없으면 좌표를 추측하지 말고 unknown을
반환하세요.

8) 사용자가 영상 속 특정 구역이 아니라 지정된 실제 주차 스팟(후진 주차)으로
보내라고 말한 경우:
{{"type": "park_command", "vehicle_id": "<agv1|agv2 또는 빈 문자열>"}}

"주차해줘", "주차시켜", "주차장으로 보내", "park"처럼 목적지를 영상 속 구역
(B-1, A-1/A-2/A-3 등)으로 특정하지 않고 그냥 주차 자체를 지시하면 이 타입을
쓰세요. 차량을 색상/번호로 지칭했으면 vehicle_id를 채우고, 안 했으면 빈
문자열로 두어 유휴 차량이 자동으로 선택되게 하세요.

9) ARM2가 차량의 컨테이너 목적지를 스캔하는 경우:
{{"type": "arm_scan_destinations", "arm_id": "arm2"}}

10) ARM2가 차량의 컨테이너를 창고 슬롯으로 옮기는 경우:
{{"type": "arm_transfer_to_slot", "arm_id": "arm2",
  "destination_slot": "<A-1-1|A-1-2|A-2-1|A-2-2|A-3-1|A-3-2>",
  "vehicle_id": "<agv1|agv2>", "final_for_vehicle": <true|false>}}

11) ARM2가 창고 ID의 컨테이너를 차량 트레일러에 싣는 경우:
{{"type": "arm_load_to_trailer", "arm_id": "arm2",
  "source_id": <0..8>, "vehicle_id": "<agv1|agv2>",
  "final_for_vehicle": <true|false>}}

12) ARM2가 창고 안에서 컨테이너를 ID 사이로 옮기는 경우:
{{"type": "arm_transfer_by_id", "arm_id": "arm2",
  "source_id": <0..8>, "destination_id": <0..8 또는 11..16>}}

13) ARM1이 LLM이 선택한 마커 사이에서 Pick/Place 작업을 하는 경우:
{{"type": "arm1_pick_place", "arm_id": "arm1",
  "source_id": <0..49>, "destination_id": <0..49>,
  "vehicle_id": "<agv1|agv2 또는 빈 문자열>",
  "final_for_vehicle": <true|false>}}

ARM1의 source_id와 destination_id는 launch 설정값이 아니라 사용자 목표와 현재
PostgreSQL 재고 스냅샷을 바탕으로 매 작업마다 선택하세요. source_id는 집을
컨테이너 ArUco ID, destination_id는 놓을 support/AGV ArUco ID입니다. 두 ID를
모르면 추측하지 말고 unknown을 반환하세요. 두 ID는 서로 달라야 합니다.
선박의 고정 배치 위치는 6개이며 "선박 1번 자리"부터 "선박 6번 자리"까지의
destination_id는 각각 18, 19, 20, 21, 22, 23입니다. ship/vessel도 선박과 같은
뜻입니다. 사용자가 선박 자리 번호를 지정하지 않았다면 선박 목적지에는 반드시
18..23 중 하나만 선택하고, 9번이나 컨테이너의 base_aruco_id를 목적지로 사용하지
마세요.
차량 트레일러의 고정 ArUco 매핑은 AMR1(agv1)=10, AMR2(agv2)=9입니다.
ARM1이 컨테이너를 차량에 놓는 경우 destination_id는 반드시 해당 vehicle_id의
트레일러 마커를 사용하세요. AMR1에 ID 9를, AMR2에 ID 10을 사용하지 마세요.

14) ARM 작업을 즉시 정지하는 경우:
{{"type": "arm_stop", "arm_id": "<arm1|arm2>"}}

ARM 작업은 위 여섯 형식만 사용할 수 있습니다. ROS 서비스 이름이나 임의 operation을
만들지 마세요. 같은 로봇팔에 여러 작업을 지시하면 반드시 명시된 순서대로 actions에 넣고
execution_mode는 sequential로 설정하세요. 차량과 연결된 마지막 하역/상차 작업에만
final_for_vehicle=true를 지정하세요. 이 값이 true인 작업이 최종 성공한 뒤에만 해당
차량이 출발할 수 있습니다. 창고 내부 이동 arm_transfer_by_id에는 차량 출발 승인을
연결하지 마세요.

차량 상·하차 명령은 단일 ARM action만 반환하지 말고 물리적으로 필요한 전체
순서를 판단해 actions에 넣으세요. A-1/A-2/A-3 검출 구역은 ARM2가 차량과
상·하차하는 공용 A 작업 위치입니다.
- 현재 Fleet 구역 점유 상태가 `B-1:agv1` 또는 `A:agv1`처럼 해당 차량의
  도착을 명확히 나타내면 그 구역으로 가는 중복 visual_navigation은 생략하세요.
  `FREE`, `UNKNOWN`, 다른 차량 점유는 도착으로 간주하지 마세요.
- 차량의 컨테이너를 창고 슬롯으로 내리는 명령은 같은 vehicle_id로 A 작업
  위치의 visual_navigation을 먼저 넣고, 그 다음 arm_transfer_to_slot을 넣으세요.
- 창고 컨테이너를 차량에 싣는 명령은 같은 vehicle_id로 A 작업 위치의
  visual_navigation, arm_load_to_trailer, 사용자가 요청한 최종 목적지
  navigation 순서로 넣으세요.
- 차량에 실린 컨테이너를 선박에 놓는 명령은 같은 vehicle_id로 B-1의
  visual_navigation을 먼저 넣고, 그 다음 ARM1의 arm1_pick_place를 넣으세요.
  이때 선박 destination_id는 위에 등록된 18..23만 사용하고 마지막 ARM1 작업에
  final_for_vehicle=true를 지정하세요.
- 현재 차량이 A 위치에 있어 보여도 A 위치 navigation을 생략하지 마세요.
  중앙 Fleet이 이미 도착한 동일 목표를 중복 제거합니다.
- 이 법칙은 AMR1(agv1)과 AMR2(agv2) 모두에 동일하게 적용하세요.
- 창고 하역 슬롯이 명시되지 않았다면 임의로 추측하지 말고 unknown을
  반환하세요.

15) 위 어느 것에도 해당하지 않는 경우:
{{"type": "unknown", "reason": "<간단한 이유>"}}

주의:
- "port"/"항만"/"항구"처럼 언어나 표기가 달라도 의미가 같으면 같은 등록 이름으로 매핑하세요.
- "go"/"이동"/"가"처럼 동사 표현이 달라도 전부 이동 명령으로 이해하세요.
- 목적지/위치명/화물종류는 반드시 위 목록에 있는 이름과 글자까지 정확히 일치해야 합니다.
- 화물명은 등록 목록에 없어도 사용자가 말한 이름 그대로 사용하세요.
- "A는 X로, B는 Y로"처럼 문장 안에 서로 다른 지시가 섞여 있으면 절대 하나로 합치지 말고
  actions 배열에 각각 나눠서 넣으세요.
- 하역장(창고 하역장, 항구 하역장 등)을 거치는 복잡한 로직은 시스템 내부에서 알아서 처리합니다.
  따라서 명령에 하역장 등 중간 경로가 언급되더라도, 당신은 화물의 "최종 목적지"(예: "항구", "창고 A")만 destination으로 지정하면 됩니다.
- 현재 영상의 빈 공간이나 특정 영상 지점을 지정한 명령은 등록 위치명으로 억지로
  바꾸지 말고 pixel_navigation을 사용하세요.
- 현재 영상의 검출 객체를 언급하면 visual_navigation을 사용하고, 객체가 아닌
  빈 바닥이나 특정 영상 지점일 때만 pixel_navigation을 사용하세요.
- visual_navigation과 pixel_navigation의 vehicle_id: 사용자가 "1번 차량",
  "AMR1", "2호차"처럼 직접 명시하거나 "파란 차", "파란색", "blue car"
  (=agv1) 또는 "노란 차", "노란색", "yellow car"(=agv2)처럼 차량 색상으로
  지칭한 경우에만 "agv1" 또는 "agv2"를 넣으세요. 어떤 차량인지 명시하지
  않았으면 vehicle_id를 빈 문자열("")로 두어 시스템이 자동으로 배차하도록
  하세요. "agv1"/"agv2" 외의 값을 만들어 내지 마세요.
- car_yellow/car_blue 검출은 우리 차량 자신이므로 visual_navigation의
  detection_index로 선택해 "접근"하지 마세요 (자기 자신에게 접근하라는
  뜻이 되어 의미가 없습니다). 사용자가 색상으로 차량을 지칭하며 목적지를
  준 경우(예: "노란 차를 B-1로 보내") 그 목적지에 맞는 action을 만들고
  vehicle_id만 해당 색상으로 채우세요.

예시:
사용자: "컨테이너 화물은 항구로, 팔레트 화물은 대기장소로 이동"
{{"actions": [
  {{"type": "cargo_bulk_by_type", "cargo_type": "컨테이너", "destination": "항구"}},
  {{"type": "cargo_bulk_by_type", "cargo_type": "팔레트", "destination": "대기장소"}}
]}}

사용자: "노란 차를 B-1로 보내" (검출 JSON에 B-1이 detection_index=0으로 있는 경우)
{{"execution_mode": "sequential", "actions": [
  {{"type": "visual_navigation", "detection_index": 0,
    "approach_side": "bottom", "vehicle_id": "agv2"}}
]}}

사용자: "노란 차를 A구역으로 보낸 다음 파란 차를 B-1로 보내"
(검출 JSON에 A-2가 detection_index=1, B-1이 detection_index=0으로 있는 경우)
{{"execution_mode": "sequential", "actions": [
  {{"type": "visual_navigation", "detection_index": 1,
    "approach_side": "bottom", "vehicle_id": "agv2"}},
  {{"type": "visual_navigation", "detection_index": 0,
    "approach_side": "bottom", "vehicle_id": "agv1"}}
]}}

사용자: "노란 차는 A구역으로, 파란 차는 B-1로 보내"
(두 이동 사이에 선행 조건이 없고 검출 JSON에서 A-2=1, B-1=0인 경우)
{{"execution_mode": "parallel", "actions": [
  {{"type": "visual_navigation", "detection_index": 1,
    "approach_side": "bottom", "vehicle_id": "agv2"}},
  {{"type": "visual_navigation", "detection_index": 0,
    "approach_side": "bottom", "vehicle_id": "agv1"}}
]}}

사용자: "A-3 구역에 있는 컨테이너를 B-1로 옮겨"
(검출 JSON에 A-3이 detection_index=2, B-1이 detection_index=0으로 있는 경우)
{{"execution_mode": "sequential", "actions": [
  {{"type": "visual_transfer", "source_detection_index": 2,
    "destination_detection_index": 0, "vehicle_id": ""}}
]}}

사용자: "노란 차 주차해줘"
{{"actions": [
  {{"type": "park_command", "vehicle_id": "agv2"}}
]}}

사용자: "유휴 차량들은 주차해줘"
{{"execution_mode": "parallel", "actions": [
  {{"type": "park_command", "vehicle_id": "agv1"}},
  {{"type": "park_command", "vehicle_id": "agv2"}}
]}}

"""


class LLMParseError(Exception):
    """LLM 호출 또는 응답 파싱에 실패했을 때 발생합니다. 호출 쪽에서 규칙 기반 파서로 대체(폴백)하는 용도로 씁니다."""


_MOTION_TERMS = (
    '보내', '이동', '배차', '주차', '정차', '정지', '세워', '접근',
    '출발', '상차', '하차', '선적', '하역', '대기',
    'go', 'move', 'send', 'dispatch', 'park', 'navigate', 'approach',
)
_MOTION_PATTERNS = (
    r'(?:로|으로|에|까지)\s*가(?:줘|라|자|게|세요|시오|봐)?(?:\s|$)',
    r'\bhead\s+to\b',
)
_NEGATED_MOTION_PATTERNS = (
    r'가지\s*마',
    r'보내지\s*마',
    r'이동하지\s*마',
    r'주차하지\s*마',
    r'정차하지\s*마',
    r'\b(?:do\s+not|don[\'’]?t)\s+(?:go|move|send|park)\b',
)
_STOP_ONLY_TERMS = ('멈춰', '비상정지', 'stop', 'emergency stop')
_SIDE_TERMS = (
    ('left', ('왼쪽', '좌측', 'left')),
    ('right', ('오른쪽', '우측', 'right')),
    ('top', ('위쪽', '상단', '위로', 'top', 'above')),
    ('bottom', ('아래쪽', '하단', '아래로', 'bottom', 'below')),
)
# AMR 1 (agv1) is the blue robot, AMR 2 (agv2) the yellow one - see
# pinky.urdf.xacro. The colour terms used to be swapped here.
_VEHICLE_TERMS = (
    ('agv1', (
        'agv1', 'amr1', 'amr 1', '1번 차량', '1호차',
        '파란 차', '파란차', '파란색',
    )),
    ('agv2', (
        'agv2', 'amr2', 'amr 2', '2번 차량', '2호차',
        '노란 차', '노란차', '노란색',
    )),
)
_LABEL_ALIASES = {
    'B-1': (
        'b-1', 'b1', 'b 1', '항구', '항만', '부두', '선적', '하역',
        '상차', '하차',
    ),
    'trailer': ('trailer', '트레일러'),
}
_A_ZONE_ALIASES = (
    'a구역', 'a 구역', 'a존', 'a 존', 'a-zone', 'a zone',
    '화물 보관 구역', '적재 대기', '화물 대기',
)

_TRANSFER_ZONE_LABELS = ('A-1', 'A-2', 'A-3', 'B-1')

_ALL_VEHICLE_TERMS = (
    '모든 차량', '전체 차량', '두 차량', '두 대', '두대', '양쪽 차량',
    # Plural/idle phrasings ("유휴차량들은 주차해줘") also address the whole
    # fleet. Fanning out is safe: the dispatcher skips any vehicle that is
    # busy or already parked, so only the genuinely idle ones move.
    '유휴 차량', '유휴차량', '차량들', '차들',
    'all vehicles', 'both vehicles', 'both agv',
    'idle vehicles', 'idle agv',
)
_SEQUENCE_TERMS = (
    '먼저', '그 다음', '그다음', '다음에', '이후', '도착한 뒤', '도착 후',
    '빠져나오면', '완료한 뒤', '완료 후', '순서대로', '차례로',
    'after ', ' then ', 'first ', 'sequential',
)
_VEHICLE_NAVIGATION_TYPES = {
    'visual_navigation', 'pixel_navigation', 'visual_transfer',
    'park_command',
}
_ARM_ACTION_TYPES = {
    'arm_scan_destinations',
    'arm_transfer_to_slot',
    'arm_load_to_trailer',
    'arm_transfer_by_id',
    'arm1_pick_place',
    'arm_stop',
}
_ARM_COMMAND_TERMS = (
    'arm1', 'arm2', '로봇팔', '매니퓰레이터', '그리퍼',
    'robot arm', 'manipulator',
)


def resolve_execution_mode(command: str, result: Dict) -> str:
    """Choose parallel execution unless the plan has a real dependency."""
    actions = result.get('actions') if isinstance(result, dict) else None
    if not isinstance(actions, list) or len(actions) <= 1:
        return 'sequential'

    lowered = f' {str(command).lower()} '
    if any(term in lowered for term in _SEQUENCE_TERMS):
        return 'sequential'

    navigation_actions = [
        action for action in actions
        if isinstance(action, dict)
        and action.get('type') in _VEHICLE_NAVIGATION_TYPES
    ]
    explicit_vehicle_ids = [
        str(action.get('vehicle_id') or '').strip().lower()
        for action in navigation_actions
        if str(action.get('vehicle_id') or '').strip().lower()
        in {'agv1', 'agv2'}
    ]
    if len(explicit_vehicle_ids) != len(set(explicit_vehicle_ids)):
        return 'sequential'

    arm_actions = [
        action for action in actions
        if isinstance(action, dict)
        and action.get('type') in _ARM_ACTION_TYPES
    ]
    arm_ids = [
        str(action.get('arm_id') or 'arm2').strip().lower()
        for action in arm_actions
    ]
    if len(arm_ids) != len(set(arm_ids)):
        return 'sequential'

    declared = str(result.get('execution_mode') or '').strip().lower()
    if declared in {'parallel', 'sequential'}:
        return declared
    return 'parallel'


def _finalize_navigation_result(command: str, result: Dict, actions) -> Dict:
    """Preserve plan metadata and repair an explicit all-vehicle request."""
    finalized = dict(result)
    finalized['actions'] = actions

    lowered = str(command).lower()
    # "차들"/"차량들" match as substrings, so a single-vehicle phrase like
    # "노란 차들을 항구로" would otherwise fan out to both vehicles and throw
    # the colour away. Naming exactly one vehicle always wins over the
    # plural wording.
    requests_all = (
        any(term in lowered for term in _ALL_VEHICLE_TERMS)
        and len(_mentioned_vehicle_ids(command)) != 1
    )
    if (
        requests_all
        and len(actions) == 1
        and isinstance(actions[0], dict)
    ):
        action = actions[0]
        if action.get('type') in _VEHICLE_NAVIGATION_TYPES:
            finalized['actions'] = []
            for vehicle_id in ('agv1', 'agv2'):
                vehicle_action = dict(action)
                vehicle_action['vehicle_id'] = vehicle_id
                finalized['actions'].append(vehicle_action)
            finalized['execution_mode'] = 'parallel'

    if len(finalized['actions']) > 1 or 'execution_mode' in result:
        finalized['execution_mode'] = resolve_execution_mode(
            command, finalized
        )
    else:
        finalized.pop('execution_mode', None)
    return finalized


def normalize_navigation_result(
    command: str,
    result: Dict,
    yolo_detections: Optional[List[Dict]],
    image_width: int = 640,
    image_height: int = 480,
) -> Dict:
    """Repair broad but actionable visual-navigation responses.

    The VLM remains responsible for understanding the request. This fallback
    only maps explicit destination semantics to objects that are present in
    the current YOLO JSON; it never invents coordinates or detections.
    """
    if not isinstance(result, dict):
        return result

    actions = result.get('actions')
    if not isinstance(actions, list):
        return result

    lowered_command = str(command).lower()
    mentions_arm = any(term in lowered_command for term in _ARM_COMMAND_TERMS)
    has_arm_action = any(
        isinstance(action, dict)
        and action.get('type') in _ARM_ACTION_TYPES
        for action in actions
    )
    if mentions_arm and not has_arm_action:
        # Never reinterpret a failed ARM tool selection as an AGV movement.
        return _finalize_navigation_result(command, result, actions)

    detections = [
        item for item in (yolo_detections or [])
        if isinstance(item, dict)
        and isinstance(item.get('detection_index'), int)
        and item.get('label')
    ]
    if not detections:
        return _finalize_navigation_result(command, result, actions)

    normalized_actions = []
    for action in actions:
        if not isinstance(action, dict):
            normalized_actions.append(action)
            continue
        action_type = action.get('type')
        transfer = _infer_visual_transfer(command, detections, action)
        if transfer is not None and (
            action_type in {
                'cargo_single',
                'cargo_bulk_by_location',
                'cargo_bulk_by_type',
                'visual_transfer',
            }
            or (len(actions) == 1 and action_type in {'unknown', 'travel'})
        ):
            normalized_actions.append(transfer)
            continue
        if action_type == 'travel':
            stops = action.get('stops')
            selected = _infer_target_detection(command, detections)
            if (
                _has_navigation_intent(command, selected is not None)
                and selected is not None
                and (not isinstance(stops, list) or len(stops) <= 1)
            ):
                normalized_actions.append(
                    _visual_action_from_detection(
                        command,
                        selected,
                        action.get('vehicle_id'),
                        image_width,
                        image_height,
                    )
                )
                continue
        if action_type == 'pixel_navigation':
            target = action.get('target')
            heading = action.get('heading')
            if not isinstance(target, dict) or not isinstance(heading, dict):
                selected = _infer_target_detection(command, detections)
                if (
                    _has_navigation_intent(command, selected is not None)
                    and selected is not None
                ):
                    normalized_actions.append(
                        _visual_action_from_detection(
                            command,
                            selected,
                            action.get('vehicle_id'),
                            image_width,
                            image_height,
                        )
                    )
                    continue
        if action_type == 'park_command':
            repaired = dict(action)
            repaired['vehicle_id'] = _infer_vehicle_id(
                command, repaired.get('vehicle_id')
            )
            normalized_actions.append(repaired)
            continue
        if action_type in _ARM_ACTION_TYPES:
            repaired = dict(action)
            repaired['vehicle_id'] = _infer_vehicle_id(
                command, repaired.get('vehicle_id')
            )
            if action_type == 'arm1_pick_place':
                try:
                    destination_id = int(repaired.get('destination_id'))
                except (TypeError, ValueError):
                    destination_id = -1
                vehicle_id = repaired['vehicle_id']
                if (
                    destination_id in set(VEHICLE_TRAILER_MARKERS.values())
                    and vehicle_id in VEHICLE_TRAILER_MARKERS
                ):
                    repaired['destination_id'] = (
                        VEHICLE_TRAILER_MARKERS[vehicle_id]
                    )
            normalized_actions.append(repaired)
            continue
        if action_type != 'visual_navigation':
            normalized_actions.append(action)
            continue

        repaired = dict(action)
        selected = _detection_by_index(
            detections,
            repaired.get('detection_index'),
        )
        if selected is None:
            selected = _infer_target_detection(
                command,
                detections,
                action=repaired,
            )
        if selected is not None:
            repaired.update(
                _visual_action_from_detection(
                    command,
                    selected,
                    repaired.get('vehicle_id'),
                    image_width,
                    image_height,
                )
            )
        normalized_actions.append(repaired)

    if any(
        isinstance(action, dict)
        and action.get('type') != 'unknown'
        for action in normalized_actions
    ):
        return _finalize_navigation_result(
            command, result, normalized_actions
        )

    transfer = _infer_visual_transfer(command, detections)
    if transfer is not None and _has_navigation_intent(command, True):
        return _finalize_navigation_result(command, result, [transfer])

    selected = _infer_target_detection(command, detections)
    if (
        selected is None
        or not _has_navigation_intent(command, selected is not None)
    ):
        return _finalize_navigation_result(
            command, result, normalized_actions
        )

    return _finalize_navigation_result(
        command,
        result,
        [
            _visual_action_from_detection(
                command,
                selected,
                '',
                image_width,
                image_height,
            )
        ],
    )


def _zone_is_owned_by(zone_status, zone_id, vehicle_id):
    """Read one owner from the fleet's compact ``ZONE:owner`` snapshot."""
    expected_zone = str(zone_id or '').strip().upper()
    expected_vehicle = str(vehicle_id or '').strip().lower()
    if not expected_zone or not expected_vehicle:
        return False
    for entry in str(zone_status or '').split(';'):
        parts = [part.strip() for part in entry.split(':')]
        if len(parts) >= 2 and parts[0].upper() == expected_zone:
            return parts[1].lower() == expected_vehicle
    return False


def _repair_arm_before_departure(result, yolo_detections, zone_status=''):
    """Keep an already-arrived vehicle at its arm until cargo work finishes."""
    if not isinstance(result, dict):
        return result
    actions = result.get('actions')
    if not isinstance(actions, list):
        return result
    labels_by_index = {
        item.get('detection_index'): str(item.get('label') or '')
        for item in (yolo_detections or [])
        if isinstance(item, dict)
    }
    repaired = list(actions)
    index = 0
    while index < len(repaired):
        action = repaired[index]
        if not isinstance(action, dict) or action.get('type') not in {
            'arm1_pick_place', 'arm_transfer_to_slot',
            'arm_load_to_trailer',
        }:
            index += 1
            continue
        vehicle_id = str(action.get('vehicle_id') or '').strip().lower()
        required_zone = (
            'B-1' if action.get('type') == 'arm1_pick_place' else 'A'
        )
        if not _zone_is_owned_by(zone_status, required_zone, vehicle_id):
            index += 1
            continue
        required_labels = (
            {'B-1'} if required_zone == 'B-1' else {'A-1', 'A-2', 'A-3'}
        )
        prior_nav_indices = [
            prior_index
            for prior_index, previous in enumerate(repaired[:index])
            if isinstance(previous, dict)
            and previous.get('type') == 'visual_navigation'
            and str(previous.get('vehicle_id') or '').strip().lower()
            == vehicle_id
        ]
        already_revisited = any(
            labels_by_index.get(repaired[prior_index].get('detection_index'))
            in required_labels
            for prior_index in prior_nav_indices
        )
        if prior_nav_indices and not already_revisited:
            insert_at = prior_nav_indices[0]
            repaired.pop(index)
            repaired.insert(insert_at, action)
            index = insert_at + 1
            continue
        index += 1
    # Moving an ARM step ahead of a departure can place the LLM's earlier
    # (incorrectly ordered) destination navigation immediately beside the
    # navigation it had already generated for the real next leg. Sending
    # both creates two chained goals for the same physical A/B-1 stop.
    deduplicated = []
    for action in repaired:
        if (
            deduplicated
            and isinstance(action, dict)
            and action.get('type') == 'visual_navigation'
            and isinstance(deduplicated[-1], dict)
            and deduplicated[-1].get('type') == 'visual_navigation'
        ):
            previous = deduplicated[-1]
            same_vehicle = (
                str(previous.get('vehicle_id') or '').strip().lower()
                == str(action.get('vehicle_id') or '').strip().lower()
            )
            previous_label = labels_by_index.get(
                previous.get('detection_index'), ''
            )
            current_label = labels_by_index.get(
                action.get('detection_index'), ''
            )
            previous_zone = (
                'A' if previous_label in {'A-1', 'A-2', 'A-3'}
                else previous_label
            )
            current_zone = (
                'A' if current_label in {'A-1', 'A-2', 'A-3'}
                else current_label
            )
            if (
                same_vehicle
                and previous_zone
                and previous_zone == current_zone
            ):
                continue
        deduplicated.append(action)
    repaired = deduplicated
    if repaired == actions:
        return result
    normalized = dict(result)
    normalized['actions'] = repaired
    normalized['execution_mode'] = 'sequential'
    return normalized


def _cargo_workflow_issues(result, yolo_detections, zone_status=''):
    """Validate physical prerequisites without interpreting user wording."""
    actions = result.get('actions') if isinstance(result, dict) else None
    if not isinstance(actions, list):
        return []
    labels_by_index = {
        item.get('detection_index'): str(item.get('label') or '')
        for item in (yolo_detections or [])
        if isinstance(item, dict)
    }
    issues = []
    for index, action in enumerate(actions):
        if not isinstance(action, dict) or action.get('type') not in {
            'arm_transfer_to_slot', 'arm_load_to_trailer',
            'arm1_pick_place',
        }:
            continue
        vehicle_id = str(action.get('vehicle_id') or '').strip().lower()
        if (
            action.get('type') == 'arm1_pick_place'
            and not vehicle_id
            and not bool(action.get('final_for_vehicle', False))
        ):
            # ARM1 also supports station-only manual Pick/Place operations.
            # Only vehicle-linked operations need a preceding B-1 arrival.
            continue
        if vehicle_id not in {'agv1', 'agv2'}:
            issues.append(
                f'actions[{index}] ARM 작업에 vehicle_id가 없음'
            )
            continue
        required_labels = (
            {'B-1'}
            if action.get('type') == 'arm1_pick_place'
            else {'A-1', 'A-2', 'A-3'}
        )
        required_zone = (
            'B-1'
            if action.get('type') == 'arm1_pick_place'
            else 'A 작업 위치'
        )
        required_zone_id = (
            'B-1' if action.get('type') == 'arm1_pick_place' else 'A'
        )
        arrived_at_arm = any(
            isinstance(previous, dict)
            and previous.get('type') == 'visual_navigation'
            and str(previous.get('vehicle_id') or '').strip().lower()
            == vehicle_id
            and labels_by_index.get(previous.get('detection_index'))
            in required_labels
            for previous in actions[:index]
        )
        vehicle_moved_before_arm = any(
            isinstance(previous, dict)
            and previous.get('type') == 'visual_navigation'
            and str(previous.get('vehicle_id') or '').strip().lower()
            == vehicle_id
            for previous in actions[:index]
        )
        arrived_at_arm = arrived_at_arm or (
            not vehicle_moved_before_arm
            and _zone_is_owned_by(
                zone_status, required_zone_id, vehicle_id
            )
        )
        if not arrived_at_arm:
            issues.append(
                f'actions[{index}] {vehicle_id} ARM 작업 전에 '
                f'{required_zone} visual_navigation이 없음'
            )
    if issues and str(result.get('execution_mode') or '').lower() != 'sequential':
        issues.append('ARM 연계 계획의 execution_mode가 sequential이 아님')
    return issues


_WAREHOUSE_SLOTS = (
    'A-1-1', 'A-1-2', 'A-2-1',
    'A-2-2', 'A-3-1', 'A-3-2',
)

_SHIP_TERMS = ('선박', 'ship', 'vessel')
_SHIP_PLACE_TERMS = (
    '놓', '내려', '하역', '적재', '싣', '실어', '선적', 'place', 'unload',
)


def _requests_arm1_ship_place(command):
    """Return whether cargo is explicitly being placed onto the vessel."""
    lowered = re.sub(r'\s+', ' ', str(command or '')).lower()
    return (
        any(term in lowered for term in _SHIP_TERMS)
        and any(term in lowered for term in _SHIP_PLACE_TERMS)
        and any(
            term in lowered
            for term in ('컨테이너', '화물', 'container', 'cargo')
        )
    )


def _mentioned_ship_destination_id(command):
    """Resolve an explicitly numbered vessel slot to its registered marker."""
    text = str(command or '').lower()
    patterns = (
        r'(?:선박|ship|vessel)\s*[- ]?([1-6])\s*(?:번\s*)?(?:자리|슬롯|slot)',
        r'(?:선박|ship|vessel)\s*(?:의\s*)?(?:자리|슬롯|slot)\s*[- ]?([1-6])',
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return ARM1_SHIP_SLOT_MARKERS[int(match.group(1))]
    marker_match = re.search(
        r'(?:선박|ship|vessel).*?(?:aruco|아루코|마커)\s*'
        r'(18|19|20|21|22|23)\b',
        text,
    )
    return int(marker_match.group(1)) if marker_match else None


def _mentioned_container_id(command):
    """Extract one container ID explicitly stated by the operator."""
    text = str(command or '').lower()
    patterns = (
        r'\b(\d{1,2})\s*번\s*(?:컨테이너|화물|container|cargo)',
        r'(?:컨테이너|화물|container|cargo)\s*(?:id\s*)?'
        r'(\d{1,2})\s*번?',
    )
    matches = {
        int(match.group(1))
        for pattern in patterns
        for match in re.finditer(pattern, text)
    }
    if len(matches) == 1:
        return next(iter(matches))
    return None


def _manual_inventory_bypass_issues(
    command, load_actions, arm1_actions, load_requested, ship_place_requested
):
    """Validate explicit operator IDs without consulting PostgreSQL."""
    if load_requested and not load_actions and not arm1_actions:
        return ['사용자 상차 요청에 arm_load_to_trailer 단계가 없음']
    if ship_place_requested and not arm1_actions:
        return ['사용자 선박 배치 요청에 arm1_pick_place 단계가 없음']

    explicit_source_id = _mentioned_container_id(command)
    issues = []
    relevant_actions = [*load_actions, *arm1_actions]
    if relevant_actions and explicit_source_id is None:
        issues.append(
            'DB 없는 테스트 모드에서는 컨테이너 ID를 명령에 하나만 '
            '명시해야 함'
        )
    for index, action in relevant_actions:
        try:
            source_id = int(action.get('source_id'))
        except (TypeError, ValueError):
            source_id = -1
        if (
            explicit_source_id is not None
            and source_id != explicit_source_id
        ):
            issues.append(
                f'actions[{index}] 명시한 컨테이너 ID는 '
                f'{explicit_source_id}인데 source_id={source_id}를 선택함'
            )

    for index, action in arm1_actions:
        if not ship_place_requested:
            continue
        try:
            destination_id = int(action.get('destination_id'))
        except (TypeError, ValueError):
            destination_id = -1
        requested_destination_id = _mentioned_ship_destination_id(command)
        if requested_destination_id is not None:
            if destination_id != requested_destination_id:
                issues.append(
                    f'actions[{index}] 지정한 선박 자리는 ArUco '
                    f'{requested_destination_id}인데 destination_id='
                    f'{destination_id}를 선택함'
                )
        elif destination_id not in ARM1_SHIP_DESTINATION_MARKERS:
            issues.append(
                f'actions[{index}] 선박 destination_id={destination_id}는 '
                '등록된 ArUco 범위 18..23에 없음'
            )
    return issues


def _mentioned_inventory_slots(command):
    """Return exact warehouse slots named in free-form Korean/English text."""
    compact = re.sub(r'\s+', '', str(command or '')).upper()
    return [slot for slot in _WAREHOUSE_SLOTS if slot in compact]


def _requests_container_loading(command):
    """Conservatively identify a request to put cargo onto an AMR."""
    if _requests_arm1_ship_place(command):
        # Korean commonly says "선박에 싣다". That is an ARM1 vessel Place,
        # not an ARM2 request to load the container onto an AMR trailer.
        return False
    text = re.sub(r'\s+', '', str(command or '')).lower()
    cargo_terms = ('컨테이너', '화물', 'cargo', 'container')
    load_terms = (
        '싣', '실어', '실고', '실은다음', '상차', '적재',
        'load',
    )
    return (
        any(term in text for term in cargo_terms)
        and any(term in text for term in load_terms)
    )


def inventory_workflow_issues(
    command, result, inventory_snapshot, allow_inventory_bypass=False
):
    """Validate ARM source IDs against the fresh read-only DB snapshot.

    The LLM chooses the container, while this function prevents a guessed or
    stale ID from reaching central control.  A location-only request always
    means the physically accessible top layer at that location.
    """
    actions = result.get('actions') if isinstance(result, dict) else None
    if not isinstance(actions, list):
        return []
    load_actions = [
        (index, action)
        for index, action in enumerate(actions)
        if isinstance(action, dict)
        and action.get('type') == 'arm_load_to_trailer'
    ]
    arm1_actions = [
        (index, action)
        for index, action in enumerate(actions)
        if isinstance(action, dict)
        and action.get('type') == 'arm1_pick_place'
    ]
    load_requested = _requests_container_loading(command)
    ship_place_requested = _requests_arm1_ship_place(command)
    if (
        not load_actions
        and not arm1_actions
        and not load_requested
        and not ship_place_requested
    ):
        return []
    if allow_inventory_bypass:
        return _manual_inventory_bypass_issues(
            command,
            load_actions,
            arm1_actions,
            load_requested,
            ship_place_requested,
        )
    cargos = (
        inventory_snapshot.get('cargos')
        if isinstance(inventory_snapshot, dict)
        else None
    )
    if not isinstance(cargos, list):
        return ['ARM 상차 판단에 필요한 PostgreSQL 재고 스냅샷이 없음']

    if load_requested and not load_actions and not arm1_actions:
        return ['사용자 상차 요청에 arm_load_to_trailer 단계가 없음']
    if ship_place_requested and not arm1_actions:
        return ['사용자 선박 배치 요청에 arm1_pick_place 단계가 없음']

    valid_cargos = []
    by_id = {}
    for cargo in cargos:
        if not isinstance(cargo, dict):
            continue
        container_id = str(cargo.get('container_id') or '').strip()
        location = str(cargo.get('location') or '').strip().upper()
        try:
            floor = int(cargo.get('floor'))
        except (TypeError, ValueError):
            continue
        if not container_id or not location or floor < 1:
            continue
        normalized = {
            'container_id': container_id,
            'location': location,
            'floor': floor,
        }
        valid_cargos.append(normalized)
        by_id[container_id] = normalized

    issues = []
    mentioned_slots = _mentioned_inventory_slots(command)
    expected_top = None
    if len(mentioned_slots) == 1:
        at_location = [
            cargo for cargo in valid_cargos
            if cargo['location'] == mentioned_slots[0]
        ]
        if not at_location:
            issues.append(
                f'{mentioned_slots[0]}에 DB상 컨테이너가 없음'
            )
        else:
            highest_floor = max(cargo['floor'] for cargo in at_location)
            top = [
                cargo for cargo in at_location
                if cargo['floor'] == highest_floor
            ]
            if len(top) != 1:
                issues.append(
                    f'{mentioned_slots[0]}의 최상단 컨테이너를 하나로 '
                    '결정할 수 없음'
                )
            else:
                expected_top = top[0]

    for index, action in load_actions:
        raw_source_id = action.get('source_id')
        selected_id = (
            '' if raw_source_id is None else str(raw_source_id).strip()
        )
        selected = by_id.get(selected_id)
        if selected is None:
            issues.append(
                f'actions[{index}] source_id={selected_id or "empty"}가 '
                'DB에 존재하지 않음'
            )
            continue
        if selected['location'] in _WAREHOUSE_SLOTS:
            same_slot = [
                cargo for cargo in valid_cargos
                if cargo['location'] == selected['location']
            ]
            highest_floor = max(cargo['floor'] for cargo in same_slot)
            if selected['floor'] != highest_floor:
                issues.append(
                    f'actions[{index}] source_id={selected_id}는 '
                    f'{selected["location"]} floor={selected["floor"]}이며 '
                    f'최상단 floor={highest_floor} 아래에 있음'
                )
                continue
        if expected_top is not None and selected_id != expected_top['container_id']:
            issues.append(
                f'actions[{index}] {expected_top["location"]} 최상단은 '
                f'container_id={expected_top["container_id"]} '
                f'(floor={expected_top["floor"]})인데 source_id='
                f'{selected_id}를 선택함'
            )
    for index, action in arm1_actions:
        raw_source_id = action.get('source_id')
        selected_id = (
            '' if raw_source_id is None else str(raw_source_id).strip()
        )
        selected = by_id.get(selected_id)
        if selected is None:
            issues.append(
                f'actions[{index}] ARM1 source_id='
                f'{selected_id or "empty"}가 DB에 존재하지 않음'
            )
            continue
        if ship_place_requested:
            raw_destination_id = action.get('destination_id')
            try:
                destination_id = int(raw_destination_id)
            except (TypeError, ValueError):
                destination_id = -1
            requested_destination_id = _mentioned_ship_destination_id(
                command
            )
            if requested_destination_id is not None:
                if destination_id != requested_destination_id:
                    issues.append(
                        f'actions[{index}] 지정한 선박 자리는 ArUco '
                        f'{requested_destination_id}인데 destination_id='
                        f'{destination_id}를 선택함'
                    )
            elif destination_id not in ARM1_SHIP_DESTINATION_MARKERS:
                issues.append(
                    f'actions[{index}] 선박 destination_id={destination_id}는 '
                    '등록된 ArUco 범위 18..23에 없음'
                )
        if selected['location'] in _WAREHOUSE_SLOTS:
            same_slot = [
                cargo for cargo in valid_cargos
                if cargo['location'] == selected['location']
            ]
            highest_floor = max(cargo['floor'] for cargo in same_slot)
            if selected['floor'] != highest_floor:
                issues.append(
                    f'actions[{index}] ARM1 source_id={selected_id}는 '
                    f'{selected["location"]} floor={selected["floor"]}이며 '
                    f'최상단 floor={highest_floor} 아래에 있음'
                )
                continue
        if expected_top is not None and selected_id != expected_top['container_id']:
            issues.append(
                f'actions[{index}] ARM1 대상 슬롯 '
                f'{expected_top["location"]} 최상단은 container_id='
                f'{expected_top["container_id"]}인데 source_id='
                f'{selected_id}를 선택함'
            )
    return issues


def _infer_visual_transfer(command, detections, action=None):
    """Resolve an explicit visible source-zone to destination-zone request."""
    action = action or {}
    source = _detection_by_index(
        detections,
        action.get('source_detection_index'),
    )
    destination = _detection_by_index(
        detections,
        action.get('destination_detection_index'),
    )
    if source is None or destination is None:
        source, destination = _infer_transfer_zone_pair(command, detections)
    if (
        source is None
        or destination is None
        or source.get('label') == destination.get('label')
    ):
        return None
    return {
        'type': 'visual_transfer',
        'source_detection_index': source['detection_index'],
        'destination_detection_index': destination['detection_index'],
        'vehicle_id': _infer_vehicle_id(command, action.get('vehicle_id')),
    }


def _infer_transfer_zone_pair(command, detections):
    text = str(command or '').strip().lower()
    by_label = {}
    for detection in detections:
        label = str(detection.get('label'))
        if label not in _TRANSFER_ZONE_LABELS:
            continue
        current = by_label.get(label)
        confidence = float(detection.get('confidence') or 0.0)
        current_confidence = float(
            (current or {}).get('confidence') or 0.0
        )
        if current is None or confidence > current_confidence:
            by_label[label] = detection
    mentions = []
    aliases = {
        'A-1': (r'a\s*[- ]?\s*1',),
        'A-2': (r'a\s*[- ]?\s*2',),
        'A-3': (r'a\s*[- ]?\s*3',),
        'B-1': (
            r'b\s*[- ]?\s*1',
            r'항구',
            r'항만',
            r'부두',
        ),
    }
    for label, patterns in aliases.items():
        if label not in by_label:
            continue
        for pattern in patterns:
            for match in re.finditer(pattern, text):
                tail = text[match.end():match.end() + 12]
                mentions.append({
                    'start': match.start(),
                    'label': label,
                    'destination': bool(
                        re.match(r'\s*(?:구역|존)?\s*(?:으)?로', tail)
                        or re.match(r'\s*(?:구역|존)?\s*까지', tail)
                    ),
                    'source': bool(
                        re.match(
                            r'\s*(?:구역|존)?\s*(?:에서|에\s*있는|의)',
                            tail,
                        )
                    ),
                })
    mentions.sort(key=lambda item: item['start'])
    unique = []
    for mention in mentions:
        if mention['label'] not in [item['label'] for item in unique]:
            unique.append(mention)
    if len(unique) < 2:
        return None, None

    destination_mentions = [item for item in unique if item['destination']]
    destination_mention = (
        destination_mentions[-1] if destination_mentions else unique[-1]
    )
    source_mentions = [
        item for item in unique
        if item['label'] != destination_mention['label'] and item['source']
    ]
    source_mention = (
        source_mentions[0]
        if source_mentions
        else next(
            item for item in unique
            if item['label'] != destination_mention['label']
        )
    )
    return (
        by_label[source_mention['label']],
        by_label[destination_mention['label']],
    )


def _has_navigation_intent(
    command: str,
    target_present: bool = False,
) -> bool:
    text = str(command or '').strip().lower()
    if not text:
        return False
    if any(re.search(pattern, text) for pattern in _NEGATED_MOTION_PATTERNS):
        return False
    if not target_present and any(term in text for term in _STOP_ONLY_TERMS):
        return False
    return (
        any(term in text for term in _MOTION_TERMS)
        or any(re.search(pattern, text) for pattern in _MOTION_PATTERNS)
    )


def _detection_by_index(detections, index):
    if isinstance(index, str) and index.strip().isdigit():
        index = int(index.strip())
    if isinstance(index, bool) or not isinstance(index, int):
        return None
    return next(
        (
            detection for detection in detections
            if detection.get('detection_index') == index
        ),
        None,
    )


def _infer_target_detection(command, detections, action=None):
    text = str(command or '').strip().lower()
    labels = {
        str(detection.get('label')): detection
        for detection in detections
    }

    action = action or {}
    action_label = str(
        action.get('target_label') or action.get('label') or ''
    ).strip()
    if action_label in labels:
        return labels[action_label]

    for label in ('A-1', 'A-2', 'A-3', 'B-1'):
        variants = {
            label.lower(),
            label.lower().replace('-', ''),
            label.lower().replace('-', ' '),
        }
        if any(variant in text for variant in variants) and label in labels:
            return labels[label]

    if any(alias in text for alias in _A_ZONE_ALIASES):
        for label in ('A-1', 'A-2', 'A-3'):
            if label in labels:
                return labels[label]

    for label, aliases in _LABEL_ALIASES.items():
        if label in labels and any(alias in text for alias in aliases):
            return labels[label]

    for label, detection in labels.items():
        normalized = label.lower()
        if normalized not in {'car_yellow', 'car_blue'} and normalized in text:
            return detection
    return None


def _infer_approach_side(
    command,
    detection,
    image_width=640,
    image_height=480,
):
    text = str(command or '').strip().lower()
    for side, aliases in _SIDE_TERMS:
        if any(alias in text for alias in aliases):
            return side

    if detection.get('label') in {'A-1', 'A-2', 'A-3', 'B-1'}:
        return 'bottom'

    bbox = detection.get('bbox_xyxy')
    if not isinstance(bbox, list) or len(bbox) != 4:
        return 'bottom'
    x1, y1, x2, y2 = (float(value) for value in bbox)
    clearances = {
        'left': x1,
        'right': float(image_width) - x2,
        'top': y1,
        'bottom': float(image_height) - y2,
    }
    return max(clearances, key=clearances.get)


def _visual_action_from_detection(
    command,
    detection,
    vehicle_id,
    image_width,
    image_height,
):
    return {
        'type': 'visual_navigation',
        'detection_index': detection['detection_index'],
        'approach_side': _infer_approach_side(
            command,
            detection,
            image_width,
            image_height,
        ),
        'vehicle_id': _infer_vehicle_id(command, vehicle_id),
    }


def _mentioned_vehicle_ids(command):
    """Return every vehicle the command names outright.

    An empty set means the request never says which vehicle; two entries mean
    it addressed both by name. Only a single entry marks a request that one
    specific vehicle must serve alone.
    """
    text = str(command or '').strip().lower()
    return {
        vehicle_id
        for vehicle_id, aliases in _VEHICLE_TERMS
        if any(alias in text for alias in aliases)
    }


def _infer_vehicle_id(command, current_value):
    mentioned = _mentioned_vehicle_ids(command)
    if len(mentioned) == 1:
        return next(iter(mentioned))
    current = str(current_value or '').strip().lower()
    if current in {'agv1', 'agv2'}:
        return current
    text = str(command or '').strip().lower()
    for vehicle_id, aliases in _VEHICLE_TERMS:
        if any(alias in text for alias in aliases):
            return vehicle_id
    return ''


def _decode_llm_response(response):
    raw_text = (response['message']['content'] or '').strip()
    if raw_text.startswith('```'):
        raw_text = raw_text.strip('`')
        if raw_text.startswith('json'):
            raw_text = raw_text[4:]
        raw_text = raw_text.strip()
    try:
        result = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        raise LLMParseError(
            f'모델 응답을 JSON으로 해석하지 못했습니다: {raw_text!r}'
        ) from exc
    if not isinstance(result, dict) or not isinstance(
        result.get('actions'), list
    ):
        raise LLMParseError(
            f'예상한 JSON 형식이 아닙니다 (actions 배열 없음): {result!r}'
        )
    return result, raw_text


def parse_command_with_llm(
    command: str,
    known_items: List[str],
    known_locations: List[str],
    known_cargo_types: Optional[List[str]] = None,
    model: Optional[str] = None,
    host: Optional[str] = None,
    timeout: float = 90.0,
    image_jpeg: Optional[bytes] = None,
    image_width: int = 640,
    image_height: int = 480,
    yolo_detections: Optional[List[Dict]] = None,
    normalization_command: Optional[str] = None,
    inventory_snapshot: Optional[Dict] = None,
    allow_inventory_bypass: bool = False,
    zone_status: str = '',
) -> Dict:
    """
    자연어 명령을 로컬 Ollama 모델로 해석해서 구조화된 dict로 반환합니다.
    반환 형식은 항상 {"actions": [action, ...]} 이며, 각 action은
    _SYSTEM_PROMPT_TEMPLATE에 정의된 15가지 type 중 하나입니다.
    (한 문장에 지시가 여러 개 섞여 있으면 actions에 여러 개가 들어옵니다)

    실패(패키지 미설치, Ollama 서버 미실행, 모델 미설치, JSON 파싱 실패 등) 시
    LLMParseError를 던집니다 - 호출하는 쪽(command_center.py)에서 이를 잡아
    규칙 기반 파서로 자동 대체(폴백)하도록 설계되어 있습니다.

    로컬 모델은 API보다 응답이 느릴 수 있어서(특히 CPU만 있는 경우) timeout을
    기본 90초로 넉넉하게 잡았습니다 (특히 모델을 방금 불러온 직후 첫 호출은
    가중치를 메모리에 올리는 데만 십몇 초가 걸릴 수 있어서 더 여유를 뒀습니다).
    """
    try:
        import ollama
    except ImportError as exc:
        raise LLMParseError(f"ollama 패키지가 설치되어 있지 않습니다: {exc}") from exc

    system_prompt = _SYSTEM_PROMPT_TEMPLATE.format(
        items=", ".join(known_items) if known_items else "(등록된 화물 없음)",
        locations=", ".join(known_locations) if known_locations else "(등록된 위치 없음)",
        cargo_types=", ".join(known_cargo_types) if known_cargo_types else "(등록된 화물종류 없음)",
        image_width=image_width,
        image_height=image_height,
        zone_status=str(zone_status or 'unavailable'),
        inventory_snapshot=json.dumps(
            inventory_snapshot,
            ensure_ascii=False,
            separators=(',', ':'),
        ) if inventory_snapshot is not None else 'unavailable',
        inventory_policy=(
            'DB 없는 수동 테스트 모드입니다. 사용자가 명령에 직접 '
            '지정한 컨테이너 ID는 source_id로 사용할 수 있지만, 생략된 '
            'ID나 적층 상태는 추측하지 말고 unknown을 반환하세요.'
            if allow_inventory_bypass
            else '재고 스냅샷이 unavailable이면 source_id를 추측하지 말고 '
            'unknown을 반환하세요.'
        ),
    )

    client = ollama.Client(host=host or OLLAMA_HOST, timeout=timeout)
    detection_context = json.dumps(
        yolo_detections or [],
        ensure_ascii=False,
        separators=(',', ':'),
    )
    user_content = (
        f'사용자 명령:\n{command}\n\n'
        '현재 프레임 YOLO 검출 JSON:\n'
        f'{detection_context}'
    )
    user_message = {"role": "user", "content": user_content}
    if image_jpeg:
        user_message["images"] = [image_jpeg]
    chat_kwargs = dict(
        model=model or MODEL_NAME,
        messages=[
            {"role": "system", "content": system_prompt},
            user_message,
        ],
        format="json",   # Ollama가 유효한 JSON만 내놓도록 강제 (지원하는 모델 기준)
        options={
            "temperature": 0,
            # The VLM prompt includes the image and detection context, so the
            # Ollama default of 4096 tokens is too small for normal requests.
            "num_ctx": LLM_NUM_CTX,
        },
    )

    try:
        # Qwen3처럼 "생각 모드"가 있는 모델은 그 과정을 끄고 바로 답만 받도록 시도
        response = client.chat(think=False, **chat_kwargs)
    except ConnectionError as exc:
        raise LLMParseError(
            "Ollama 서버에 연결할 수 없습니다. 'ollama serve'가 실행 중인지 확인해주세요. "
            f"({exc})"
        ) from exc
    except Exception:
        # think 파라미터를 서버/모델이 아직 지원하지 않는 경우를 대비한 재시도
        # (오래된 Ollama 서버 버전이거나, 생각 모드 자체가 없는 모델일 때)
        try:
            response = client.chat(**chat_kwargs)
        except ConnectionError as exc:
            raise LLMParseError(
                "Ollama 서버에 연결할 수 없습니다. 'ollama serve'가 실행 중인지 확인해주세요. "
                f"({exc})"
            ) from exc
        except Exception as exc:
            # 모델이 설치되어 있지 않을 때도 여기로 들어옵니다 (예: "ollama pull qwen2.5:7b" 필요)
            raise LLMParseError(f"로컬 LLM 호출 실패: {exc}") from exc

    result, raw_text = _decode_llm_response(response)
    normalized = normalize_navigation_result(
        normalization_command or command,
        result,
        yolo_detections,
        image_width,
        image_height,
    )
    normalized = _repair_arm_before_departure(
        normalized, yolo_detections, zone_status
    )
    issues = _cargo_workflow_issues(
        normalized, yolo_detections, zone_status
    )
    issues.extend(inventory_workflow_issues(
        normalization_command or command,
        normalized,
        inventory_snapshot,
        allow_inventory_bypass=allow_inventory_bypass,
    ))
    if not issues:
        return normalized

    correction = (
        '방금 생성한 계획은 물리적 선행조건을 누락했습니다. '
        '아래 검증 오류를 모두 해결하여 전체 JSON 계획을 다시 '
        '생성하세요. 사용자 명령을 다시 해석하고, 주어진 YOLO '
        '검출에 있는 detection_index만 사용하세요.\n'
        + '\n'.join(f'- {issue}' for issue in issues)
    )
    review_kwargs = dict(chat_kwargs)
    review_kwargs['messages'] = [
        *chat_kwargs['messages'],
        {'role': 'assistant', 'content': raw_text},
        {'role': 'user', 'content': correction},
    ]
    try:
        reviewed_response = client.chat(think=False, **review_kwargs)
    except Exception:
        try:
            reviewed_response = client.chat(**review_kwargs)
        except Exception as exc:
            raise LLMParseError(f'LLM 계획 보정 실패: {exc}') from exc
    reviewed, _ = _decode_llm_response(reviewed_response)
    normalized = normalize_navigation_result(
        normalization_command or command,
        reviewed,
        yolo_detections,
        image_width,
        image_height,
    )
    normalized = _repair_arm_before_departure(
        normalized, yolo_detections, zone_status
    )
    remaining_issues = _cargo_workflow_issues(
        normalized, yolo_detections, zone_status
    )
    remaining_issues.extend(inventory_workflow_issues(
        normalization_command or command,
        normalized,
        inventory_snapshot,
        allow_inventory_bypass=allow_inventory_bypass,
    ))
    if remaining_issues:
        return {
            'execution_mode': 'sequential',
            'actions': [{
                'type': 'unknown',
                'reason': (
                    'LLM 계획이 ARM 안전 선행조건을 충족하지 '
                    '못함: ' + '; '.join(remaining_issues)
                ),
            }],
            'suppress_rule_fallback': True,
        }
    return normalized


if __name__ == "__main__":
    # 실제 Ollama가 실행 중일 때만 동작하는 간단한 수동 확인용
    import sys

    if len(sys.argv) < 2:
        print("사용법: python llm_command_parser.py \"cargo A를 port로 go\"")
        sys.exit(1)

    demo_items = ["화물A", "화물B"]
    demo_locations = ["항구", "창고 A", "창고 B", "창고 C", "대기장소"]
    demo_cargo_types = ["컨테이너", "팔레트", "일반화물"]

    try:
        result = parse_command_with_llm(sys.argv[1], demo_items, demo_locations, demo_cargo_types)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except LLMParseError as exc:
        print(f"파싱 실패: {exc}")
