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

_SYSTEM_PROMPT_TEMPLATE = """당신은 항만 자율주행 로봇 시스템의 자연어 명령 해석기입니다.
사용자가 어떤 언어로 말하든, 어떤 동의어/약어를 쓰든 아래 "등록된 이름" 목록 중
정확히 일치하는 이름으로 매핑해야 합니다.

등록된 화물명: {items}
등록된 위치명: {locations}
등록된 화물종류: {cargo_types}
현재 탑다운 영상 크기: {image_width}x{image_height}

우리 AGV 차량은 YOLO 검출 JSON에서 label로 구분됩니다: car_yellow=agv1(노란색
차량), car_blue=agv2(파란색 차량). 이 둘은 화물이나 장애물이 아니라 우리가 직접
제어하는 차량 자신입니다.

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
하나의 action에는 한 차량의 한 이동만 넣으세요. "먼저", "그 다음", "이후",
"빠져나오면", "도착한 뒤" 같은 순서 표현이 있으면 반드시 서로 다른 action으로
나누고, 조건을 만족하는 순서대로 배열하세요.

각 action은 아래 14가지 형식 중 하나입니다:

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

13) ARM2 작업을 즉시 정지하는 경우:
{{"type": "arm_stop", "arm_id": "arm2"}}

ARM 작업은 위 다섯 형식만 사용할 수 있습니다. ROS 서비스 이름이나 임의 operation을
만들지 마세요. ARM1은 아직 중앙 서비스 계약이 없으므로 ARM1 작업 요청은 unknown으로
반환하세요. 같은 ARM2에 여러 작업을 지시하면 반드시 명시된 순서대로 actions에 넣고
execution_mode는 sequential로 설정하세요. 차량과 연결된 마지막 하역/상차 작업에만
final_for_vehicle=true를 지정하세요. 이 값이 true인 작업이 최종 성공한 뒤에만 해당
차량이 출발할 수 있습니다. 창고 내부 이동 arm_transfer_by_id에는 차량 출발 승인을
연결하지 마세요.

14) 위 어느 것에도 해당하지 않는 경우:
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
    requests_all = any(term in lowered for term in _ALL_VEHICLE_TERMS)
    if requests_all and len(actions) == 1 and isinstance(actions[0], dict):
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


def _infer_vehicle_id(command, current_value):
    current = str(current_value or '').strip().lower()
    if current in {'agv1', 'agv2'}:
        return current
    text = str(command or '').strip().lower()
    for vehicle_id, aliases in _VEHICLE_TERMS:
        if any(alias in text for alias in aliases):
            return vehicle_id
    return ''


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
) -> Dict:
    """
    자연어 명령을 로컬 Ollama 모델로 해석해서 구조화된 dict로 반환합니다.
    반환 형식은 항상 {"actions": [action, ...]} 이며, 각 action은
    _SYSTEM_PROMPT_TEMPLATE에 정의된 14가지 type 중 하나입니다.
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

    raw_text = (response["message"]["content"] or "").strip()

    # 혹시 모델이 ```json ... ``` 코드블록으로 감싸서 응답하면 벗겨냄 (안전장치)
    if raw_text.startswith("```"):
        raw_text = raw_text.strip("`")
        if raw_text.startswith("json"):
            raw_text = raw_text[4:]
        raw_text = raw_text.strip()

    try:
        result = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        raise LLMParseError(f"모델 응답을 JSON으로 해석하지 못했습니다: {raw_text!r}") from exc

    if not isinstance(result, dict) or not isinstance(result.get("actions"), list):
        raise LLMParseError(f"예상한 JSON 형식이 아닙니다 (actions 배열 없음): {result!r}")

    return normalize_navigation_result(
        normalization_command or command,
        result,
        yolo_detections,
        image_width,
        image_height,
    )


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
