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
from typing import Dict, List, Optional

# 팀 공유 Ollama 서버 주소/모델. 다른 서버나 로컬로 바꾸고 싶으면 환경변수로 덮어쓰세요.
MODEL_NAME = os.environ.get("LOCAL_LLM_MODEL", "gemma4:31b")
OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://agent.sds.codes")

_SYSTEM_PROMPT_TEMPLATE = """당신은 항만 자율주행 로봇 시스템의 자연어 명령 해석기입니다.
사용자가 어떤 언어로 말하든, 어떤 동의어/약어를 쓰든 아래 "등록된 이름" 목록 중
정확히 일치하는 이름으로 매핑해야 합니다.

등록된 화물명: {items}
등록된 위치명: {locations}
등록된 화물종류: {cargo_types}
현재 탑다운 영상 크기: {image_width}x{image_height}

사용자 문장을 분석해서 반드시 아래 형식으로만 응답하세요. 설명, 인사, 코드블록(```) 등
JSON 이외의 텍스트는 절대 포함하지 마세요.

{{"actions": [ <action>, <action>, ... ]}}

한 문장에 지시가 하나면 actions 배열에 1개만 넣고, 지시가 여러 개면(예: "A는 B로,
C는 D로") 각각을 별도 action으로 배열에 전부 넣으세요.

각 action은 아래 7가지 형식 중 하나입니다:

1) 화물 하나를 특정 위치로 옮기는 경우 (계층적 적재 포함):
{{"type": "cargo_single", "item": "<화물명>", "destination": "<등록된 위치명>", "target_aruco_id": "<목표 ArUco ID>", "target_floor": <목표 층수 정수>}}
주의: 
- 로봇팔은 반드시 ArUco ID나 적재 층수(위치) 기준으로만 동작 가능합니다. 사용자가 특정 위치나 베이스를 지목하면 target_aruco_id와 target_floor를 채워주세요.
- item은 등록된 화물명 목록에 없는 이름이어도 사용자가 말한 그대로 넣으세요.
- destination(목적지)은 반드시 등록된 위치명 중 하나여야 합니다.
- target_aruco_id: 사용자가 특정 ArUco ID(예: ARUCO_9번 위)를 지목하면 기입하고, 언급이 없으면 빈 문자열 ""을 넣으세요.
- target_floor: 사용자가 층수를 지목하면 해당 숫자를 넣고(예: 3층 -> 3), 언급이 없으면 기본값 1을 넣으세요.

2) 특정 위치에 있는 화물 전부를 다른 위치로 옮기는 경우 (예: "창고에 있는 물건 전부를 항만으로 이동"):
{{"type": "cargo_bulk_by_location", "source_location": "<등록된 위치명>", "destination": "<등록된 위치명>"}}

3) 특정 종류의 화물 전부를 옮기는 경우 (예: "컨테이너 화물은 항구로"):
{{"type": "cargo_bulk_by_type", "cargo_type": "<등록된 화물종류>", "destination": "<등록된 위치명>"}}

4) 화물 언급 없이 순수 위치 이동만 하는 경우 (여러 지점 경유 가능, 언급 순서대로):
{{"type": "travel", "stops": ["<등록된 위치명>", "..."]}}

5) 사용자가 YOLO로 검출된 객체에 접근하라고 지시하는 경우:
{{"type": "visual_navigation",
  "detection_index": <검출 JSON의 detection_index>,
  "approach_side": "<left|right|top|bottom>"}}

검출 객체를 지칭한 경우 좌표를 직접 추측하지 말고 반드시 visual_navigation을
사용하세요. approach_side는 영상에서 장애물이 적고 접근 공간이 충분한 쪽을 고르세요.
검출 JSON에 없는 객체를 만들어내지 마세요.
단, "B-1 차를 보내줘", "A 차량을 이동시켜"처럼 검출된 차량 자체를 이동시키라고
했지만 목적지가 없는 명령은 그 차량으로 접근하라는 뜻이 아닙니다. 목적지가 없으므로
unknown을 반환하세요. visual_navigation은 "B-1 근처로 가", "B-1의 아래쪽으로 접근해"
같이 현재 제어 차량이 검출 객체 근처로 이동한다는 뜻이 명확할 때만 사용하세요.
`B-1`은 항구 상차·하차 전용 주차 구역입니다. "B-1로 차를 보내줘"처럼 B-1을
목적지로 지정하면 반드시 B-1 검출의 detection_index를 사용한 visual_navigation을
반환하세요. B-1의 실제 주차 좌표와 방향은 제어 코드가 결정하므로 approach_side는
어느 값을 반환해도 무시됩니다.

6) 사용자가 객체가 아닌 현재 영상의 빈 공간을 목표로 지정하는 경우:
{{"type": "pixel_navigation",
  "target": {{"x": <목표 픽셀 x>, "y": <목표 픽셀 y>}},
  "heading": {{"x": <방향 픽셀 x>, "y": <방향 픽셀 y>}}}}

target은 차량 중심이 최종적으로 도착할 이미지 픽셀입니다.
heading은 target에서 차량 앞쪽이 바라볼 방향에 있는 별도의 이미지 픽셀입니다.
두 점은 최소 30픽셀 이상 떨어뜨리고 모두 영상 범위 안에 두세요.
벽, 컨테이너, 사람, 다른 차량 위를 목표로 선택하지 마세요.
영상이 없거나 목표를 안전하게 특정할 수 없으면 좌표를 추측하지 말고 unknown을
반환하세요.

7) 특정 색상의 PinkyPro 로봇의 시동(bringup)을 거는 경우 (예: "노란색 핑키프로 시동 걸어", "파란색 시동 켜"):
{{"type": "bringup", "color": "<색상>"}}
주의: color는 "yellow", "blue" 등 영어 색상명 또는 "노란색", "파란색" 등 한국어 색상명에서 핵심 색상만 추출하여 영어 소문자(yellow, blue 등)로 통일해서 넣으세요.

8) 특정 색상의 PinkyPro 로봇의 시동(bringup 및 LED)을 끄는 경우 (예: "노란색 핑키프로 시동 꺼", "파란색 종료해"):
{{"type": "shutdown", "color": "<색상>"}}

10) 객체(화물, 차량 등)의 위치를 물어보는 경우 (예: "C0 어딨어?", "차량 위치 알려줘"):
{{"type": "query_location", "item": "<객체명>"}}

11) 위 어느 것에도 해당하지 않는 경우:
{{"type": "unknown", "reason": "<간단한 이유>"}}

주의:
- "port"/"항만"/"항구"처럼 언어나 표기가 달라도 의미가 같으면 같은 등록 이름으로 매핑하세요.
- "go"/"이동"/"가"처럼 동사 표현이 달라도 전부 이동 명령으로 이해하세요.
- 목적지/위치명/화물종류는 반드시 위 목록에 있는 이름과 글자까지 정확히 일치해야 합니다.
- 화물명은 등록 목록에 없어도 사용자가 말한 이름 그대로 사용하세요.
- "A는 X로, B는 Y로"처럼 문장 안에 서로 다른 지시가 섞여 있으면 절대 하나로 합치지 말고
  actions 배열에 각각 나눠서 넣으세요.
- AGV는 목적지가 '창고'면 실제 창고 내부가 아닌 '창고 하역장'으로 가고, '배/항구'면 '야드 하역장'으로 가는 것이 물리적 규칙입니다. 하지만 이런 복잡한 하역장 경유 로직과 'AGV 도착 후 로봇팔 동작' 같은 시퀀스는 중앙 제어 시스템이 알아서 처리합니다. 당신은 사용자가 말한 "최종 목적지"(예: "항구", "A-1-1")만 destination으로 지정하면 됩니다.
- 작업 1순위는 화물 명령 수행이며, 명령이 끝나면 AGV는 자동으로 주차(대기)합니다. 단, 사용자가 명시적으로 "차량 대기해", "주차장으로 가"라고 지시할 경우에만 LLM에서 직접 travel(대기장소) 명령을 내리세요.
- 현재 영상의 빈 공간이나 특정 영상 지점을 지정한 명령은 등록 위치명으로 억지로
  바꾸지 말고 pixel_navigation을 사용하세요.
- 현재 영상의 검출 객체를 언급하면 visual_navigation을 사용하고, 객체가 아닌
  빈 바닥이나 특정 영상 지점일 때만 pixel_navigation을 사용하세요.

예시:
사용자: "컨테이너 화물은 항구로, 팔레트 화물은 대기장소로 이동"
{{"actions": [
  {{"type": "cargo_bulk_by_type", "cargo_type": "컨테이너", "destination": "항구"}},
  {{"type": "cargo_bulk_by_type", "cargo_type": "팔레트", "destination": "대기장소"}}
]}}
"""


class LLMParseError(Exception):
    """LLM 호출 또는 응답 파싱에 실패했을 때 발생합니다. 호출 쪽에서 규칙 기반 파서로 대체(폴백)하는 용도로 씁니다."""


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
) -> Dict:
    """
    자연어 명령을 로컬 Ollama 모델로 해석해서 구조화된 dict로 반환합니다.
    반환 형식은 항상 {"actions": [action, ...]} 이며, 각 action은
    _SYSTEM_PROMPT_TEMPLATE에 정의된 6가지 type 중 하나입니다.
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
        options={"temperature": 0},  # 항상 같은 해석이 나오도록 (일관성 우선)
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

    return result


if __name__ == "__main__":
    # 실제 Ollama가 실행 중일 때만 동작하는 간단한 수동 확인용
    import sys

    if len(sys.argv) < 2:
        print("사용법: python llm_command_parser.py \"cargo A를 port로 go\"")
        sys.exit(1)

    demo_items = ["컨테이너_C0", "컨테이너_C1", "컨테이너_C2", "컨테이너_C3", "컨테이너_C4"]
    demo_locations = ["항구", "A-1-1", "A-1-2", "A-2-1", "A-2-2", "A-3-1", "A-3-2", "대기장소 1", "대기장소 2", "야드 하역장", "창고 하역장"]
    demo_cargo_types = ["컨테이너", "팔레트", "특수화물"]

    try:
        result = parse_command_with_llm(sys.argv[1], demo_items, demo_locations, demo_cargo_types)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except LLMParseError as exc:
        print(f"파싱 실패: {exc}")
