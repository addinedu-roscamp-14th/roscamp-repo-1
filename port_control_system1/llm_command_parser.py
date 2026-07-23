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

사용자 문장을 분석해서 반드시 아래 형식으로만 응답하세요. 설명, 인사, 코드블록(```) 등
JSON 이외의 텍스트는 절대 포함하지 마세요.

{{"actions": [ <action>, <action>, ... ]}}

한 문장에 지시가 하나면 actions 배열에 1개만 넣고, 지시가 여러 개면(예: "A는 B로,
C는 D로") 각각을 별도 action으로 배열에 전부 넣으세요.

각 action은 아래 5가지 형식 중 하나입니다:

1) 화물 하나를 특정 위치로 옮기는 경우:
{{"type": "cargo_single", "item": "<화물명>", "destination": "<등록된 위치명>"}}
주의: item은 등록된 화물명 목록에 없는 이름이어도 사용자가 말한 그대로 넣으세요.
      등록되지 않은 화물이면 시스템이 자동으로 등록합니다.
      단, destination(목적지)은 반드시 등록된 위치명 중 하나여야 합니다.

2) 특정 위치에 있는 화물 전부를 다른 위치로 옮기는 경우 (예: "창고에 있는 물건 전부를 항만으로 이동"):
{{"type": "cargo_bulk_by_location", "source_location": "<등록된 위치명>", "destination": "<등록된 위치명>"}}

3) 특정 종류의 화물 전부를 옮기는 경우 (예: "컨테이너 화물은 항구로"):
{{"type": "cargo_bulk_by_type", "cargo_type": "<등록된 화물종류>", "destination": "<등록된 위치명>"}}

4) 화물 언급 없이 순수 위치 이동만 하는 경우 (여러 지점 경유 가능, 언급 순서대로):
{{"type": "travel", "stops": ["<등록된 위치명>", "..."]}}

5) 위 어느 것에도 해당하지 않는 경우:
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
) -> Dict:
    """
    자연어 명령을 로컬 Ollama 모델로 해석해서 구조화된 dict로 반환합니다.
    반환 형식은 항상 {"actions": [action, ...]} 이며, 각 action은
    _SYSTEM_PROMPT_TEMPLATE에 정의된 5가지 type 중 하나입니다.
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
    )

    client = ollama.Client(host=host or OLLAMA_HOST, timeout=timeout)
    chat_kwargs = dict(
        model=model or MODEL_NAME,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": command},
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

    demo_items = ["화물A", "화물B"]
    demo_locations = ["항구", "창고 A", "창고 B", "창고 C", "대기장소"]
    demo_cargo_types = ["컨테이너", "팔레트", "일반화물"]

    try:
        result = parse_command_with_llm(sys.argv[1], demo_items, demo_locations, demo_cargo_types)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except LLMParseError as exc:
        print(f"파싱 실패: {exc}")
