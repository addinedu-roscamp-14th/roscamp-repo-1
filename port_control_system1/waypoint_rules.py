"""
waypoint_rules.py

특정 목적지로 이동할 때 반드시 거쳐야 하는 경유지(회차지점) 규칙을 관리합니다.
예: 항구로 갈 때는 항상 회차지점A를 거치고, 창고로 갈 때는 항상 회차지점B를 거친다.

- 경유지 자체도 일반 위치와 똑같이 location_marker_tool.py / dual_view_calibrator.py로
  마킹해서 등록하면 됩니다. (경유지 전용 도구가 따로 필요하지 않습니다)
- 이 모듈은 규칙 저장/조회 + 경로에 규칙을 적용하는 순수 로직만 담당합니다.
  customtkinter에 의존하지 않아서 다른 도구들에서 그대로 import해서 씁니다.
"""

import json
from pathlib import Path
from typing import Dict, List, Optional

# 실행 위치(cwd)와 무관하게 항상 이 스크립트가 있는 폴더의 파일을 쓰도록 고정합니다.
_APP_DIR = Path(__file__).resolve().parent
WAYPOINT_RULES_FILE = str(_APP_DIR / "waypoint_rules.json")


def load_waypoint_rules(path: str = WAYPOINT_RULES_FILE) -> Dict[str, str]:
    """{"목적지명": "필수경유지명"} 형태로 반환."""
    p = Path(path)
    if not p.exists():
        return {}
    return json.loads(p.read_text(encoding="utf-8"))


def save_waypoint_rules(rules: Dict[str, str], path: str = WAYPOINT_RULES_FILE) -> None:
    Path(path).write_text(json.dumps(rules, ensure_ascii=False, indent=2), encoding="utf-8")


def resolve_travel_route(
    destination: str,
    rules: Optional[Dict[str, str]] = None,
    _visited: Optional[set] = None,
) -> List[str]:
    """
    현재 위치가 어디든 상관없이, destination으로 갈 때 반드시 거쳐야 하는 경유지를
    앞에 붙여서 순서대로 반환합니다. (예: "항구" -> ["회차지점A", "항구"])
    규칙이 없으면 [destination] 그대로 반환합니다.
    경유지 자체에도 규칙이 걸려 있으면 재귀적으로 더 앞에 붙습니다(순환 참조는 방지).
    """
    if rules is None:
        rules = load_waypoint_rules()
    if _visited is None:
        _visited = set()  # 재귀 중에 이미 거쳐간 목적지를 기록해서 무한 루프 방지

    if destination in _visited:
        # 순환 참조(A의 경유지가 B, B의 경유지가 다시 A인 경우) 방지
        return [destination]
    _visited.add(destination)

    waypoint = rules.get(destination)  # 이 목적지에 필수 경유지 규칙이 있는지 조회
    if not waypoint or waypoint == destination:
        return [destination]  # 규칙이 없으면(또는 자기 자신이면) 그대로 목적지만 반환

    # 경유지 자체에도 규칙이 걸려 있을 수 있으므로 재귀 호출로 계속 앞에 붙여나감
    # 예: 항구의 경유지가 회차지점A이고, 회차지점A의 경유지가 대문이라면
    #     최종적으로 [대문, 회차지점A, 항구] 순서가 됨
    return resolve_travel_route(waypoint, rules, _visited) + [destination]


def expand_leg(
    from_location: str,
    to_location: str,
    rules: Optional[Dict[str, str]] = None,
) -> List[str]:
    """
    화물 배차처럼 "어디서 출발해서 어디로 가는지"가 이미 정해진 구간(leg)에 대해,
    to_location에 필수 경유지 규칙이 있으면 그 경유지를 거치도록 확장합니다.
    (출발지와 경유지가 같으면 중복으로 넣지 않음)
    """
    if rules is None:
        rules = load_waypoint_rules()

    # to_location까지 필요한 경유지를 전부 구함 (예: ["회차지점B", "창고"])
    route = resolve_travel_route(to_location, rules)
    # route의 맨 앞이 이미 지금 있는 곳(from_location)과 같다면 중복이니 제거
    # (예: 이미 회차지점B에 있는데 창고로 갈 때, "회차지점B, 창고"가 아니라 "창고"만 남겨야 함)
    if route and route[0] == from_location:
        route = route[1:]
    return route or [to_location]


if __name__ == "__main__":
    # 간단한 자체 확인
    rules = {"항구": "회차지점A", "창고": "회차지점B"}

    print("항구로 이동:", resolve_travel_route("항구", rules))
    print("창고로 이동:", resolve_travel_route("창고", rules))
    print("대기장소로 이동 (규칙 없음):", resolve_travel_route("대기장소", rules))
    print("대기장소 -> 항구 구간 확장:", expand_leg("대기장소", "항구", rules))
    print("회차지점A -> 항구 구간 확장 (이미 경유지에 있음):", expand_leg("회차지점A", "항구", rules))
