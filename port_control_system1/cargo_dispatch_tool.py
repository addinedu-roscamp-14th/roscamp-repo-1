"""
cargo_dispatch_tool.py

화물이 지금 어느 등록된 위치에 있는지 기록해두는 도구입니다.
  1) 화물 위치를 수동/엑셀 일괄 등록
  2) 위치가 바뀐 화물이 담긴 엑셀을 불러오면, 예전 위치와 다른 화물만 골라
     [대기장소 -> 픽업 -> 새 위치] 경로를 자동 생성해서 큐로 순서대로 실행 (데모)
  3) 완료되면 화물의 위치를 목적지로 갱신

자연어 명령("화물A를 항구로 옮겨")으로 배차하는 기능은 command_center.py(우측 하단 "🗣️ 명령"
버튼 팝업)로 옮겼습니다 - 이 파일의 build_route/RouteStep/load_cargo_registry 등을 그대로
가져다 씁니다.

location_marks_verified.json (또는 location_marks.json)에 이미 등록해두신 위치들을
그대로 재사용합니다. 실차 연동 시에는 파일 하단의 CargoDispatcherROS2 참고 코드로
Nav2 목표 전송 + 적재/하역 확인 신호 대기를 연결하시면 됩니다.
"""

import json
import time
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog
from typing import Dict, List, Optional, Tuple

import psycopg2
import customtkinter as ctk
from openpyxl import load_workbook

from waypoint_rules import expand_leg, load_waypoint_rules
from generate_cargo_template import build_template as build_cargo_excel_template


# 실행할 때의 현재 폴더(cwd)가 아니라, 이 파일이 실제로 있는 폴더를 기준으로 삼습니다.
# 예전에는 "cargo_locations.json"처럼 상대경로로 열고 저장했는데, 이러면 파이썬을
# 어느 폴더에서 실행하느냐에 따라 실제로 읽고 쓰는 파일이 달라지는 문제가 있었습니다
# (예: Downloads에서 실행할 때와 프로젝트 폴더에서 실행할 때 서로 다른 파일을 봄).
# 이제는 항상 이 스크립트가 있는 폴더의 파일을 쓰도록 고정합니다.
_APP_DIR = Path(__file__).resolve().parent

VERIFIED_LOCATIONS_FILE = str(_APP_DIR / "location_marks_verified.json")
SIMPLE_LOCATIONS_FILE = str(_APP_DIR / "location_marks.json")
CARGO_FILE = str(_APP_DIR / "cargo_locations.json")
STANDBY_LOCATION = "대기장소"  # 등록된 개별 대기 위치가 없을 때 쓰는 예비 기본값

# ---- 차량 대수/위치 추적 (자연어 명령 팝업과 엑셀 재배치가 공용으로 씁니다) ----
NUM_VEHICLES = 2  # 보유 차량 대수 - 늘리거나 줄이려면 이 값만 바꾸면 됩니다.

# 차량마다 자기만의 "집(대기 위치)"이 따로 있는 경우 여기에 순서대로 적어주세요.
# 예: 차량1은 "대기장소 1"에서, 차량2는 "대기장소 2"에서 대기 - 실제 등록된 위치명과
# 글자까지 정확히 같아야 합니다. 이 목록이 NUM_VEHICLES보다 짧으면, 모자란 차량은
# STANDBY_LOCATION(공용 기본값)을 씁니다.
VEHICLE_HOME_LOCATIONS = ["대기장소 1", "대기장소 2"]

VEHICLE_STATE_FILE = str(_APP_DIR / "vehicle_positions.json")


def vehicle_home_location(vehicle_idx: int) -> str:
    """vehicle_idx번 차량의 기본 대기 위치 이름을 돌려줍니다."""
    if 0 <= vehicle_idx < len(VEHICLE_HOME_LOCATIONS):
        return VEHICLE_HOME_LOCATIONS[vehicle_idx]
    return STANDBY_LOCATION


def load_vehicle_state(num_vehicles: int = NUM_VEHICLES) -> Tuple[List[str], int]:
    """차량별 현재 위치와, 다음에 배차할 차량 순번을 불러옵니다.
    처음 실행하거나 파일이 없으면 각 차량이 자기 대기 위치(VEHICLE_HOME_LOCATIONS)에
    있는 것으로 시작합니다."""
    positions: List[str] = []
    next_index = 0

    if Path(VEHICLE_STATE_FILE).exists():
        try:
            data = json.loads(Path(VEHICLE_STATE_FILE).read_text(encoding="utf-8"))
            positions = list(data.get("positions", []))
            next_index = int(data.get("next_index", 0))
        except Exception:
            positions, next_index = [], 0

    if len(positions) < num_vehicles:
        for i in range(len(positions), num_vehicles):
            positions.append(vehicle_home_location(i))
    positions = positions[:num_vehicles]
    next_index = next_index % num_vehicles

    return positions, next_index


def save_vehicle_state(positions: List[str], next_index: int) -> None:
    Path(VEHICLE_STATE_FILE).write_text(
        json.dumps({"positions": positions, "next_index": next_index}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


# ---- 차량/로봇팔 상태 추적 (배터리, 이상 유무, 최근 작업 이력) ----
# 참고: 실제 센서 연동 전이라 배터리/이상 유무는 자동으로 갱신되지 않고, 기본값에서
# 시작해서 화면의 버튼으로 직접 표시를 바꾸는 방식입니다 (수동 관리).
# "마지막 작업"만 화물 이동을 실행할 때마다 자동으로 기록됩니다.
VEHICLE_STATUS_FILE = str(_APP_DIR / "vehicle_status.json")
ROBOT_ARM_STATUS_FILE = str(_APP_DIR / "robot_arm_status.json")


def _default_status() -> Dict:
    return {"battery_pct": 100, "fault": None, "last_job": None, "last_job_time": None}


def load_vehicle_status(num_vehicles: int = NUM_VEHICLES) -> Dict[str, Dict]:
    """차량별 상태를 불러옵니다. 파일이 없거나 새로 늘어난 차량은 기본값으로 채웁니다."""
    data = {}
    if Path(VEHICLE_STATUS_FILE).exists():
        try:
            data = json.loads(Path(VEHICLE_STATUS_FILE).read_text(encoding="utf-8"))
        except Exception:
            data = {}
    result = {}
    for i in range(num_vehicles):
        key = "Yellow" if i == 0 else ("Blue" if i == 1 else f"차량 {i + 1}")
        result[key] = {**_default_status(), **data.get(key, {})}
    return result


def save_vehicle_status(status: Dict[str, Dict]) -> None:
    Path(VEHICLE_STATUS_FILE).write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding="utf-8")


def record_vehicle_job(vehicle_idx: int, description: str) -> None:
    """화물 이동이 끝날 때마다 호출해서 그 차량의 "마지막 작업" 기록을 남깁니다."""
    status = load_vehicle_status()
    key = "Yellow" if vehicle_idx == 0 else ("Blue" if vehicle_idx == 1 else f"차량 {vehicle_idx + 1}")
    if key not in status:
        status[key] = _default_status()
    status[key]["last_job"] = description
    status[key]["last_job_time"] = time.strftime("%H:%M:%S")
    save_vehicle_status(status)


def load_robot_arm_status(names: List[str]) -> Dict[str, Dict]:
    """등록된 로봇팔 이름들의 상태를 불러옵니다. 새로 등록된 로봇팔은 기본값으로 채웁니다."""
    data = {}
    if Path(ROBOT_ARM_STATUS_FILE).exists():
        try:
            data = json.loads(Path(ROBOT_ARM_STATUS_FILE).read_text(encoding="utf-8"))
        except Exception:
            data = {}
    result = {}
    for name in names:
        result[name] = {**_default_status(), **data.get(name, {})}
    return result


def save_robot_arm_status(status: Dict[str, Dict]) -> None:
    Path(ROBOT_ARM_STATUS_FILE).write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding="utf-8")


def build_return_to_standby_route(
    current_location: str,
    vehicle_idx: int = 0,
    waypoint_rules: Optional[Dict[str, str]] = None,
) -> List["RouteStep"]:
    """current_location에서 이 차량(vehicle_idx)의 자기 대기 위치로 복귀하는 경로를 만듭니다
    (필수 경유지 규칙 반영). 이미 그 위치에 있으면 빈 리스트를 반환합니다."""
    home = vehicle_home_location(vehicle_idx)
    if current_location == home:
        return []
    if waypoint_rules is None:
        waypoint_rules = load_waypoint_rules()

    steps = [RouteStep(current_location, "출발")]  # 실제 출발 위치를 첫 단계로 명시 (build_route와 동일한 패턴)
    for name in expand_leg(current_location, home, waypoint_rules):
        action = "복귀" if name == home else "경유"
        steps.append(RouteStep(name, action))
    return steps


def describe_route_sentence(item_label: str, route: List["RouteStep"]) -> str:
    """경로를 "OO에서 출발해 XX를 거쳐 YY로 이동 완료" 형태의 문장 하나로 요약합니다."""
    if not route:
        return f"{item_label}: 이동할 경로가 없습니다."

    start = route[0].location
    end = route[-1].location
    via_names = [step.location for step in route[1:-1]]

    if via_names:
        via_text = "', '".join(via_names)
        return f"{item_label}: '{start}'에서 출발해 '{via_text}'를 거쳐 '{end}'로 이동 완료"
    return f"{item_label}: '{start}'에서 출발해 '{end}'로 이동 완료"


# -----------------------------------------------------------------------------
# 위치 데이터 로딩 (dual_view_calibrator.py / location_marker_tool.py 결과물 재사용)
# -----------------------------------------------------------------------------

def load_named_locations() -> Dict[str, Dict]:
    """
    두 종류의 저장 형식을 모두 지원합니다.
    - location_marks_verified.json: {"항구": {"cctv_pixel": [..], "map_meters": [..]}}
    - location_marks.json (구버전): {"항구": [px, py]}
    반환값은 항상 {"항구": {"cctv_pixel": [...], "map_meters": [...] (있으면)}} 형태로 통일합니다.
    """
    if Path(VERIFIED_LOCATIONS_FILE).exists():
        # 신버전 파일은 이미 원하는 형태(dict of dict)라서 그대로 반환
        raw = json.loads(Path(VERIFIED_LOCATIONS_FILE).read_text(encoding="utf-8"))
        return raw

    if Path(SIMPLE_LOCATIONS_FILE).exists():
        # 구버전 파일은 {"항구": [px, py]} 형태라서, 신버전과 같은 모양으로 감싸줌
        raw = json.loads(Path(SIMPLE_LOCATIONS_FILE).read_text(encoding="utf-8"))
        return {name: {"cctv_pixel": xy} for name, xy in raw.items()}

    return {}


# -----------------------------------------------------------------------------
# 화물 위치 등록부
# -----------------------------------------------------------------------------

CARGO_DETAILS_FILE = str(_APP_DIR / "cargo_details.json")
EXCEL_DATA_START_ROW = 6  # generate_cargo_template.py의 DATA_START_ROW와 일치해야 함


def _get_db_conn():
    return psycopg2.connect(
        host="localhost",
        database="port_db",
        user="postgres",
        password="1234",
        port="5432"
    )

def load_cargo_registry() -> Dict[str, str]:
    registry = {}
    try:
        conn = _get_db_conn()
        cur = conn.cursor()
        cur.execute("SELECT name, location FROM cargos")
        rows = cur.fetchall()
        for name, location in rows:
            registry[name] = location
        cur.close()
        conn.close()
    except Exception as e:
        print(f"DB Load Error (Registry): {e}")
    return registry


def save_cargo_registry(registry: Dict[str, str]) -> None:
    try:
        conn = _get_db_conn()
        cur = conn.cursor()
        
        cur.execute("SELECT name FROM cargos")
        db_names = {row[0] for row in cur.fetchall()}
        
        to_delete = db_names - set(registry.keys())
        for name in to_delete:
            cur.execute("DELETE FROM cargos WHERE name = %s", (name,))
            
        for name, location in registry.items():
            cur.execute("""
                INSERT INTO cargos (name, location)
                VALUES (%s, %s)
                ON CONFLICT (name) DO UPDATE SET
                location = EXCLUDED.location
            """, (name, location))
            
        conn.commit()
        cur.close()
        conn.close()
    except Exception as e:
        print(f"DB Save Error (Registry): {e}")


def load_cargo_details() -> Dict[str, Dict[str, str]]:
    details = {}
    try:
        conn = _get_db_conn()
        cur = conn.cursor()
        cur.execute("SELECT name, container_id, cargo_type, note, base_aruco_id, floor FROM cargos")
        rows = cur.fetchall()
        for name, container_id, cargo_type, note, base_aruco_id, floor in rows:
            details[name] = {
                "컨테이너ID": container_id or "",
                "화물종류": cargo_type or "",
                "비고": note or "",
                "기반ArUco": base_aruco_id or "",
                "층수": str(floor) if floor is not None else "1"
            }
        cur.close()
        conn.close()
    except Exception as e:
        print(f"DB Load Error (Details): {e}")
    return details


def save_cargo_details(details: Dict[str, Dict[str, str]]) -> None:
    try:
        conn = _get_db_conn()
        cur = conn.cursor()
        for name, detail in details.items():
            container_id = detail.get("컨테이너ID", "")
            cargo_type = detail.get("화물종류", "")
            note = detail.get("비고", "")
            base_aruco_id = detail.get("기반ArUco", "")
            floor_str = detail.get("층수", "1")
            floor = int(floor_str) if floor_str.isdigit() else 1
            
            cur.execute("""
                UPDATE cargos SET 
                container_id = %s,
                cargo_type = %s,
                note = %s,
                base_aruco_id = %s,
                floor = %s
                WHERE name = %s
            """, (container_id, cargo_type, note, base_aruco_id, floor, name))
        conn.commit()
        cur.close()
        conn.close()
    except Exception as e:
        print(f"DB Save Error (Details): {e}")


def bulk_import_cargo_from_excel(
    path: str,
    known_locations,
) -> Tuple[Dict[str, str], Dict[str, Dict[str, str]], List[str]]:
    """
    generate_cargo_template.py로 만든 양식을 읽어 (name->location, name->세부정보, 오류목록)을 반환합니다.
    known_locations에 없는 위치명이 있어도 등록은 하되, 오류 목록에 남겨서 사용자가 확인하게 합니다.
    """
    wb = load_workbook(path, data_only=True)  # data_only=True: 수식이 아니라 계산된 값만 읽음
    ws = wb["화물등록"] if "화물등록" in wb.sheetnames else wb.active  # 시트 이름이 바뀌었어도 안전하게 동작

    known_set = set(known_locations)  # in 검사를 빠르게 하려고 set으로 변환
    new_registry: Dict[str, str] = {}
    new_details: Dict[str, Dict[str, str]] = {}
    errors: List[str] = []

    # 헤더/예시 행을 건너뛰고 실제 데이터가 시작되는 6행부터 끝까지 한 줄씩 읽음
    for row_idx in range(EXCEL_DATA_START_ROW, ws.max_row + 1):
        name = ws.cell(row=row_idx, column=1).value          # A열: 화물명
        location = ws.cell(row=row_idx, column=2).value       # B열: 현재위치
        container_id = ws.cell(row=row_idx, column=3).value   # C열: 컨테이너/ArUco ID
        cargo_type = ws.cell(row=row_idx, column=4).value     # D열: 화물종류
        note = ws.cell(row=row_idx, column=5).value           # E열: 비고

        if name is None or str(name).strip() == "":
            continue  # 화물명이 비어있으면 빈 행으로 간주하고 건너뜀

        name = str(name).strip()
        location = str(location).strip() if location is not None else ""

        if not location:
            errors.append(f"{row_idx}행: '{name}'의 현재위치가 비어있어 건너뛰었습니다.")
            continue

        if known_set and location not in known_set:
            # 등록되지 않은 위치명이어도 일단 저장은 하되, 나중에 사용자가 확인할 수 있게 기록만 해둠
            errors.append(f"{row_idx}행: '{name}'의 위치 '{location}'는 등록된 위치 목록에 없습니다. (그대로 저장은 됨)")

        new_registry[name] = location
        new_details[name] = {
            "컨테이너ID": str(container_id).strip() if container_id else "",
            "화물종류": str(cargo_type).strip() if cargo_type else "",
            "비고": str(note).strip() if note else "",
        }

    return new_registry, new_details, errors


def plan_relocations_from_excel(
    path: str,
    old_registry: Dict[str, str],
    known_locations,
    vehicle_positions: Optional[List[str]] = None,
    waypoint_rules: Optional[Dict[str, str]] = None,
) -> Tuple[List[Dict], List[str], List[str], Dict[str, str], Dict[str, Dict[str, str]], List[str], int]:
    """
    엑셀(새 위치)과 기존 등록부(old_registry, 옛 위치)를 비교해서
    - jobs: 위치가 바뀐 화물들의 이동 경로 목록 [{"item":.., "route":[...], "container_id":..,
      "vehicle_index":.., "is_return":False/True, "label":..}] (맨 뒤에 차량 복귀 job도 포함)
    - unchanged: 위치 변경이 없는 화물명 목록
    - new_items: 이번에 처음 등록되는 화물명 목록 (이전 위치가 없어 경로 생성 대상 아님)
    - final_next_vehicle_index: 이번 배치에서 다 쓰고 난 뒤의 차량 순번(다음 배치에 이어서 씀)
    을 함께 반환합니다.

    차량 대수(NUM_VEHICLES)만큼은 대기장소에서 출발하고, 그 이후 화물부터는 직전에
    그 차량이 마지막으로 도착한 위치에서 출발하도록 순서대로 시뮬레이션합니다.
    배치 마지막에는 실제로 사용한 차량들을 대기장소로 복귀시키는 job도 추가됩니다.
    """
    if waypoint_rules is None:
        waypoint_rules = load_waypoint_rules()
    if vehicle_positions is None:
        vehicle_positions = [vehicle_home_location(i) for i in range(NUM_VEHICLES)]
    sim_positions = list(vehicle_positions)  # 원본을 훼손하지 않도록 복사본에서 시뮬레이션
    next_index = 0

    new_registry, new_details, errors = bulk_import_cargo_from_excel(path, known_locations)

    jobs: List[Dict] = []       # 위치가 바뀐 화물 -> 이동 경로가 필요한 것들
    unchanged: List[str] = []   # 예전과 위치가 같은 화물 -> 아무것도 안 해도 됨
    new_items: List[str] = []   # 등록부에 아예 없던 화물 -> "이동"이 아니라 "신규 등록" 대상
    used_vehicle_indices = set()

    for name, new_location in new_registry.items():
        old_location = old_registry.get(name)  # 기존 등록부에서 이 화물의 예전 위치 조회

        if old_location is None:
            new_items.append(name)  # 예전 기록이 없으니 이동이 아니라 신규 등록으로 처리
            continue

        if old_location == new_location:
            unchanged.append(name)  # 위치가 그대로면 재배차 대상 아님
            continue

        vehicle_idx = next_index % len(sim_positions)
        start_location = sim_positions[vehicle_idx]
        next_index += 1

        # 위치가 실제로 바뀐 경우에만 [차량 현재위치 -> 예전위치(픽업) -> 새위치(하역)] 경로를 생성
        # build_route는 원래 "화물등록부에서 위치를 조회"하는 함수인데, 여기서는 이미 old_location을
        # 알고 있으므로 {name: old_location} 라는 임시 1개짜리 등록부를 만들어 그대로 재사용합니다.
        route = build_route(name, new_location, {name: old_location},
                            standby_location=start_location, waypoint_rules=waypoint_rules)
        
        is_crane_only = any(step.action == "크레인 전용 이동" for step in route)
        if is_crane_only:
            next_index -= 1 # 배차 취소 (차량 이동 없음)
            jobs.append({
                "item": name,
                "route": route,
                "container_id": new_details.get(name, {}).get("컨테이너ID", ""),
                "vehicle_index": -1,
                "is_return": False,
                "label": name,
            })
            continue

        agv_final_loc = route[-1].location
        for step in reversed(route):
            if step.action not in ["크레인 최종 이동", "크레인 전용 이동"]:
                agv_final_loc = step.location
                break

        sim_positions[vehicle_idx] = agv_final_loc
        used_vehicle_indices.add(vehicle_idx)

        jobs.append({
            "item": name,
            "route": route,
            "container_id": new_details.get(name, {}).get("컨테이너ID", ""),
            "vehicle_index": vehicle_idx,
            "is_return": False,
            "label": name,
        })

    # 배치 마지막에 실제로 쓰인 차량들을 각자의 대기 위치로 복귀시키는 job을 큐 맨 뒤에 추가
    for vehicle_idx in sorted(used_vehicle_indices):
        current = sim_positions[vehicle_idx]
        if "창고 하역장" in current:
            continue  # 창고 하역장에 정차 유지

        return_route = build_return_to_standby_route(current, vehicle_idx, waypoint_rules)
        if not return_route:
            continue  # 이미 자기 대기 위치에 있음
        jobs.append({
            "item": None,
            "route": return_route,
            "container_id": "",
            "vehicle_index": vehicle_idx,
            "is_return": True,
            "label": f"{'Yellow' if vehicle_idx == 0 else ('Blue' if vehicle_idx == 1 else f'차량 {vehicle_idx + 1}')} 복귀",
        })
        sim_positions[vehicle_idx] = vehicle_home_location(vehicle_idx)

    return jobs, unchanged, new_items, new_registry, new_details, errors, next_index % len(sim_positions)

    return jobs, unchanged, new_items, new_registry, new_details, errors


# -----------------------------------------------------------------------------
# 경로 계획
# -----------------------------------------------------------------------------

class RouteStep:
    def __init__(self, location: str, action: str):
        self.location = location  # 위치 이름
        self.action = action      # "출발" / "적재" / "하역"

    def __repr__(self):
        return f"{self.location}({self.action})"


def build_route(
    item_name: str,
    destination: str,
    cargo_registry: Dict[str, str],
    standby_location: str = STANDBY_LOCATION,
    waypoint_rules: Optional[Dict[str, str]] = None,
) -> List[RouteStep]:
    """
    화물 등록부에서 item_name의 현재 위치(픽업 지점)를 찾아
    [대기장소 -> (필수 경유지 있으면 경유) -> 픽업 -> (필수 경유지 있으면 경유) -> 목적지]
    순서의 경로를 만듭니다. 화물이 이미 목적지에 있으면 픽업 단계 없이 안내만 합니다.
    """
    if item_name not in cargo_registry:
        raise ValueError(f"'{item_name}'의 현재 위치를 알 수 없습니다. 먼저 화물 위치를 등록해주세요.")

    if waypoint_rules is None:
        waypoint_rules = load_waypoint_rules()

    pickup = cargo_registry[item_name]  # 이 화물이 지금 실제로 놓여있는 위치

    if pickup == destination:
        # 목적지가 같으면(층수 변경, 구역 내 정리 등) 크레인(로봇팔) 전용 이동
        return [
            RouteStep(pickup, "크레인 상차"),
            RouteStep(destination, "크레인 전용 이동")
        ]

    def _is_warehouse(loc: str) -> bool:
        return loc in ["A-1-1", "A-1-2", "A-2-1", "A-2-2", "A-3-1", "A-3-2"] or ("창고" in loc and "하역장" not in loc)

    is_pickup_warehouse = _is_warehouse(pickup)
    is_pickup_port = "항구" in pickup and "하역장" not in pickup
    is_dest_warehouse = _is_warehouse(destination)
    is_dest_port = "항구" in destination and "하역장" not in destination

    # 1) 창고 -> 창고 (크레인 전용 이동)
    if is_pickup_warehouse and is_dest_warehouse:
        return [
            RouteStep(pickup, "크레인 상차"),
            RouteStep(destination, "크레인 전용 이동")
        ]
        
    # 2) 항구 -> 항구 (배 내부 이동, 크레인 전용 이동)
    if is_pickup_port and is_dest_port:
        return [
            RouteStep(pickup, "크레인 상차"),
            RouteStep(destination, "크레인 전용 이동")
        ]

    agv_pickup = pickup
    agv_destination = destination

    # 2) 창고 -> 항구
    if is_pickup_warehouse and is_dest_port:
        agv_pickup = "창고 하역장"
        agv_destination = "항구 하역장"
    # 3) 항구 -> 창고
    elif is_pickup_port and is_dest_warehouse:
        agv_pickup = "항구 하역장"
        agv_destination = "창고 하역장"

    steps = [RouteStep(standby_location, "출발")]  # 차량은 항상 대기장소에서 출발
    current = standby_location  # 지금까지 경로상 마지막으로 도달한 위치 (다음 구간 계산의 출발점)

    if agv_pickup != standby_location:
        # [대기장소 -> 픽업] 구간에 필수 경유지가 있으면 expand_leg가 자동으로 끼워줌
        # 예: expand_leg("대기장소", "창고 하역장", rules) -> ["회차지점B", "창고 하역장"]
        for name in expand_leg(current, agv_pickup, waypoint_rules):
            action = "적재 (크레인 연계)" if name == agv_pickup else "경유"
            steps.append(RouteStep(name, action))
        current = agv_pickup  # 픽업까지 왔으니, 다음 구간은 여기서부터 계산

    # [픽업 -> 목적지] 구간도 마찬가지로 필수 경유지를 자동으로 끼워 넣음
    for name in expand_leg(current, agv_destination, waypoint_rules):
        action = "하역 (크레인 연계)" if name == agv_destination else "경유"
        steps.append(RouteStep(name, action))

    # AGV 이동 후, 화물이 최종 목적지로 크레인을 통해 이동함을 명시
    if agv_destination != destination:
        steps.append(RouteStep(destination, "크레인 최종 이동"))

    return steps


def location_coord_text(name: str, locations: Dict[str, Dict]) -> str:
    entry = locations.get(name)
    if not entry:
        return f"{name} (좌표 미등록)"
    if "map_meters" in entry:
        mx, my = entry["map_meters"]
        return f"{name} (map {mx:.2f}, {my:.2f} m)"
    if "cctv_pixel" in entry:
        px, py = entry["cctv_pixel"]
        return f"{name} (CCTV px {px:.0f}, {py:.0f})"
    return name


# -----------------------------------------------------------------------------
# GUI
# -----------------------------------------------------------------------------

def write_cargo_excel(cargo_registry: Dict[str, str], cargo_details: Dict[str, Dict[str, str]], output_path: str) -> None:
    """현재 화물 위치/세부정보를 엑셀로 내보냅니다. generate_cargo_template.py와 같은
    형식(화물등록 시트, 6행부터 데이터)으로 저장하기 때문에, 이 파일을 그대로
    "엑셀로 일괄 등록" 또는 "재배치 엑셀 불러오기"로 다시 불러올 수 있습니다."""
    from openpyxl import Workbook
    from openpyxl.styles import Alignment, Font, PatternFill

    wb = Workbook()
    ws = wb.active
    ws.title = "화물등록"

    ws.merge_cells("A1:E1")
    ws["A1"] = "화물 위치 내보내기 (현재 상태 스냅샷)"
    ws["A1"].font = Font(name="Arial", size=14, bold=True)

    ws.merge_cells("A2:E2")
    ws["A2"] = (
        f"내보낸 시각: {time.strftime('%Y-%m-%d %H:%M:%S')} - "
        "이 파일은 '엑셀로 일괄 등록' 또는 '재배치 엑셀 불러오기'로 다시 불러올 수 있습니다."
    )
    ws["A2"].font = Font(name="Arial", size=10, italic=True, color="808080")

    headers = ["화물명", "현재위치", "컨테이너/ArUco ID", "화물종류", "비고"]
    header_fill = PatternFill(start_color="1F538D", end_color="1F538D", fill_type="solid")
    for col_idx, title in enumerate(headers, start=1):
        cell = ws.cell(row=4, column=col_idx, value=title)
        cell.font = Font(name="Arial", size=11, bold=True, color="FFFFFF")
        cell.fill = header_fill
        cell.alignment = Alignment(horizontal="center")

    widths = [16, 16, 18, 14, 30]
    for col_idx, width in enumerate(widths, start=1):
        ws.column_dimensions[chr(64 + col_idx)].width = width

    row_idx = EXCEL_DATA_START_ROW
    for name, location in cargo_registry.items():
        detail = cargo_details.get(name, {})
        ws.cell(row=row_idx, column=1, value=name)
        ws.cell(row=row_idx, column=2, value=location)
        ws.cell(row=row_idx, column=3, value=detail.get("컨테이너ID", ""))
        ws.cell(row=row_idx, column=4, value=detail.get("화물종류", ""))
        ws.cell(row=row_idx, column=5, value=detail.get("비고", ""))
        row_idx += 1

    wb.save(output_path)


class CargoDispatchTool(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=20, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=14, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=12)

        self.locations = load_named_locations()       # 등록된 위치 이름과 좌표들
        self.cargo_registry = load_cargo_registry()   # {화물명: 현재위치} - 실제 배차 로직이 참조하는 핵심 데이터
        self.cargo_details = load_cargo_details()     # {화물명: {컨테이너ID, 화물종류, 비고}} - 부가정보
        self.rules = load_waypoint_rules()            # 필수 경유지 규칙 (build_route에 전달)
        self.vehicle_positions, self.next_vehicle_index = load_vehicle_state()  # 차량별 현재 위치

        # 배차 실행 큐 상태 - 자연어 단건 명령이든 엑셀 다건 재배치든 이 큐 하나로 통일해서 처리
        self.dispatch_queue: List[Dict] = []  # [{"item": 화물명, "route": [RouteStep, ...]}, ...]
        self.active_job_index: int = 0        # 큐에서 지금 처리 중인 화물(job)의 인덱스
        self.active_step_index: int = 0       # 그 화물의 경로 중 지금 처리 중인 단계(step)의 인덱스

        self._build_ui()
        self._refresh_cargo_list()
        
        # UDP 리스너 초기화 (arm2 등의 상태 패킷 수신)
        try:
            from udp_listener import RobotStatusListener
            self.udp_listener = RobotStatusListener(port=15002, callback=self._on_robot_status)
            self.udp_listener.start()
            
            # 툴 종료 시 리스너 스레드 정리를 위해 바인딩
            def on_destroy(event):
                if event.widget == self:
                    self.udp_listener.stop()
            self.bind("<Destroy>", on_destroy, add="+")
        except ImportError:
            self.udp_listener = None
            print("Warning: udp_listener module not found. Robot status monitoring disabled.")

        self.after(1000, self._auto_refresh_loop)

    def _auto_refresh_loop(self) -> None:
        if not self.winfo_exists():
            return
        try:
            new_registry = load_cargo_registry()
            new_details = load_cargo_details()
            if new_registry != self.cargo_registry or new_details != self.cargo_details:
                self.cargo_registry = new_registry
                self.cargo_details = new_details
                self._refresh_cargo_list()
        except Exception:
            pass
        self.after(1000, self._auto_refresh_loop)

    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=5)
        self.grid_columnconfigure(1, weight=7)
        self.grid_rowconfigure(1, weight=1)
        self.configure(fg_color="#242424")

        # Header
        header_frame = ctk.CTkFrame(self, height=56, fg_color="#2b2b2b", corner_radius=0, border_width=1, border_color="#1a1a1a")
        header_frame.grid(row=0, column=0, columnspan=2, sticky="ew")
        header_frame.pack_propagate(False)
        ctk.CTkLabel(header_frame, text="🚚 화물 위치 추적 및 자연어 배차", font=self.font_title, text_color="#dce4ee").pack(side="left", padx=24, pady=16)

        # ---- Left Panel: Cargo Registration & List ----
        left = ctk.CTkFrame(self, fg_color="transparent")
        left.grid(row=1, column=0, sticky="nsew", padx=(24, 12), pady=24)
        left.grid_columnconfigure(0, weight=1)
        left.grid_rowconfigure(2, weight=1)

        # 1) Registration Form Card
        reg_card = ctk.CTkFrame(left, fg_color="#2b2b2b", corner_radius=6, border_width=1, border_color="#1a1a1a")
        reg_card.grid(row=0, column=0, sticky="ew", pady=(0, 24))
        
        ctk.CTkLabel(reg_card, text="화물 위치 등록", font=self.font_subtitle, text_color="#dce4ee").pack(anchor="w", padx=20, pady=(20, 16))

        form = ctk.CTkFrame(reg_card, fg_color="transparent")
        form.pack(fill="x", padx=20, pady=(0, 20))
        form.grid_columnconfigure(0, weight=1)

        self.cargo_name_var = ctk.StringVar()
        entry_kwargs = {"fg_color": "#343638", "border_color": "#565b5e", "text_color": "#dce4ee", "corner_radius": 6, "height": 36}
        ctk.CTkEntry(form, textvariable=self.cargo_name_var, placeholder_text="화물 이름 (예: 화물A)", **entry_kwargs).grid(row=0, column=0, sticky="ew", pady=(0, 16))

        self.location_option_var = ctk.StringVar()
        location_names = list(self.locations.keys()) or ["대기장소"]
        
        loc_row = ctk.CTkFrame(form, fg_color="transparent")
        loc_row.grid(row=1, column=0, sticky="ew", pady=(0, 16))
        loc_row.grid_columnconfigure(0, weight=1)
        
        self.location_menu = ctk.CTkOptionMenu(loc_row, variable=self.location_option_var, values=location_names,
                                              fg_color="#343638", button_color="#565b5e", button_hover_color="#2e86c1", corner_radius=6, height=36)
        self.location_menu.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        
        self.floor_var = ctk.StringVar(value="1")
        ctk.CTkEntry(loc_row, textvariable=self.floor_var, placeholder_text="층수", width=60, **entry_kwargs).grid(row=0, column=1)

        btn_kwargs = {"corner_radius": 6, "height": 36, "font": self.font_body}
        
        ctk.CTkButton(form, text="위치 등록 / 갱신", fg_color="#2e86c1", hover_color="#21618c", command=self.register_cargo_location, **btn_kwargs).grid(row=2, column=0, sticky="ew", pady=4)
        
        # Detail Banner
        self.detail_banner = ctk.CTkFrame(form, fg_color="#343638", corner_radius=6, border_width=1, border_color="#2e86c1")
        ctk.CTkLabel(self.detail_banner, text="📋 추가 정보 입력 (선택사항)", font=self.font_subtitle, text_color="#2e86c1").grid(row=0, column=0, columnspan=4, sticky="w", padx=10, pady=10)
        
        self.aruco_var = ctk.StringVar()
        ctk.CTkLabel(self.detail_banner, text="내 ArUco ID:", font=self.font_body).grid(row=1, column=0, sticky="w", padx=10, pady=2)
        ctk.CTkEntry(self.detail_banner, textvariable=self.aruco_var, placeholder_text="예: ARUCO_42", width=120, **entry_kwargs).grid(row=1, column=1, sticky="ew", padx=(0, 10), pady=2)
        
        self.base_aruco_var = ctk.StringVar()
        ctk.CTkLabel(self.detail_banner, text="바닥(Base) ArUco:", font=self.font_body).grid(row=1, column=2, sticky="w", padx=10, pady=2)
        ctk.CTkEntry(self.detail_banner, textvariable=self.base_aruco_var, placeholder_text="어느 ArUco 위에?", width=120, **entry_kwargs).grid(row=1, column=3, sticky="ew", padx=(0, 10), pady=2)

        self.cargo_type_var = ctk.StringVar()
        ctk.CTkLabel(self.detail_banner, text="화물 종류:", font=self.font_body).grid(row=2, column=0, sticky="w", padx=10, pady=2)
        ctk.CTkEntry(self.detail_banner, textvariable=self.cargo_type_var, placeholder_text="예: 컨테이너, 벌크", width=120, **entry_kwargs).grid(row=2, column=1, sticky="ew", padx=(0, 10), pady=2)
        
        # 층수(floor_var)는 메인 폼으로 이동됨

        self.cargo_note_var = ctk.StringVar()
        ctk.CTkLabel(self.detail_banner, text="비고:", font=self.font_body).grid(row=3, column=0, sticky="w", padx=10, pady=2)
        ctk.CTkEntry(self.detail_banner, textvariable=self.cargo_note_var, placeholder_text="메모", width=120, **entry_kwargs).grid(row=3, column=1, columnspan=3, sticky="ew", padx=(0, 10), pady=2)
        
        banner_btn_row = ctk.CTkFrame(self.detail_banner, fg_color="transparent")
        banner_btn_row.grid(row=4, column=0, columnspan=4, sticky="ew", padx=10, pady=10)
        ctk.CTkButton(banner_btn_row, text="✅ 저장", width=80, fg_color="#27ae60", hover_color="#1e8449", command=self._confirm_register, **btn_kwargs).pack(side="left", padx=(0, 6))
        ctk.CTkButton(banner_btn_row, text="건너뛰기", width=80, fg_color="#565b5e", hover_color="#333333", command=self._skip_details_register, **btn_kwargs).pack(side="left", padx=(0, 6))
        ctk.CTkButton(banner_btn_row, text="취소", width=60, fg_color="#c0392b", hover_color="#922b21", command=self._cancel_register, **btn_kwargs).pack(side="left")

        self._pending_cargo_name: Optional[str] = None
        self._pending_cargo_location: Optional[str] = None

        ctk.CTkButton(form, text="📄 엑셀로 일괄 등록", fg_color="#27ae60", hover_color="#1e8449", command=self.bulk_import_from_excel, **btn_kwargs).grid(row=4, column=0, sticky="ew", pady=4)
        ctk.CTkButton(form, text="🧾 빈 엑셀 양식 생성", fg_color="#343638", hover_color="#333333", border_width=1, border_color="#565b5e", text_color="#dce4ee", command=self.generate_template, **btn_kwargs).grid(row=5, column=0, sticky="ew", pady=4)
        ctk.CTkButton(form, text="📤 현재 화물정보 엑셀로 내보내기", fg_color="#f39c12", hover_color="#d68910", command=self.export_cargo_to_excel, **btn_kwargs).grid(row=6, column=0, sticky="ew", pady=4)

        # 2) Registered Cargo List Card
        list_card = ctk.CTkFrame(left, fg_color="#2b2b2b", corner_radius=6, border_width=1, border_color="#1a1a1a")
        list_card.grid(row=2, column=0, sticky="nsew")
        list_card.grid_columnconfigure(0, weight=1)
        list_card.grid_rowconfigure(1, weight=1)

        list_header = ctk.CTkFrame(list_card, fg_color="#2b2b2b", height=52, corner_radius=6)
        list_header.grid(row=0, column=0, sticky="ew")
        list_header.pack_propagate(False)
        ctk.CTkLabel(list_header, text="등록된 화물 목록", font=self.font_subtitle, text_color="#dce4ee").pack(side="left", padx=16, pady=16)
        ctk.CTkButton(list_header, text="🔄 새로고침", width=80, fg_color="transparent", text_color="#2e86c1", hover_color="#333333", command=self.refresh_from_disk).pack(side="right", padx=16, pady=16)

        ctk.CTkFrame(list_card, height=1, fg_color="#1a1a1a").grid(row=0, column=0, sticky="sew")

        self.cargo_rows_container = ctk.CTkScrollableFrame(list_card, fg_color="#242424", border_width=1, border_color="#1a1a1a", corner_radius=6)
        self.cargo_rows_container.grid(row=1, column=0, sticky="nsew", padx=16, pady=16)
        self.cargo_rows_container.grid_columnconfigure(0, weight=1)

        # ---- Right Panel: Batch Redispatch & Log ----
        right = ctk.CTkFrame(self, fg_color="#2b2b2b", corner_radius=6, border_width=1, border_color="#1a1a1a")
        right.grid(row=1, column=1, sticky="nsew", padx=(12, 24), pady=24)
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)

        # Top Section (Controls)
        top_sec = ctk.CTkFrame(right, fg_color="transparent")
        top_sec.grid(row=0, column=0, sticky="ew", padx=24, pady=24)

        ctk.CTkLabel(top_sec, text="🔄 엑셀 기반 일괄 재배치", font=self.font_subtitle, text_color="#dce4ee").pack(anchor="w", pady=(0, 8))
        self.relocation_label = ctk.CTkLabel(top_sec, text="바뀐 위치가 담긴 엑셀을 불러오면, 이전 위치와 다른 화물만 골라 이동 경로를 한번에 계획합니다.", font=self.font_body, text_color="#9e9e9e", wraplength=650, justify="left")
        self.relocation_label.pack(anchor="w", pady=(0, 20))

        reloc_row = ctk.CTkFrame(top_sec, fg_color="transparent")
        reloc_row.pack(fill="x", pady=(0, 24))
        ctk.CTkButton(reloc_row, text="📄 재배치 엑셀 불러오기", fg_color="#343638", border_width=1, border_color="#565b5e", text_color="#dce4ee", hover_color="#333333", command=self.import_relocation_excel, **btn_kwargs).pack(side="left", padx=(0, 16))
        
        ctk.CTkFrame(top_sec, height=1, fg_color="#1a1a1a").pack(fill="x", pady=(0, 20))

        action_row = ctk.CTkFrame(top_sec, fg_color="transparent")
        action_row.pack(fill="x")
        self.step_button = ctk.CTkButton(action_row, text="▶ 배차 실행 (데모 시작)", command=self.advance_queue, state="disabled", fg_color="#2e86c1", hover_color="#21618c", **btn_kwargs)
        self.step_button.pack(side="left")
        self.step_status_label = ctk.CTkLabel(action_row, text="", font=self.font_body, text_color="#9e9e9e")
        self.step_status_label.pack(side="left", padx=16)

        # Bottom Section (Execution Log)
        log_sec = ctk.CTkFrame(right, fg_color="transparent")
        log_sec.grid(row=1, column=0, sticky="nsew", padx=24, pady=(0, 24))
        log_sec.grid_columnconfigure(0, weight=1)
        log_sec.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(log_sec, text="진행 로그 (Execution Log)", font=("Consolas", 12, "bold"), text_color="#9e9e9e").grid(row=0, column=0, sticky="w", pady=(0, 12))
        self.log_box = ctk.CTkTextbox(log_sec, font=("Consolas", 13), fg_color="#1e1e1e", border_width=1, border_color="#1a1a1a", text_color="#dce4ee")
        self.log_box.grid(row=1, column=0, sticky="nsew")
        self.log_box.configure(state="disabled")

        # Command Button at bottom right of the main frame
        bottom_row = ctk.CTkFrame(self, fg_color="transparent")
        bottom_row.place(relx=1.0, rely=1.0, anchor="se", x=-24, y=-24)
        ctk.CTkButton(bottom_row, text="🗣️ 명령", font=self.font_subtitle, fg_color="#92ccff", text_color="#003351",
                      hover_color="#cce5ff", height=40, width=100, command=self.open_command_popup).pack(side="right")

    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        # 팝업에서 화물 위치를 바꾸면, 지금 이 화면(이미 메모리에 옛 데이터를 들고 있음)이
        # 자동으로 최신 상태로 갱신되도록 콜백을 넘겨줍니다.
        open_command_popup(self, on_cargo_updated=self.refresh_from_disk)

    def refresh_from_disk(self) -> None:
        """위치/화물 데이터를 디스크에서 다시 읽어와 화면을 최신 상태로 갱신합니다.
        팝업에서 명령을 실행한 직후 자동으로 호출되며, "🔄 새로고침" 버튼으로 수동으로도
        호출할 수 있습니다 (예: 다른 창에서 엑셀로 데이터를 바꾼 경우 등)."""
        self.locations = load_named_locations()
        self.cargo_registry = load_cargo_registry()
        self.cargo_details = load_cargo_details()
        self._refresh_cargo_list()

    # ------------------------------------------------------------------
    # 화물 위치 등록
    # ------------------------------------------------------------------
    def register_cargo_location(self) -> None:
        """화물명과 위치를 확인한 뒤, 추가 정보(ArUco, 화물종류, 비고)를 입력할 수 있는
        배너를 폼 아래에 표시합니다. 기존에 세부정보가 있으면 미리 채워줍니다."""
        # 권한 체크
        try:
            app = self.winfo_toplevel()
            if hasattr(app, "current_user_id") and app.current_user_id:
                role = app.USERS.get(app.current_user_id, {}).get("role", "")
                if role not in ["최고 관리자 (Admin)", "현장 관리자 (Manager)"]:
                    messagebox.showerror("접근 거부", "화물 등록 권한이 없습니다.\n(관리자 전용 기능)")
                    return
        except Exception:
            pass
        name = self.cargo_name_var.get().strip()
        location = self.location_option_var.get().strip()

        if not name:
            messagebox.showinfo("이름 필요", "화물 이름을 입력해주세요.")
            return
        if not location:
            messagebox.showinfo("위치 필요", "위치를 선택해주세요.")
            return

        # 이름/위치를 임시 저장하고, 기존 세부정보가 있으면 배너에 미리 채움
        self._pending_cargo_name = name
        self._pending_cargo_location = location

        existing = self.cargo_details.get(name, {})
        self.aruco_var.set(existing.get("컨테이너ID", ""))
        self.cargo_type_var.set(existing.get("화물종류", ""))
        self.cargo_note_var.set(existing.get("비고", ""))
        self.base_aruco_var.set(existing.get("기반ArUco", ""))
        
        # 층수는 메인 폼에서 입력된 값을 유지하되, 기존 세부정보가 있고 메인 폼이 1이면 덮어씌움
        if existing.get("층수") and self.floor_var.get() == "1":
            self.floor_var.set(existing.get("층수"))

        # 배너 표시 (row=3 - 등록 버튼 바로 아래)
        self.detail_banner.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(4, 4))

    def _on_robot_status(self, msg: dict) -> None:
        """UDP 패킷으로 로봇팔 상태가 수신되었을 때 호출됩니다 (백그라운드 스레드)."""
        robot = msg.get("로봇", "unknown")
        status = msg.get("상태", "")
        text = msg.get("메시지", "")
        
        # 로그 출력 (스레드 안전을 위해 after 사용)
        log_msg = f"[{robot} 상태] {status}: {text}"
        self.after(0, lambda: self._log(log_msg))
        
        # ---------------------------------------------------------
        # 안전장치 (실패 처리)
        # ---------------------------------------------------------
        if status == "실패":
            # 큐를 즉시 중단(초기화)합니다.
            self.dispatch_queue = []
            self.active_job_index = 0
            self.active_step_index = 0
            
            # 메인 스레드(UI)에서 에러 팝업을 띄우고 UI 텍스트 초기화
            def handle_failure():
                from tkinter import messagebox
                self.step_status_label.configure(text=f"[긴급 정지] {robot} 작업 실패", text_color="#EA5455")
                self.step_button.configure(text="배차 중단됨 (오류)", state="disabled")
                messagebox.showerror(
                    "작업 실패 알림",
                    f"{robot}에서 작업 실패가 감지되었습니다.\n명령어: {msg.get('명령어', '')}\n메시지: {text}\n\n모든 배차 시퀀스가 중지되었습니다."
                )
            self.after(0, handle_failure)
            return  # 이후 성공 체크 로직 타지 않음
        
        # ---------------------------------------------------------
        # 시퀀스 자동 전진 로직
        # ---------------------------------------------------------
        # 로봇팔 코드 수정 반영: 하나의 명령 시퀀스가 전부 완료되었을 때만 "상태":"성공" 패킷이 수신됨
        is_final_success = (status == "성공")
                
        # 최종 작업 완료 신호일 때만 큐를 전진시킵니다.
        if is_final_success:
            if getattr(self, "dispatch_queue", None) and self.active_job_index < len(self.dispatch_queue):
                if self.active_step_index <= len(self.dispatch_queue[self.active_job_index]["route"]):
                    self.after(0, self.advance_queue)

    def _confirm_register(self) -> None:
        """배너의 '저장' 버튼 - 추가 정보를 포함해서 화물을 등록합니다."""
        name = self._pending_cargo_name
        location = self._pending_cargo_location
        if not name or not location:
            return

        self.cargo_registry[name] = location
        
        # 층수가 기본값(1)이고 base도 비어있으면, 같은 위치의 최대 층수를 자동 계산
        input_floor = self.floor_var.get().strip() or "1"
        input_base = self.base_aruco_var.get().strip()
        if input_floor == "1" and not input_base:
            existing_at_loc = [
                d for n, d in self.cargo_details.items()
                if self.cargo_registry.get(n) == location and n != name
            ]
            if existing_at_loc:
                max_floor = max(int(d.get("층수", "1")) for d in existing_at_loc)
                input_floor = str(max_floor + 1)
                # 가장 높은 층의 ArUco를 base로 자동 설정
                top_cargo = [d for d in existing_at_loc if d.get("층수") == str(max_floor)]
                if top_cargo:
                    input_base = top_cargo[0].get("컨테이너ID", "")
        
        self.cargo_details[name] = {
            "컨테이너ID": self.aruco_var.get().strip(),
            "화물종류": self.cargo_type_var.get().strip(),
            "비고": self.cargo_note_var.get().strip(),
            "기반ArUco": input_base,
            "층수": input_floor
        }
        save_cargo_registry(self.cargo_registry)
        save_cargo_details(self.cargo_details)

        self._hide_detail_banner()
        self._refresh_cargo_list()

    def _skip_details_register(self) -> None:
        """배너의 '건너뛰기' 버튼 - 추가 정보 없이 이름/위치만 저장합니다."""
        name = self._pending_cargo_name
        location = self._pending_cargo_location
        if not name or not location:
            return

        self.cargo_registry[name] = location
        
        # 건너뛰기 시 메인 폼에서 입력한 층수를 그대로 사용합니다.
        input_floor = self.floor_var.get().strip() or "1"
        auto_base = ""
        
        # 층수가 1이고 기반 정보가 없으면 기존 위치 화물들을 참조해 자동 계산 (선택적)
        existing_at_loc = [
            d for n, d in self.cargo_details.items()
            if self.cargo_registry.get(n) == location and n != name
        ]
        
        if input_floor == "1" and existing_at_loc:
            max_floor = max(int(d.get("층수", "1")) for d in existing_at_loc)
            input_floor = str(max_floor + 1)
            top_cargo = [d for d in existing_at_loc if d.get("층수") == str(max_floor)]
            if top_cargo:
                auto_base = top_cargo[0].get("컨테이너ID", "")
        
        self.cargo_details[name] = {
            "컨테이너ID": "",
            "화물종류": "",
            "비고": "",
            "기반ArUco": auto_base,
            "층수": input_floor
        }
        save_cargo_registry(self.cargo_registry)
        save_cargo_details(self.cargo_details)

        self._hide_detail_banner()
        self._refresh_cargo_list()

    def _cancel_register(self) -> None:
        """배너의 '취소' 버튼 - 등록을 취소하고 배너를 숨깁니다."""
        self._hide_detail_banner()

    def _hide_detail_banner(self) -> None:
        """배너를 숨기고 입력 필드/임시 변수를 초기화합니다."""
        self.detail_banner.grid_forget()
        self._pending_cargo_name = None
        self._pending_cargo_location = None
        self.aruco_var.set("")
        self.base_aruco_var.set("")
        self.floor_var.set("1")
        self.cargo_type_var.set("")
        self.cargo_note_var.set("")
        self.cargo_name_var.set("")

    def delete_cargo(self, name: str) -> None:
        """화물 목록의 각 행에 있는 "삭제" 버튼에서 호출됩니다. name이 이미 정해져 있으니
        이름을 따로 입력받지 않고 확인 창만 띄우고 바로 지웁니다."""
        # 권한 체크
        try:
            app = self.winfo_toplevel()
            if hasattr(app, "current_user_id") and app.current_user_id:
                role = app.USERS.get(app.current_user_id, {}).get("role", "")
                if role not in ["최고 관리자 (Admin)", "현장 관리자 (Manager)"]:
                    messagebox.showerror("접근 거부", "화물 삭제 권한이 없습니다.\n(관리자 전용 기능)")
                    return
        except Exception:
            pass

        if name not in self.cargo_registry:
            return
        if not messagebox.askyesno("화물 삭제", f"'{name}' 화물을 삭제할까요?"):
            return

        del self.cargo_registry[name]
        self.cargo_details.pop(name, None)  # 세부정보도 같이 있으면 제거, 없으면 조용히 무시
        save_cargo_registry(self.cargo_registry)
        save_cargo_details(self.cargo_details)
        self._refresh_cargo_list()

    def generate_template(self) -> None:
        """generate_cargo_template.py의 build_template()를 호출해서 빈 엑셀 양식을 만들어줍니다."""
        try:
            build_cargo_excel_template()
        except Exception as exc:
            messagebox.showerror("생성 실패", f"엑셀 양식 생성 중 오류가 발생했습니다:\n{exc}")
            return
        messagebox.showinfo("생성 완료", "cargo_template.xlsx 파일이 현재 폴더에 생성되었습니다.")

    def export_cargo_to_excel(self) -> None:
        """지금 화면에 있는 화물 위치/세부정보 전체를 엑셀 파일로 내보냅니다."""
        default_name = f"cargo_export_{time.strftime('%Y%m%d_%H%M%S')}.xlsx"
        path = filedialog.asksaveasfilename(
            title="화물 정보 내보내기",
            defaultextension=".xlsx",
            initialfile=default_name,
            filetypes=[("Excel Files", "*.xlsx")],
        )
        if not path:
            return

        try:
            write_cargo_excel(self.cargo_registry, self.cargo_details, path)
        except Exception as exc:
            messagebox.showerror("내보내기 실패", f"엑셀로 내보내는 중 오류가 발생했습니다:\n{exc}")
            return

        messagebox.showinfo(
            "내보내기 완료",
            f"현재 화물 정보 {len(self.cargo_registry)}건을 저장했습니다:\n{path}",
        )

    def bulk_import_from_excel(self) -> None:
        """엑셀 파일을 통째로 읽어서 화물 등록부를 한 번에 채웁니다 (재배치 아님 - 단순 일괄 등록)."""
        path = filedialog.askopenfilename(
            title="화물 등록 엑셀 파일 선택",
            filetypes=[("Excel Files", "*.xlsx"), ("All Files", "*.*")],
        )
        if not path:
            return  # 사용자가 파일 선택 취소함

        try:
            new_registry, new_details, errors = bulk_import_cargo_from_excel(path, self.locations.keys())
        except Exception as exc:
            messagebox.showerror("가져오기 실패", f"엑셀 파일을 읽는 중 오류가 발생했습니다:\n{exc}")
            return

        if not new_registry:
            messagebox.showinfo("가져올 데이터 없음", "엑셀 파일에서 유효한 화물 데이터를 찾지 못했습니다.")
            return

        # 기존 등록부에 엑셀에서 읽은 내용을 덮어씀 (dict.update: 같은 키는 최신값으로 교체)
        self.cargo_registry.update(new_registry)
        self.cargo_details.update(new_details)
        save_cargo_registry(self.cargo_registry)
        save_cargo_details(self.cargo_details)
        self._refresh_cargo_list()

        summary = f"{len(new_registry)}건의 화물 위치를 등록했습니다."
        if errors:
            summary += "\n\n확인이 필요한 항목:\n" + "\n".join(errors[:15])  # 너무 길어지지 않게 15개까지만 표시
            if len(errors) > 15:
                summary += f"\n... 외 {len(errors) - 15}건"
        messagebox.showinfo("엑셀 일괄 등록 완료", summary)

    def _refresh_cargo_list(self) -> None:
        """화물 목록을 다시 그립니다. 각 행에 이름/위치/부가정보와 삭제 버튼이 함께 보입니다."""
        for widget in self.cargo_rows_container.winfo_children():
            widget.destroy()

        if not self.cargo_registry:
            ctk.CTkLabel(self.cargo_rows_container, text="등록된 화물이 없습니다.",
                        font=self.font_body, text_color="#9e9e9e").pack(anchor="w", pady=10)
            return

        for name, location in self.cargo_registry.items():
            detail = self.cargo_details.get(name, {})
            extra_parts = [v for v in (detail.get("컨테이너ID"), detail.get("화물종류")) if v]
            base_id = detail.get("기반ArUco")
            floor = detail.get("층수", "1")
            
            base_info = f" (Base: {base_id}, {floor}층)" if base_id else f" ({floor}층)"
            extra = f" [{', '.join(extra_parts)}]" if extra_parts else ""
            extra += base_info

            row = ctk.CTkFrame(self.cargo_rows_container, fg_color="#2b2b2b", corner_radius=6, border_width=1, border_color="#565b5e")
            row.pack(fill="x", pady=(0, 8))
            row.grid_columnconfigure(0, weight=1)

            text_lbl = ctk.CTkLabel(row, text=f"{name}: {location}{extra}", font=("Consolas", 13), text_color="#dce4ee", anchor="w")
            text_lbl.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
            
            ctk.CTkButton(row, text="🗣️ 배차", width=60, fg_color="#343638", border_width=1, border_color="#565b5e", text_color="#2e86c1", hover_color="#404040",
                         command=lambda n=name: self.send_cargo_to_command_popup(n)).grid(row=0, column=1, padx=(0, 8), pady=10)
            ctk.CTkButton(row, text="삭제", width=50, fg_color="#c0392b", hover_color="#922b21", text_color="white", font=self.font_body,
                         command=lambda n=name: self.delete_cargo(n)).grid(row=0, column=2, padx=(0, 10), pady=10)

    def send_cargo_to_command_popup(self, name: str) -> None:
        """화물 목록에서 이 버튼을 누르면, 그 화물명이 미리 채워진 채로 명령 팝업이 열립니다.
        목적지만 이어서 입력하고 실행하면 됩니다 (예: "화물A를 " + "항구로 옮겨")."""
        from command_center import open_command_popup
        open_command_popup(self, on_cargo_updated=self.refresh_from_disk, initial_text=f"{name}를 ")

    # ------------------------------------------------------------------
    # 엑셀 기반 일괄 재배치
    # ------------------------------------------------------------------
    def import_relocation_excel(self) -> None:
        """엑셀(바뀐 위치들)을 불러와서 기존 등록부와 비교(diff)한 뒤,
        실제로 위치가 바뀐 화물들만 골라 자동으로 이동 경로를 계획하고 큐에 등록합니다."""
        path = filedialog.askopenfilename(
            title="재배치 엑셀 파일 선택",
            filetypes=[("Excel Files", "*.xlsx"), ("All Files", "*.*")],
        )
        if not path:
            return

        try:
            jobs, unchanged, new_items, new_registry, new_details, errors, final_next_index = plan_relocations_from_excel(
                path, self.cargo_registry, self.locations.keys(),
                vehicle_positions=self.vehicle_positions, waypoint_rules=self.rules,
            )
        except Exception as exc:
            messagebox.showerror("가져오기 실패", f"엑셀 파일을 읽는 중 오류가 발생했습니다:\n{exc}")
            return

        # 차량 순번(다음 배차 순서)은 이번 배치 계획에 맞춰 바로 반영해둡니다.
        # (차량의 실제 위치는 큐가 한 단계씩 진행되면서 advance_queue에서 갱신됩니다)
        self.next_vehicle_index = final_next_index

        # 세부정보(컨테이너/ArUco ID 등)는 항상 최신으로 갱신 - 위치 자체는 큐 실행 완료 시에만 갱신
        self.cargo_details.update(new_details)
        save_cargo_details(self.cargo_details)

        # 처음 등록되는 화물은 이동할 이전 위치가 없으므로 바로 등록
        for name in new_items:
            self.cargo_registry[name] = new_registry[name]
        if new_items:
            save_cargo_registry(self.cargo_registry)
            self._refresh_cargo_list()

        if not jobs:
            self.relocation_label.configure(
                text=f"이동이 필요한 화물이 없습니다. (변경없음 {len(unchanged)}건, 신규 등록 {len(new_items)}건)",
                text_color="gray70",
            )
            return

        lines = []
        for job in jobs:
            route_text = " → ".join(f"{s.location}[{s.action}]" for s in job["route"])
            container_note = f" (ArUco: {job['container_id']})" if job.get("container_id") else ""
            lines.append(f"- {job['item']}{container_note}: {route_text}")

        summary = (
            f"이동 필요 {len(jobs)}건 / 변경없음 {len(unchanged)}건 / 신규 등록 {len(new_items)}건\n"
            + "\n".join(lines)
        )
        self.relocation_label.configure(text=summary, text_color="#28C76F")

        self._log(f"[재배치 계획] 엑셀 불러옴 - 이동 필요 {len(jobs)}건, 변경없음 {len(unchanged)}건, 신규 {len(new_items)}건")
        for line in lines:
            self._log(line)
        if errors:
            self._log(f"[확인 필요] {len(errors)}건 - 상세는 엑셀 일괄 등록 시 표시되는 목록 참고")

        self._start_queue(jobs)

    # ------------------------------------------------------------------
    # 큐 기반 배차 실행 (엑셀 재배치 다건 처리용 - 자연어 명령은 command_center.py 참고)
    # ------------------------------------------------------------------
    def _start_queue(self, jobs: List[Dict]) -> None:
        """새로운 작업 목록으로 큐를 초기화합니다. (자연어 명령의 job 1개짜리든,
        엑셀 재배치의 job 여러 개짜리든 이 함수 하나로 통일해서 시작합니다)"""
        self.dispatch_queue = jobs
        self.active_job_index = 0   # 큐의 첫 번째 화물부터 시작
        self.active_step_index = 0  # 그 화물 경로의 첫 번째 단계부터 시작
        self.step_status_label.configure(text="")

        if jobs:
            first_label = jobs[0].get("label") or jobs[0].get("item")
            self.step_button.configure(text=f"배차 실행 (데모 시작 - {first_label})", state="normal")
        else:
            self.step_button.configure(state="disabled")

    def advance_queue(self) -> None:
        """"다음 단계로" 버튼을 누를 때마다 호출됩니다. 큐에 쌓인 화물들을 하나씩,
        그 화물의 경로도 한 단계씩 순서대로 진행시키는 상태 머신입니다.

        진행 흐름:
          1. 지금 큐에서 처리 중인 화물(job)과, 그 화물 경로에서 지금 단계(step)를 가져옴
          2. 로그를 남기고 화면 상태 텍스트 갱신
          3. 이번 화물의 마지막 단계까지 끝났으면:
             - 일반 화물 job이면 화물 위치를 최종 도착지로 갱신하고 저장
             - 차량 복귀 job(is_return)이면 화물 등록부는 건드리지 않고 차량 위치만 갱신
             - 전체 경로를 한 문장으로 요약해서 로그에 남김
             - 다음 화물(job)로 넘어감 (더 없으면 전체 완료 처리)
             - 아직 남았으면: 다음 단계로 버튼 텍스트만 갱신
        """
        if not self.dispatch_queue or self.active_job_index >= len(self.dispatch_queue):
            return  # 큐가 비어있거나 이미 다 끝난 상태면 아무것도 안 함

        job = self.dispatch_queue[self.active_job_index]
        route: List[RouteStep] = job["route"]
        label = job.get("label") or job.get("item") or "AGV"
        step = route[self.active_step_index]

        if self.active_step_index == 0:
            self._log(f"[{label}] 이동 시작 - {step.location} 에서 출발")
        else:
            prev = route[self.active_step_index - 1]
            self._log(f"[{label}] 이동 중: {prev.location} → {step.location} ({step.action})")
            self._log(f"[{label}] 도착: {step.location} - {step.action} 단계 진행")

        job_progress = f"[{self.active_job_index + 1}/{len(self.dispatch_queue)}]"  # 예: "[2/5]"
        self.step_status_label.configure(text=f"{job_progress} {label} - 현재 단계: {step.location} ({step.action})")

        self.active_step_index += 1  # 다음 클릭을 위해 미리 한 칸 전진

        if self.active_step_index >= len(route):
            item = job.get("item")
            vehicle_idx = job.get("vehicle_index")

            if job.get("is_return"):
                # 차량 복귀 job - 화물 등록부는 안 건드리고 차량 위치만 갱신
                if vehicle_idx is not None:
                    self.vehicle_positions[vehicle_idx] = route[-1].location
                self._log(f"[{label}] 복귀 완료 - 대기장소 도착")
            else:
                # 일반 화물 job - 화물 위치를 최종 목적지로 갱신
                self.cargo_registry[item] = route[-1].location
                save_cargo_registry(self.cargo_registry)
                self._refresh_cargo_list()
                if vehicle_idx is not None:
                    self.vehicle_positions[vehicle_idx] = route[-1].location
                self._log(f"[{label}] 완료 - 위치가 '{route[-1].location}'로 갱신됨")

            self._log(describe_route_sentence(label, route))
            save_vehicle_state(self.vehicle_positions, self.next_vehicle_index)

            self.active_job_index += 1   # 큐의 다음 화물로 이동
            self.active_step_index = 0   # 새 화물이니 단계는 처음부터

            if self.active_job_index >= len(self.dispatch_queue):
                # 큐에 더 이상 화물이 없으면 전체 작업 종료
                self._log("[전체 완료] 모든 배차 작업이 끝났습니다.")
                self.step_button.configure(text="배차 완료", state="disabled")
            else:
                # 다음 화물이 남아있으면 버튼 텍스트만 바꿔서 계속 누를 수 있게 함
                next_job = self.dispatch_queue[self.active_job_index]
                next_label = next_job.get("label") or next_job.get("item") or "AGV"
                self.step_button.configure(text=f"다음 화물로 ({next_label})")
        else:
            # 같은 화물의 다음 단계가 남아있는 경우
            next_step = route[self.active_step_index]
            self.step_button.configure(text=f"다음 단계로 ({next_step.location})")

    # ------------------------------------------------------------------
    def _log(self, text: str) -> None:
        """진행 로그 창에 시각 태그를 붙여 한 줄 추가합니다."""
        self.log_box.configure(state="normal")  # 잠깐 편집 가능 상태로 풀었다가
        timestamp = time.strftime("%H:%M:%S")
        self.log_box.insert("end", f"[{timestamp}] {text}\n")
        self.log_box.see("end")  # 항상 최신 로그가 보이도록 맨 아래로 스크롤
        self.log_box.configure(state="disabled")  # 다시 읽기 전용으로 잠금


# -----------------------------------------------------------------------------
# 실차(ROS2 Nav2) 연동 참고 코드
# -----------------------------------------------------------------------------
#
# 실제 AGV와 연결할 때는 위 GUI의 "다음 단계로" 버튼 클릭을 아래와 같은 자동화된
# 흐름으로 바꾸면 됩니다. rclpy/nav2_msgs가 있는 실제 워크스페이스에서만 동작하며,
# 이 환경에는 ROS2가 없어 실행 테스트는 못 했습니다 - 실제 워크스페이스에서 한 번
# 돌려보시고 에러 있으면 바로 고쳐드리겠습니다.
#
# from location_to_nav_goal import send_nav_goal
#
# class CargoDispatcherROS2:
#     def __init__(self, lookup_path: str):
#         self.lookup_path = lookup_path
#
#     def dispatch(self, item_name: str, destination: str, cargo_registry: dict):
#         route = build_route(item_name, destination, cargo_registry)
#         for step in route:
#             send_nav_goal(step.location, self.lookup_path)   # 이동 완료까지 블로킹
#             if step.action == "적재":
#                 self.wait_for_load_confirmation()             # 적재 확인 대기
#             elif step.action == "하역":
#                 self.wait_for_unload_confirmation()            # 하역 확인 대기
#         cargo_registry[item_name] = route[-1].location
#
#     def wait_for_load_confirmation(self):
#         # TODO: 중앙 카메라가 "차량 위 화물 적재됨"을 발행하는 토픽 구독 후
#         #       메시지가 올 때까지 대기하도록 구현 (예: rclpy.spin_until_future_complete)
#         raise NotImplementedError
#
#     def wait_for_unload_confirmation(self):
#         # TODO: 하역 완료 확인 토픽 구독 대기
#         raise NotImplementedError


# -----------------------------------------------------------------------------
# 단독 실행
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")

    root = ctk.CTk()
    root.title("화물 위치 추적 및 엑셀 재배치")
    root.geometry("1100x700")

    app = CargoDispatchTool(root)
    app.pack(fill="both", expand=True, padx=15, pady=15)

    root.mainloop()
