"""
command_center.py

자연어 명령을 처리하는 팝업 창입니다. 각 화면(위치 마킹, 경유지 규칙, 화물 위치/배차)
우측 하단의 "명령" 버튼을 누르면 이 팝업이 뜹니다.

1순위: llm_command_parser.py(Ollama VLM)로 해석합니다. 언어/동의어에 무관하게 이해합니다
  - "cargo A를 port로 go" 처럼 영어가 섞여도, "이동"을 "go"라고 해도 인식
  - "창고에 있는 물건 전부를 항만으로 이동" 같은 위치 기반 일괄 이동도 처리
  - 현재 탑다운 영상을 함께 보고 목표 픽셀과 헤딩 픽셀을 결정
2순위(폴백): LLM 호출이 실패(패키지 미설치/API 키 없음/네트워크 오류/응답에 등록되지
  않은 이름 포함)하면 규칙 기반 문자열 파서로 자동 전환합니다. 오프라인에서도 기본
  기능(정확한 등록 이름을 그대로 말하는 경우)은 계속 동작합니다.

처리 결과는 다음 4가지입니다:
- 화물 단건 이동 (예: "화물A를 항구로 옮겨") -> 화물의 픽업 위치를 거쳐가는 경로 계산 후
  즉시 화물 위치를 목적지로 갱신하고 cargo_locations.json에 저장
- 위치 기반 일괄 이동 (예: "창고에 있는 물건 전부를 항만으로 이동") -> 그 위치에 있는
  모든 화물을 찾아 각각 경로 계산 후 전부 이동 처리
- 순수 위치 이동 (예: "항구로 이동해줘") -> 화물 데이터는 건드리지 않고 경로만 계산
- 영상 좌표 이동 -> VLM 좌표를 중앙제어 HTTP API로 전달해 Nav2 목표 생성

필요한 로직은 새로 만들지 않고 cargo_dispatch_tool.py / waypoint_rules.py에 있는 것을
그대로 가져다 씁니다.
"""

import re
import time
from typing import Dict, List, Optional, Tuple

import customtkinter as ctk
import cv2

from central_control_client import (
    CentralControlApiError,
    CentralControlClient,
)
from cctv_monitor_view import CCTVMonitorView
from waypoint_rules import expand_leg, load_waypoint_rules, resolve_travel_route
from cargo_dispatch_tool import (
    RouteStep,
    STANDBY_LOCATION,
    NUM_VEHICLES,
    build_route,
    build_return_to_standby_route,
    vehicle_home_location,
    record_vehicle_job,
    describe_route_sentence,
    load_vehicle_state,
    save_vehicle_state,
    load_cargo_registry,
    load_cargo_details,
    save_cargo_registry,
    save_cargo_details,
    load_named_locations,
    location_coord_text,
)
from llm_command_parser import LLMParseError, parse_command_with_llm
from ros_control_bridge import RosControlBridge
from visual_navigation import (
    VisualNavigationError,
    compact_detections,
    resolve_detection_approach,
    validate_pixel_navigation,
)
from yolo_detection_client import (
    YoloDetectionClient,
    YoloDetectionError,
)

PointXY = Tuple[float, float]


# -----------------------------------------------------------------------------
# 자연어 파싱 - 위치 이동 명령용
# -----------------------------------------------------------------------------

_DESTINATION_SUFFIXES = [
    "으로 이동해줘", "로 이동해줘", "으로 이동", "로 이동",
    "으로 가줘", "로 가줘", "으로 가", "로 가",
    "에 가줘", "에 가", "까지 가줘", "까지 가",
    "으로 보내줘", "로 보내줘", "으로 보내", "로 보내",
]


def extract_travel_sequence(command: str, known_names) -> List[str]:
    """문장에 등록된 위치명이 여러 개 나오면, 등장 순서 그대로 전부 뽑아냅니다.
    예: "대기장소에서 창고로 갔다가 항구로가" -> ["대기장소", "창고", "항구"]"""
    text = command.strip()
    if not text:
        return []

    raw_matches: List[Tuple[int, str]] = []
    for name in known_names:
        if not name:
            continue
        start = 0
        while True:
            pos = text.find(name, start)
            if pos == -1:
                break
            raw_matches.append((pos, name))
            start = pos + len(name)

    if not raw_matches:
        return []

    raw_matches.sort(key=lambda m: (m[0], -len(m[1])))

    sequence: List[str] = []
    last_end = -1
    for pos, name in raw_matches:
        if pos < last_end:
            continue
        sequence.append(name)
        last_end = pos + len(name)

    return sequence


def resolve_multi_stop_route(stops: List[str], rules: Optional[Dict[str, str]] = None) -> List[str]:
    """언급된 여러 지점을 순서대로 이어 붙이면서, 구간마다 필수 경유지 규칙도 적용합니다."""
    if not stops:
        return []
    if rules is None:
        rules = load_waypoint_rules()

    full_route = list(resolve_travel_route(stops[0], rules))
    for i in range(1, len(stops)):
        frm = full_route[-1]
        to = stops[i]
        for name in expand_leg(frm, to, rules):
            full_route.append(name)

    return full_route


# -----------------------------------------------------------------------------
# 자연어 파싱 - 화물 배차 명령용
# -----------------------------------------------------------------------------

_CARGO_HINT_WORDS = ["화물", "물품", "물건"]
_ACTION_PHRASES = sorted(set(_DESTINATION_SUFFIXES + [
    "으로 옮겨줘", "로 옮겨줘", "으로 옮겨", "로 옮겨",
    "으로 가져다줘", "로 가져다줘", "으로 가져가", "로 가져가",
    "이동해줘", "이동", "옮겨줘", "옮겨", "보내줘", "보내", "가져다줘", "가져가",
]), key=len, reverse=True)


def mentions_cargo(command: str, known_cargo_names) -> bool:
    """문장이 화물 관련 명령인지 판단합니다 (화물/물품/물건 단어, 또는 등록된 화물명 포함 여부)."""
    return any(word in command for word in _CARGO_HINT_WORDS) or any(
        name and name in command for name in known_cargo_names
    )


def extract_cargo_command(
    command: str,
    known_items,
    known_locations,
) -> Tuple[Optional[str], Optional[str]]:
    """명령 문장에서 (화물명, 목적지명)을 추출합니다. 못 찾으면 각각 None.
    조사가 있든("물품 A를 항구로 옮겨") 없든("화물 A 항구로 이동") 둘 다 처리합니다."""
    text = command.strip()
    known_items = list(known_items)
    known_locations = list(known_locations)

    destination = None
    for name in sorted(known_locations, key=len, reverse=True):
        if name and name in text:
            destination = name
            break
    if destination is None:
        for suffix in sorted(_DESTINATION_SUFFIXES, key=len, reverse=True):
            if text.endswith(suffix):
                destination = text[: -len(suffix)].strip()
                break

    remainder = text
    if destination and destination in remainder:
        remainder = remainder.replace(destination, " ", 1)
    for phrase in _ACTION_PHRASES:
        remainder = remainder.replace(phrase, " ")
    remainder = remainder.strip()

    item = None
    for name in sorted(known_items, key=len, reverse=True):
        if name and name in remainder:
            item = name
            break

    if item is None:
        collapsed = remainder.replace(" ", "")
        for name in sorted(known_items, key=len, reverse=True):
            if name and name in collapsed:
                item = name
                break

    if item is None:
        cleaned = remainder
        for prefix in ["화물", "물품", "물건"]:
            cleaned = cleaned.replace(prefix, " ")
        for particle in ["를", "을"]:
            cleaned = cleaned.replace(particle, " ")
        candidate = cleaned.strip()
        if candidate:
            item = candidate.split()[0]

    return item, destination


_BULK_QUANTIFIER_WORDS = ["모든", "전체", "전부", "죄다", "있는 거 다"]


def mentions_bulk_quantifier(command: str) -> bool:
    """"모든/전체/전부" 같은 수량 표현이 있는지 확인합니다 (일괄 이동 명령인지 판단용)."""
    return any(word in command for word in _BULK_QUANTIFIER_WORDS)


def extract_bulk_cargo_command(command: str, known_locations) -> Tuple[Optional[str], Optional[str]]:
    """
    "창고에 있는 모든 화물을 항구로 이동" 같은 문장에서 (출발위치, 목적지)를 추출합니다.
    문장에 등장하는 위치명 순서상 맨 처음 나온 걸 출발위치, 맨 마지막에 나온 걸 목적지로 봅니다.
    (extract_travel_sequence로 위치명들을 순서대로 뽑아낸 뒤 첫/마지막만 사용)
    """
    stops = extract_travel_sequence(command, known_locations)
    if len(stops) < 2:
        return None, None
    return stops[0], stops[-1]


def extract_cargo_range(command: str, known_items) -> Optional[List[str]]:
    """
    "화물 A~D 창고로 이동" / "화물A~화물D 창고로 이동" 같은 범위 표현에서
    해당 범위에 들어가는 실제 등록된 화물명을 전부 뽑아냅니다.
    LLM에게 맡기면 "A~D"를 위치명으로 착각하는 등 일관성이 떨어져서, 이 패턴은
    코드에서 직접(결정론적으로) 처리합니다.

    동작 방식: "~" 앞에서 등록된 화물명과 일치하는 부분을 찾아 시작점으로 삼고,
    "~" 뒤에 오는 접미사(알파벳 한 글자 또는 숫자)로 끝점을 정한 뒤, 그 사이에
    해당하는 화물명들 중 실제로 등록된 것만 순서대로 반환합니다.
    """
    if "~" not in command:
        return None

    idx = command.index("~")
    before_text = command[:idx].replace(" ", "")  # 공백 무시하고 비교 ("화물 B" -> "화물B")
    after_text = command[idx + 1:]

    # "~" 바로 앞에서 등록된 화물명과 일치하는 가장 긴 것을 시작점으로 채택
    start_item = None
    for name in sorted(known_items, key=len, reverse=True):
        if name and before_text.endswith(name):
            start_item = name
            break
    if start_item is None:
        return None

    # 시작 화물명을 "접두어 + 접미사"(예: "화물" + "A")로 분리
    m = re.match(r"^(.*?)([A-Za-z0-9]+)$", start_item)
    if not m:
        return None
    prefix, start_suffix = m.groups()

    # "~" 뒤 접미사 추출: 등록된 화물명 전체가 있으면 그 접미사를, 아니면 알파벳/숫자만 추출
    after_stripped = after_text.lstrip()
    end_suffix = None
    for name in sorted(known_items, key=len, reverse=True):
        if name and after_stripped.startswith(name):
            m3 = re.match(r"^(.*?)([A-Za-z0-9]+)$", name)
            if m3:
                end_suffix = m3.group(2)
            break
    if end_suffix is None:
        m2 = re.match(r"([A-Za-z0-9]+)", after_stripped)
        if m2:
            end_suffix = m2.group(1)
    if end_suffix is None:
        return None

    if start_suffix.isalpha() and end_suffix.isalpha() and len(start_suffix) == 1 and len(end_suffix) == 1:
        start_ord, end_ord = sorted([ord(start_suffix.upper()), ord(end_suffix.upper())])
        candidates = [f"{prefix}{chr(c)}" for c in range(start_ord, end_ord + 1)]
    elif start_suffix.isdigit() and end_suffix.isdigit():
        start_n, end_n = sorted([int(start_suffix), int(end_suffix)])
        candidates = [f"{prefix}{n}" for n in range(start_n, end_n + 1)]
    else:
        return None

    result = [c for c in candidates if c in known_items]
    return result or None


# -----------------------------------------------------------------------------
# 팝업 창
# -----------------------------------------------------------------------------

class CommandPopup(ctk.CTkToplevel):
    """어느 화면에서 열든 항상 최신 데이터(위치/화물/경유지 규칙)를 디스크에서 새로 읽어옵니다.

    on_cargo_updated: 화물 명령이 실행되어 화물 위치가 실제로 바뀌었을 때 호출되는 콜백.
    팝업을 연 화면(주로 화물 위치/배차 탭)이 이미 메모리에 들고 있는 화물 목록은
    파일이 바뀌어도 저절로 갱신되지 않으므로, 이 콜백으로 "지금 바로" 화면을 새로고침합니다.
    """

    def __init__(self, master, on_cargo_updated=None, initial_text=None, **kwargs):
        super().__init__(master, **kwargs)
        self.title("🗣️ 자연어 명령")
        self.geometry("720x520")
        self.attributes("-topmost", True)  # 팝업이 항상 위에 보이도록
        self.on_cargo_updated = on_cargo_updated
        self.initial_text = initial_text  # 다른 화면에서 "이 화물로 명령 시작하기" 눌렀을 때 미리 채울 문구

        self.locations = load_named_locations()
        self.cargo_registry = load_cargo_registry()
        self.cargo_details = load_cargo_details()
        self.vehicle_positions, self.next_vehicle_index = load_vehicle_state()
        self.rules = load_waypoint_rules()

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=18, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=12)

        self._build_ui()

    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(4, weight=1)

        ctk.CTkLabel(self, text="🗣️ 자연어 명령", font=self.font_title).grid(
            row=0, column=0, sticky="w", padx=15, pady=(15, 5))
        ctk.CTkLabel(
            self,
            text="Claude Haiku가 우선 해석합니다 (동의어/영어 혼용/일괄 이동 가능).\n"
                 "예: \"cargo A를 port로 go\" / \"창고에 있는 물건 전부를 항만으로 이동\"",
            font=self.font_body, text_color="gray70",
        ).grid(row=1, column=0, sticky="w", padx=15, pady=(0, 10))

        cmd_row = ctk.CTkFrame(self, fg_color="transparent")
        cmd_row.grid(row=2, column=0, sticky="ew", padx=15, pady=(0, 10))
        cmd_row.grid_columnconfigure(0, weight=1)

        self.command_var = ctk.StringVar()
        if self.initial_text:
            self.command_var.set(self.initial_text)  # 예: "화물A를 " -> 목적지만 이어서 입력하면 됨
        cmd_entry = ctk.CTkEntry(cmd_row, textvariable=self.command_var,
                                 placeholder_text="예: 화물A를 항구로 옮겨")
        cmd_entry.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        cmd_entry.bind("<Return>", lambda e: self.run_command())
        cmd_entry.focus_set()
        cmd_entry.icursor("end")  # 미리 채운 문구 뒤에 바로 이어 쓸 수 있게 커서를 맨 끝으로
        ctk.CTkButton(cmd_row, text="실행", width=90, command=self.run_command).grid(row=0, column=1)

        self.result_label = ctk.CTkLabel(self, text="", font=self.font_body, text_color="#3B82F6",
                                         wraplength=680, justify="left")
        self.result_label.grid(row=3, column=0, sticky="w", padx=15, pady=(0, 10))

        log_frame = ctk.CTkFrame(self, corner_radius=10)
        log_frame.grid(row=4, column=0, sticky="nsew", padx=15, pady=(0, 15))
        log_frame.grid_columnconfigure(0, weight=1)
        log_frame.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(log_frame, text="실행 로그", font=self.font_body).grid(
            row=0, column=0, sticky="w", padx=15, pady=(10, 5))
        self.log_box = ctk.CTkTextbox(log_frame, font=("Consolas", 12))
        self.log_box.grid(row=1, column=0, sticky="nsew", padx=15, pady=(0, 12))
        self.log_box.configure(state="disabled")

    # ------------------------------------------------------------------
    def run_command(self) -> None:
        command = self.command_var.get().strip()
        if not command:
            return

        # 0순위: "화물 A~D" 같은 범위 표현은 LLM에 맡기면 일관성이 떨어져서
        # (위치명으로 착각하는 등) 코드에서 먼저 확실하게 처리하고 끝냅니다.
        if "~" in command and mentions_cargo(command, self.cargo_registry.keys()):
            range_items = extract_cargo_range(command, self.cargo_registry.keys())
            if range_items:
                destination = None
                for name in sorted(self.locations.keys(), key=len, reverse=True):
                    if name and name in command:
                        destination = name
                        break
                if destination:
                    self._log(f"[명령 해석-범위감지] '{command}' -> 화물={range_items}, 목적지={destination}")
                    moved = self._move_items_to(range_items, destination, "범위 지정된 화물")
                    self.result_label.configure(
                        text=f"[범위 이동 완료] {', '.join(range_items)} → '{destination}' ({len(moved)}건)",
                        text_color="#28C76F",
                    )
                    return
                self._log("[범위 감지됨] 하지만 목적지를 못 찾아 다른 방식으로 재시도")

        # 1순위: LLM으로 해석 시도 (언어/동의어 무관, 일괄 이동/화물종류별 이동/복합 명령까지 이해)
        llm_result = None
        image_jpeg, image_width, image_height = self._current_vlm_image()
        detection_summary = self._current_yolo_detections()
        compact_detection_list = compact_detections(detection_summary)
        known_cargo_types = sorted({
            detail.get("화물종류", "") for detail in self.cargo_details.values() if detail.get("화물종류")
        })
        try:
            llm_result = parse_command_with_llm(
                command,
                list(self.cargo_registry.keys()),
                list(self.locations.keys()),
                known_cargo_types,
                image_jpeg=image_jpeg,
                image_width=image_width,
                image_height=image_height,
                yolo_detections=compact_detection_list,
            )
        except LLMParseError as exc:
            self._log(f"[LLM 사용 불가 - 규칙 기반으로 대체] {exc}")

        if llm_result is not None:
            self._log(f"[VLM 원본 응답] {llm_result}")
            handled = self._handle_llm_result(
                command,
                llm_result,
                detection_summary,
                image_width,
                image_height,
            )
            if handled:
                return
            self._log("[LLM 결과를 적용할 수 없어 규칙 기반으로 재시도]")

        # 폴백: 규칙 기반 파서 (LLM 미설정/실패/등록되지 않은 이름 반환 시)
        if mentions_cargo(command, self.cargo_registry.keys()):
            if mentions_bulk_quantifier(command):
                # "모든/전체/전부" 같은 표현이 있으면 화물 하나가 아니라
                # 특정 위치의 화물 전체를 옮기라는 뜻이므로 일괄 이동으로 먼저 시도
                source, destination = extract_bulk_cargo_command(command, self.locations.keys())
                if source and destination:
                    self._log(f"[명령 해석-규칙기반] '{command}' -> '{source}'의 화물 전체를 '{destination}'로 이동")
                    self._execute_cargo_bulk(source, destination)
                    return
                self._log("[일괄 이동 위치를 못 찾음 - 단건 명령으로 재시도]")
            self._run_cargo_command(command)
        else:
            self._run_travel_command(command)

    def _current_vlm_image(self):
        """Encode the latest top-down dashboard frame for the local VLM."""
        frame = CCTVMonitorView.SHARED_FRAME
        if frame is None:
            self._log("[VLM 영상] 현재 수신된 탑다운 프레임이 없습니다.")
            return None, 640, 480
        frame = frame.copy()
        height, width = frame.shape[:2]
        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 80],
        )
        if not success:
            self._log("[VLM 영상] 현재 프레임 JPEG 변환에 실패했습니다.")
            return None, width, height
        self._log(f"[VLM 영상] 현재 프레임 {width}x{height}를 함께 전송합니다.")
        return encoded.tobytes(), width, height

    def _current_yolo_detections(self):
        """Fetch one fresh YOLO summary paired with the current VLM request."""
        try:
            summary = YoloDetectionClient().get_latest()
        except YoloDetectionError as exc:
            self._log(f"[VLM YOLO JSON 없음] {exc}")
            return None
        detections = compact_detections(summary)
        labels = [
            f'{item["detection_index"]}:{item["label"]}'
            for item in detections
        ]
        self._log(
            f'[VLM YOLO JSON] {len(detections)}개 검출: '
            f'{", ".join(labels) if labels else "없음"}'
        )
        return summary

    def _handle_llm_result(
        self,
        command: str,
        result: Dict,
        detection_summary=None,
        image_width=640,
        image_height=480,
    ) -> bool:
        """LLM 응답({"actions": [...]})을 검증하고 순서대로 실행합니다.
        한 문장에 지시가 여러 개 섞여 있으면 actions에 여러 개가 들어오고, 전부 실행합니다.
        하나도 실행하지 못했으면 False를 반환해서 규칙 기반 파서로 넘어가게 합니다."""
        actions = result.get("actions")
        if not isinstance(actions, list) or not actions:
            return False

        any_handled = False
        for action in actions:
            if (
                isinstance(action, dict)
                and self._handle_single_action(
                    command,
                    action,
                    detection_summary,
                    image_width,
                    image_height,
                )
            ):
                any_handled = True
        return any_handled

    def _handle_single_action(
        self,
        command: str,
        action: Dict,
        detection_summary=None,
        image_width=640,
        image_height=480,
    ) -> bool:
        """actions 배열 안의 액션 하나를 검증하고 실행합니다.
        검증에 실패하면 왜 실패했는지 로그에 남깁니다 - 여러 액션 중 하나가 조용히
        빠지면 사용자가 원인을 알기 어려우므로, 실패 이유를 항상 보이게 합니다."""
        action_type = action.get("type")

        if action_type == "cargo_single":
            item = action.get("item")
            destination = action.get("destination")
            if not item:
                self._log(f"[액션 무시] 화물명이 비어있습니다: {action}")
                return False
            if destination not in self.locations:
                self._log(f"[액션 무시] 목적지 '{destination}'이 등록된 위치 목록에 없습니다: {action}")
                return False

            target_aruco_id = action.get("target_aruco_id", "")
            target_floor = action.get("target_floor", 1)

            if item not in self.cargo_registry:
                # 미등록 화물 → 목적지에 자동 등록하고 바로 배치 완료 처리
                self._log(f"[자동 등록] 화물 '{item}'이 등록부에 없어 '{destination}'에 신규 등록합니다.")
                self.cargo_registry[item] = destination
                
                # 추가 정보로 빈 세부정보 생성 후 층수/기반 설정
                detail = self.cargo_details.get(item, {})
                detail["기반ArUco"] = target_aruco_id
                detail["층수"] = str(target_floor)
                self.cargo_details[item] = detail
                
                save_cargo_registry(self.cargo_registry)
                save_cargo_details(self.cargo_details)
                
                if self.on_cargo_updated:
                    self.on_cargo_updated()
                self.result_label.configure(
                    text=f"[신규 등록 완료] 화물 '{item}' → '{destination}' (Base: {target_aruco_id}, {target_floor}층)",
                    text_color="#28C76F",
                )
                return True

            self._log(f"[LLM 해석] '{command}' -> 화물={item}, 목적지={destination}, Base={target_aruco_id}, Floor={target_floor}")
            self._execute_cargo_single(item, destination, target_aruco_id, target_floor)
            return True

        if action_type == "cargo_bulk_by_location":
            source = action.get("source_location")
            destination = action.get("destination")
            if source not in self.locations:
                self._log(f"[액션 무시] 출발위치 '{source}'이 등록된 위치 목록에 없습니다: {action}")
                return False
            if destination not in self.locations:
                self._log(f"[액션 무시] 목적지 '{destination}'이 등록된 위치 목록에 없습니다: {action}")
                return False
            self._log(f"[LLM 해석] '{command}' -> '{source}'의 화물 전체를 '{destination}'로 이동")
            self._execute_cargo_bulk(source, destination)
            return True

        if action_type == "cargo_bulk_by_type":
            cargo_type = action.get("cargo_type")
            destination = action.get("destination")
            known_types = {d.get("화물종류") for d in self.cargo_details.values() if d.get("화물종류")}
            if not cargo_type or cargo_type not in known_types:
                self._log(f"[액션 무시] 화물종류 '{cargo_type}'이 등록된 종류 목록({sorted(known_types)})에 없습니다: {action}")
                return False
            if destination not in self.locations:
                self._log(f"[액션 무시] 목적지 '{destination}'이 등록된 위치 목록({sorted(self.locations.keys())})에 없습니다: {action}")
                return False
            self._log(f"[LLM 해석] '{command}' -> 화물종류='{cargo_type}' 전체를 '{destination}'로 이동")
            self._execute_cargo_bulk_by_type(cargo_type, destination)
            return True

        if action_type == "query_location":
            item = action.get("item", "").strip()
            if not item:
                self.result_label.configure(text="[조회 실패] 위치를 확인할 대상이 명확하지 않습니다.", text_color="#EA5455")
                return True
                
            self._log(f"[LLM 해석] '{command}' -> '{item}' 위치 조회")
            
            # 1) 차량 검색
            if "차" in item or "agv" in item.lower():
                for idx, pos in enumerate(self.vehicle_positions):
                    if str(idx+1) in item or ("yellow" in item.lower() and idx == 0) or ("blue" in item.lower() and idx == 1):
                        self.result_label.configure(text=f"[위치 조회] 차량 {idx+1}은(는) 현재 '{pos}'에 대기 중입니다.", text_color="#00CFE8")
                        return True
                self.result_label.configure(text=f"[위치 조회] 전체 {len(self.vehicle_positions)}대의 차량이 맵 상에 있습니다. 레이더 뷰를 확인하세요.", text_color="#00CFE8")
                return True
                
            # 2) 화물 검색
            found = False
            for cargo, loc in self.cargo_registry.items():
                if item in cargo or cargo in item:
                    detail = self.cargo_details.get(cargo, {})
                    floor = detail.get("층수", "1")
                    base = detail.get("기반ArUco", "")
                    base_str = f", 기반: {base}" if base else ""
                    self.result_label.configure(text=f"[위치 조회] '{cargo}' 화물은 '{loc}' ({floor}층{base_str})에 있습니다.", text_color="#00CFE8")
                    found = True
                    break
                    
            if not found:
                self.result_label.configure(text=f"[위치 조회] 시스템에 '{item}' 객체가 등록되어 있지 않거나 찾을 수 없습니다.", text_color="#EA5455")
            return True

        if action_type == "travel":
            stops = [s for s in (action.get("stops") or []) if s in self.locations]
            if not stops:
                self._log(f"[액션 무시] 등록된 위치와 일치하는 지점이 없습니다: {action}")
                return False
            self._log(f"[LLM 해석] '{command}' -> 위치 이동: {' → '.join(stops)}")
            self._execute_travel(stops)
            return True

        if action_type == "visual_navigation":
            if detection_summary is None:
                self._log(
                    '[VLM 객체 접근 거부] 최신 YOLO 검출 JSON이 없습니다.'
                )
                return True
            try:
                target, heading, selected = resolve_detection_approach(
                    action,
                    detection_summary,
                    image_width,
                    image_height,
                )
                response = CentralControlClient().send_pixel_goal(
                    target,
                    heading,
                    mode=(
                        'parking_b1'
                        if selected['label'] == 'B-1'
                        else 'direct'
                    ),
                )
            except (VisualNavigationError, CentralControlApiError) as exc:
                self._log(f"[VLM 좌표 전송 실패] {exc}")
                self.result_label.configure(
                    text=f"[VLM 좌표 전송 실패] {exc}",
                    text_color="#EA5455",
                )
                return True

            command_id = response.get("command_id", "unknown")
            duplicate = bool(response.get("duplicate", False))
            self._log(
                f"[VLM 좌표 전송] target={target}, heading={heading}, "
                f"command_id={command_id}, duplicate={duplicate}"
            )
            self.result_label.configure(
                text=(
                    "[VLM 차량 이동 명령 전송 완료]\n"
                    f"목표={target}, 방향={heading}\n"
                    f"명령 ID={command_id}"
                ),
                text_color="#28C76F",
            )
            return True

        if action_type == "bringup":
            color_str = str(action.get("color", "")).lower().strip()
            
            if "blue" in color_str or "파란" in color_str or "파랑" in color_str:
                domain_id = 12
                ip_address = "192.168.0.102"
                robot_name = "Blue"
            elif "yellow" in color_str or "노란" in color_str or "노랑" in color_str or "옐로우" in color_str:
                domain_id = 13
                ip_address = "192.168.0.96"
                robot_name = "Yellow"
            else:
                self._log(f"[액션 무시] 지원하지 않는 색상이거나 색상을 인식하지 못했습니다: {color_str}")
                return False
                
            self._log(f"[LLM 해석] '{command}' -> {robot_name} 로봇 시동 (Bringup)")
            
            import subprocess
            import threading
            import time
            
            session_name = f"pinky_bringup_{robot_name.lower()}"
            cmd_str = f"export ROS_DOMAIN_ID={domain_id}; ros2 launch pinky bringup_robot.launch.xml"
            title = f"Pinky Bringup (Domain: {domain_id})"
            
            subprocess.run(["/usr/bin/tmux", "kill-session", "-t", session_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # 터미널 창을 띄우지 않고 백그라운드(숨김) 모드로 tmux 세션 시작
            subprocess.run(["/usr/bin/tmux", "new-session", "-d", "-s", session_name, f"ssh -tt pinky@{ip_address}"])
            
            def auto_type():
                # 1. Bringup 실행
                time.sleep(5.0)
                subprocess.run(["/usr/bin/tmux", "send-keys", "-t", session_name, "1", "Enter"])
                time.sleep(5.0)
                subprocess.run(["/usr/bin/tmux", "send-keys", "-t", session_name, cmd_str, "Enter"])
                
                # 자동으로 불(LED)도 켬 (Yellow, Blue 공통)
                # 2. LED Server 및 Service Call 실행 (단일 터미널)
                time.sleep(5.0)  # Bringup이 어느정도 올라오길 잠시 대기
                led_session = f"pinky_led_{robot_name.lower()}"
                subprocess.run(["/usr/bin/tmux", "kill-session", "-t", led_session], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                # 터미널 창을 띄우지 않고 백그라운드(숨김) 모드로 tmux 세션 시작
                subprocess.run(["/usr/bin/tmux", "new-session", "-d", "-s", led_session, f"ssh -tt pinky@{ip_address}"])
                time.sleep(5.0)
                subprocess.run(["/usr/bin/tmux", "send-keys", "-t", led_session, "1", "Enter"])
                time.sleep(5.0)
                
                # LED 서버를 백그라운드(&)로 실행
                led_run_cmd = f"export ROS_DOMAIN_ID={domain_id}; ros2 run pinky led_server &"
                subprocess.run(["/usr/bin/tmux", "send-keys", "-t", led_session, led_run_cmd, "Enter"])
                
                # 5초 지연시간 후 같은 터미널에 서비스 호출 명령어 입력
                time.sleep(5.0)
                if robot_name == "Yellow":
                    r, g, b = 255, 255, 0
                else:
                    r, g, b = 0, 0, 255
                    
                led_srv_cmd = f"export ROS_DOMAIN_ID={domain_id}; ros2 service call /set_led pinky/srv/SetLed \"{{command: 'fill', r: {r}, g: {g}, b: {b}}}\""
                subprocess.run(["/usr/bin/tmux", "send-keys", "-t", led_session, led_srv_cmd, "Enter"])
                    
            threading.Thread(target=auto_type, daemon=True).start()
            
            self.result_label.configure(
                text=f"[{robot_name} 로봇 시동]\n{ip_address}(도메인 {domain_id})으로 자동 연결하여 Bringup을 실행합니다.",
                text_color="#28C76F",
            )
            return True

        if action_type == "shutdown":
            color_str = str(action.get("color", "")).lower().strip()
            
            if "blue" in color_str or "파란" in color_str or "파랑" in color_str:
                robot_name = "Blue"
                ip_address = "192.168.0.102"
            elif "yellow" in color_str or "노란" in color_str or "노랑" in color_str or "옐로우" in color_str:
                robot_name = "Yellow"
                ip_address = "192.168.0.96"
            else:
                self._log(f"[액션 무시] 지원하지 않는 색상이거나 색상을 인식하지 못했습니다: {color_str}")
                return False
                
            self._log(f"[LLM 해석] '{command}' -> {robot_name} 로봇 시동 끄기 (Shutdown)")
            
            import os
            import subprocess
            import threading
            import time
            
            def auto_shutdown():
                domain_id = 12 if robot_name == "Blue" else 13
                led_session = f"pinky_led_{robot_name.lower()}"
                
                # 1. 이미 열려 있는 기존 LED 터미널에 조명 끄기(r:0, g:0, b:0) 서비스 명령 즉시 전송
                # (새로 SSH 연결할 필요 없이, 백그라운드(&)로 실행 중인 터미널의 프롬프트를 재활용)
                off_cmd = f"export ROS_DOMAIN_ID={domain_id}; ros2 service call /set_led pinky/srv/SetLed \"{{command: 'fill', r: 0, g: 0, b: 0}}\""
                subprocess.run(["/usr/bin/tmux", "send-keys", "-t", led_session, off_cmd, "Enter"])
                
                # 서비스가 처리되고 조명이 물리적으로 꺼질 때까지 충분히 대기
                time.sleep(5.0)
                
                # 2. 모든 세션 및 프로세스 완전 종료
                try:
                    subprocess.run(["/usr/bin/tmux", "kill-session", "-t", f"pinky_bringup_{robot_name.lower()}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    subprocess.run(["/usr/bin/tmux", "kill-session", "-t", f"pinky_teleop_{robot_name.lower()}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    subprocess.run(["/usr/bin/tmux", "kill-session", "-t", led_session], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    # pkill은 IP를 통해 해당 로봇으로 가는 SSH 세션만 닫습니다.
                    os.system(f"pkill -9 -f 'ssh -tt pinky@{ip_address}'")
                except Exception as e:
                    print(f"[종료 실패] 프로세스 강제 종료 중 오류: {e}")
                    
            threading.Thread(target=auto_shutdown, daemon=True).start()
                
            self.result_label.configure(
                text=f"[{robot_name} 로봇 시동 종료]\nLED를 끄고 모든 원격 연결을 종료하는 중입니다...",
                text_color="#EA5455",
            )
            return True

        # type == "unknown" 이거나 예상 밖의 값이면 처리 못 한 것으로 간주
        return False

    def _run_cargo_command(self, command: str) -> None:
        """화물 배차 명령 (규칙 기반 폴백 경로)."""
        item, destination = extract_cargo_command(
            command, self.cargo_registry.keys(), self.locations.keys()
        )

        if item is None or destination is None:
            self.result_label.configure(
                text=(
                    f"명령을 해석하지 못했습니다. (화물 인식: {item}, 목적지 인식: {destination})\n"
                    f"등록된 화물: {', '.join(self.cargo_registry.keys()) or '없음'} / "
                    f"등록된 위치: {', '.join(self.locations.keys()) or '없음'}"
                ),
                text_color="#EA5455",
            )
            return

        self._log(f"[명령 해석] '{command}' -> 화물={item}, 목적지={destination}")
        self._execute_cargo_single(item, destination, "", 1)

    def _assign_next_vehicle(self) -> Tuple[int, str]:
        """다음 작업을 맡을 차량 번호와, 그 차량이 지금 있는 위치(출발점)를 정합니다.
        차량 대수(NUM_VEHICLES)만큼은 각자 마지막으로 도착한 곳에서 순서대로 돌아가며
        배차되고, 처음 두 대(대수만큼)는 전부 대기장소에서 시작합니다."""
        vehicle_idx = self.next_vehicle_index
        start_location = self.vehicle_positions[vehicle_idx]
        self.next_vehicle_index = (self.next_vehicle_index + 1) % len(self.vehicle_positions)
        return vehicle_idx, start_location

    def _calculate_target_info(self, location: str) -> tuple[int, str]:
        # 바닥(1층) 고유 ArUco 매핑
        base_aruco_map = {
            "A-1-1": "11",
            "A-1-2": "12",
            "A-2-1": "13",
            "A-2-2": "14",
            "A-3-1": "15",
            "A-3-2": "16"
        }
        
        existing_at_loc = [
            d for n, d in self.cargo_details.items()
            if self.cargo_registry.get(n) == location
        ]
        if not existing_at_loc:
            # 해당 구역에 화물이 없으면 1층 배정 및 바닥 고유 ArUco 반환
            return 1, base_aruco_map.get(location, "")
        # 최상단 화물을 찾아, 그 화물의 '컨테이너ID'를 새 화물의 '기반ArUco'로 상속
        top_cargo = max(existing_at_loc, key=lambda d: int(d.get("층수", "1")))
        max_floor = int(top_cargo.get("층수", "1"))
        inherited_aruco = top_cargo.get("컨테이너ID", "")
        
        return max_floor + 1, inherited_aruco

    def _execute_cargo_single(self, item: str, destination: str, target_aruco_id: str = "", target_floor: int = 1) -> None:
        """화물 하나를 목적지까지 이동시키고, 즉시 화물 등록부를 갱신/저장합니다. (언스택/리스택 포함)"""
        current_loc = self.cargo_registry.get(item)
        if not current_loc:
            self.result_label.configure(text=f"'{item}'의 현재 위치를 알 수 없습니다.", text_color="#EA5455")
            return
            
        current_floor = int(self.cargo_details.get(item, {}).get("층수", "1"))
        
        # 1. 대상 화물 위에 쌓인 화물들 찾기
        cargos_on_top = []
        for other_item, other_loc in self.cargo_registry.items():
            if other_loc == current_loc and other_item != item:
                other_floor = int(self.cargo_details.get(other_item, {}).get("층수", "1"))
                if other_floor > current_floor:
                    cargos_on_top.append((other_item, other_floor))
        
        if cargos_on_top:
            self._log(f"[Unstacking 감지] '{item}' 위에 {len(cargos_on_top)}개의 화물이 있어 임시 이동을 시작합니다.")
            # 층수가 높은 것부터 순서대로 (맨 위부터)
            cargos_on_top.sort(key=lambda x: x[1], reverse=True)
            
            # 임시 구역 찾기 (하역장/크레인/대기장소가 아닌 일반 적재구역 중, 최대 3층을 넘지 않는 곳)
            valid_locs = [loc for loc in self.locations.keys() if not any(kw in loc for kw in ["하역장", "회차", "크레인", "대기"])]
            
            # 각 위치별 현재 쌓여있는 층수(화물 개수) 파악
            loc_counts = {loc: 0 for loc in valid_locs}
            for l in self.cargo_registry.values():
                if l in loc_counts:
                    loc_counts[l] += 1
                    
            needed_space = len(cargos_on_top)
            # 현재 위치(current_loc)와 최종 목적지(destination)는 제외하고, 짐을 옮겨도 총 3층 이하인 곳을 후보로 선정
            candidate_locs = [loc for loc in valid_locs if loc_counts[loc] + needed_space <= 3 and loc not in (current_loc, destination)]
            
            # 만약 모두 꽉 차서 3층을 넘기게 된다면 최후의 수단으로 대기장소 사용
            temp_location = candidate_locs[0] if candidate_locs else "대기장소 1"
            self._log(f"[임시 구역 선정] '{temp_location}'으로 상단 화물들을 이동시킵니다.")
            
            # 2. 위에 있는 화물들을 임시 구역으로 이동
            moved_cargos = []
            for top_item, top_floor in cargos_on_top:
                temp_floor, temp_aruco = self._calculate_target_info(temp_location)
                self._log(f"-> {top_item}({top_floor}층) 화물을 임시구역 {temp_location}({temp_floor}층)으로 이동")
                self._execute_single_step(top_item, temp_location, temp_aruco, temp_floor, is_temp_move=True)
                moved_cargos.append((top_item, top_floor))
                
            # 3. 목표 화물 본래 목적지로 이동
            actual_target_floor, actual_target_aruco = target_floor, target_aruco_id
            if target_floor == 1 and not target_aruco_id:
                actual_target_floor, actual_target_aruco = self._calculate_target_info(destination)
                
            self._log(f"-> 목표 화물 '{item}'을(를) 최종 목적지 '{destination}'({actual_target_floor}층)으로 이동")
            self._execute_single_step(item, destination, actual_target_aruco, actual_target_floor, is_temp_move=False)
            
            # 4. 임시 구역에 뒀던 상단 화물들을 다시 원래 구역으로 원상복구
            self._log(f"[Restacking] 임시 구역에 둔 화물들을 다시 '{current_loc}'으로 복귀시킵니다.")
            # 다시 돌아올 때는 가장 아래(원래 층수가 낮았던 것)부터
            moved_cargos.sort(key=lambda x: x[1])
            
            for top_item, original_floor in moved_cargos:
                new_floor, new_aruco = self._calculate_target_info(current_loc)
                self._log(f"<- {top_item} 화물을 '{current_loc}'의 {new_floor}층으로 복귀")
                self._execute_single_step(top_item, current_loc, new_aruco, new_floor, is_temp_move=True)
                
            self.result_label.configure(
                text=f"[순차 이동 완료] 상단 화물 대피 및 '{item}' 이동 후 원상복구 완료",
                text_color="#28C76F",
            )
            
        else:
            # 위에 아무것도 없으면 그냥 바로 이동
            actual_target_floor, actual_target_aruco = target_floor, target_aruco_id
            if target_floor == 1 and not target_aruco_id:
                actual_target_floor, actual_target_aruco = self._calculate_target_info(destination)
            self._execute_single_step(item, destination, actual_target_aruco, actual_target_floor, is_temp_move=False)

    def _generate_auto_note(self, destination: str, target_floor: int, target_aruco_id: str) -> str:
        if target_floor == 1:
            return f"{destination} 바닥"
        
        # 2층 이상인 경우 Base ArUco(컨테이너ID)를 가진 화물 이름을 찾아 표시
        if target_aruco_id:
            for name, d in self.cargo_details.items():
                if d.get("컨테이너ID") == target_aruco_id and self.cargo_registry.get(name) == destination:
                    return f"{name} 위 적재"
        return f"{destination} {target_floor}층 적재"

    def _execute_single_step(self, item: str, destination: str, target_aruco_id: str = "", target_floor: int = 1, is_temp_move: bool = False) -> None:
        """실제로 화물 하나를 지정된 목적지와 층수로 옮기는 단일 작업입니다."""
        vehicle_idx, start_location = self._assign_next_vehicle()
        try:
            route = build_route(item, destination, self.cargo_registry,
                                standby_location=start_location, waypoint_rules=self.rules)
        except ValueError as exc:
            if not is_temp_move:
                self.result_label.configure(text=str(exc), text_color="#EA5455")
            else:
                self._log(f"[오류] 경로 생성 실패: {exc}")
            self.next_vehicle_index = vehicle_idx  # 롤백
            return

        is_crane_only = any(step.action == "크레인 전용 이동" for step in route)
        if is_crane_only:
            self.next_vehicle_index = vehicle_idx  # 배차 취소
            self._run_route_log(item, route)
            self.cargo_registry[item] = route[-1].location
            
            detail = self.cargo_details.get(item, {})
            detail["기반ArUco"] = target_aruco_id
            detail["층수"] = str(target_floor)
            detail["비고"] = self._generate_auto_note(route[-1].location, target_floor, target_aruco_id)
            self.cargo_details[item] = detail
            
            save_cargo_registry(self.cargo_registry)
            save_cargo_details(self.cargo_details)
            if self.on_cargo_updated:
                self.on_cargo_updated()
            
            route_text = " → ".join(
                f"{location_coord_text(step.location, self.locations)}[{step.action}]" for step in route
            )
            if not is_temp_move:
                self.result_label.configure(
                    text=f"[크레인 이동 완료] 화물 '{item}' → '{route[-1].location}'\n경로: {route_text}",
                    text_color="#28C76F",
                )
            
            # 중앙 서버(ROS)로 JSON 명령 전송 (크레인)
            try:
                from central_control_client import CentralControlClient, CentralControlApiError
                client = CentralControlClient()
                resp = client.send_cargo_dispatch(
                    item=item,
                    destination=route[-1].location,
                    target_floor=target_floor,
                    target_aruco_id=target_aruco_id,
                    is_temp_move=is_temp_move,
                    vehicle_idx=vehicle_idx,
                    is_crane_only=True
                )
                self._log(f"[API 전송 성공] 크레인 명령 ID: {resp.get('command_id', 'unknown')}")
            except Exception as e:
                self._log(f"[API 전송 실패] 중앙 제어 서버 미연결 상태입니다. (DB만 갱신됨): {e}")
                
            return

        self._log(f"[차량 {vehicle_idx + 1}] '{start_location}'에서 출발해 배차됨")
        self._run_route_log(item, route)

        agv_final_loc = route[-1].location
        for step in reversed(route):
            if step.action not in ["크레인 최종 이동", "크레인 전용 이동"]:
                agv_final_loc = step.location
                break

        self.cargo_registry[item] = route[-1].location
        
        detail = self.cargo_details.get(item, {})
        detail["기반ArUco"] = target_aruco_id
        detail["층수"] = str(target_floor)
        detail["비고"] = self._generate_auto_note(route[-1].location, target_floor, target_aruco_id)
        self.cargo_details[item] = detail
        
        self.vehicle_positions[vehicle_idx] = agv_final_loc
        save_cargo_registry(self.cargo_registry)
        save_cargo_details(self.cargo_details)
        save_vehicle_state(self.vehicle_positions, self.next_vehicle_index)
        record_vehicle_job(vehicle_idx, f"'{item}' → '{route[-1].location}' 이동 완료")
        self._log(f"[적용 완료] DB에 '{item}' -> '{route[-1].location}' (Base: {target_aruco_id}, {target_floor}층) 로 저장됨")

        # 중앙 서버(ROS)로 JSON 명령 전송 (AGV 이동 포함)
        try:
            from central_control_client import CentralControlClient, CentralControlApiError
            client = CentralControlClient()
            resp = client.send_cargo_dispatch(
                item=item,
                destination=route[-1].location,
                target_floor=target_floor,
                target_aruco_id=target_aruco_id,
                is_temp_move=is_temp_move,
                vehicle_idx=vehicle_idx,
                is_crane_only=False
            )
            self._log(f"[API 전송 성공] 차량 이동 명령 ID: {resp.get('command_id', 'unknown')}")
        except Exception as e:
            self._log(f"[API 전송 실패] 중앙 제어 서버 미연결 상태입니다. (DB만 갱신됨): {e}")

        self._return_used_vehicles_to_standby([vehicle_idx])

        if self.on_cargo_updated:
            self.on_cargo_updated()

        route_text = " → ".join(
            f"{location_coord_text(step.location, self.locations)}[{step.action}]" for step in route
        )
        if not is_temp_move:
            self.result_label.configure(
                text=f"[화물 이동 완료] 화물 '{item}' → '{route[-1].location}'\n경로: {route_text}",
                text_color="#28C76F",
            )

    def _move_items_to(self, items: List[str], destination: str, label: str) -> List[str]:
        """items에 있는 화물명들을 전부 destination으로 이동시키고, 실제로 이동에
        성공한 화물명 목록을 반환합니다 (여러 이동 헬퍼가 공통으로 재사용).
        차량 대수만큼은 대기장소에서 출발하고, 그 이후부터는 각 차량이 직전에
        마지막으로 도착한 위치에서 순서대로 돌아가며 출발합니다. 이 배치가 전부
        끝나면, 이번에 실제로 사용한 차량들을 대기장소로 복귀시킵니다."""
        self._log(f"{label} {len(items)}건을 '{destination}'로 이동 시작")

        moved = []
        used_vehicle_indices = set()
        for item in items:
            vehicle_idx, start_location = self._assign_next_vehicle()
            try:
                route = build_route(item, destination, self.cargo_registry,
                                    standby_location=start_location, waypoint_rules=self.rules)
            except ValueError as exc:
                self._log(f"[{item}] 경로 계산 실패 - 건너뜀: {exc}")
                self.next_vehicle_index = vehicle_idx  # 롤백
                continue
            
            is_crane_only = any(step.action == "크레인 전용 이동" for step in route)
            if is_crane_only:
                self.next_vehicle_index = vehicle_idx  # 배차 취소
                self._run_route_log(item, route)
                self.cargo_registry[item] = route[-1].location
                moved.append(item)
                continue

            self._log(f"[차량 {vehicle_idx + 1}] '{start_location}'에서 출발해 '{item}' 배차됨")
            self._run_route_log(item, route)

            agv_final_loc = route[-1].location
            for step in reversed(route):
                if step.action not in ["크레인 최종 이동", "크레인 전용 이동"]:
                    agv_final_loc = step.location
                    break

            self.cargo_registry[item] = route[-1].location
            self.vehicle_positions[vehicle_idx] = agv_final_loc
            record_vehicle_job(vehicle_idx, f"'{item}' → '{route[-1].location}' 이동 완료")
            used_vehicle_indices.add(vehicle_idx)
            moved.append(item)

        save_cargo_registry(self.cargo_registry)
        save_vehicle_state(self.vehicle_positions, self.next_vehicle_index)
        self._log(f"[적용 완료] {len(moved)}건 갱신됨: {', '.join(moved) or '없음'}")

        # 배치 전체가 끝났으니, 이번에 실제로 사용한 차량들을 대기장소로 복귀시킴
        self._return_used_vehicles_to_standby(used_vehicle_indices)

        if self.on_cargo_updated:
            self.on_cargo_updated()

        return moved

    def _execute_cargo_bulk(self, source_location: str, destination: str) -> None:
        """source_location에 있는 화물 전부를 destination으로 한 번에 이동시킵니다.
        예: "창고에 있는 물건 전부를 항만으로 이동" """
        matching_items = [name for name, loc in self.cargo_registry.items() if loc == source_location]

        if not matching_items:
            self.result_label.configure(
                text=f"'{source_location}'에 있는 화물이 없습니다.", text_color="#F59E0B",
            )
            return

        moved = self._move_items_to(matching_items, destination, f"'{source_location}'의 화물")

        self.result_label.configure(
            text=f"[일괄 이동 완료] '{source_location}' → '{destination}' ({len(moved)}건)\n"
                 f"이동된 화물: {', '.join(moved) or '없음'}",
            text_color="#28C76F",
        )

    def _execute_cargo_bulk_by_type(self, cargo_type: str, destination: str) -> None:
        """cargo_details.json의 "화물종류"가 일치하는 화물 전부를 destination으로 이동시킵니다.
        예: "컨테이너 화물은 항구로 이동" """
        matching_items = [
            name for name in self.cargo_registry
            if self.cargo_details.get(name, {}).get("화물종류") == cargo_type
        ]

        if not matching_items:
            self.result_label.configure(
                text=f"화물종류 '{cargo_type}'에 해당하는 화물이 없습니다.", text_color="#F59E0B",
            )
            return

        moved = self._move_items_to(matching_items, destination, f"화물종류 '{cargo_type}' 화물")

        self.result_label.configure(
            text=f"[화물종류 일괄 이동 완료] '{cargo_type}' → '{destination}' ({len(moved)}건)\n"
                 f"이동된 화물: {', '.join(moved) or '없음'}",
            text_color="#28C76F",
        )

    def _run_travel_command(self, command: str) -> None:
        """순수 위치 이동 명령 (규칙 기반 폴백 경로): 화물 데이터는 건드리지 않습니다."""
        stops = extract_travel_sequence(command, self.locations.keys())

        if not stops:
            available = ", ".join(self.locations.keys()) or "없음"
            self.result_label.configure(
                text=f"명령에서 등록된 위치를 찾지 못했습니다. (등록된 위치: {available})",
                text_color="#EA5455",
            )
            return

        if len(stops) > 1:
            self._log(f"[명령 해석] '{command}' -> 위치 이동, {len(stops)}개 지점: {' → '.join(stops)}")
        else:
            self._log(f"[명령 해석] '{command}' -> 위치 이동")

        self._execute_travel(stops)

    def _execute_travel(self, stops: List[str]) -> None:
        """Resolve a route and publish it to the central ROS navigation bridge."""
        if len(stops) == 1:
            travel_names = resolve_travel_route(stops[0], self.rules)
        else:
            travel_names = resolve_multi_stop_route(stops, self.rules)

        route = [RouteStep(name, "경유") for name in travel_names]
        if route:
            route[-1].action = "도착"

        self._run_route_log("AGV", route)
        waypoint_values = []
        missing_coordinates = []
        for step in route:
            entry = self.locations.get(step.location, {})
            map_xy = entry.get("map_meters")
            if not isinstance(map_xy, list) or len(map_xy) != 2:
                missing_coordinates.append(step.location)
                continue
            waypoint_values.append((float(map_xy[0]), float(map_xy[1]), 0.0))

        route_text = " → ".join(
            f"{location_coord_text(step.location, self.locations)}[{step.action}]" for step in route
        )
        if missing_coordinates:
            self.result_label.configure(
                text=(
                    "[전송 실패] map 좌표가 없는 위치: "
                    f"{', '.join(missing_coordinates)}"
                ),
                text_color="#EA5455",
            )
            return

        bridge = RosControlBridge.get_instance()
        if bridge.send_waypoints(waypoint_values):
            topic = "/central/target_map_waypoints"
            self._log(f"[ROS 전송] {topic}에 목표를 발행했습니다.")
            self.result_label.configure(
                text=f"[위치 이동 전송 완료] 경로: {route_text}",
                text_color="#28C76F",
            )
        else:
            self.result_label.configure(
                text=(
                    "[ROS 연결 실패] 경로는 계산했지만 차량에 전송하지 못했습니다.\n"
                    f"경로: {route_text}"
                ),
                text_color="#EA5455",
            )

    def _run_route_log(self, label: str, route: List[RouteStep]) -> None:
        """경로의 각 구간을 로그에 순서대로 남기고, 마지막에 전체 경로를 한 문장으로 요약합니다."""
        for i, step in enumerate(route):
            if i == 0:
                self._log(f"[{label}] 이동 시작 - {step.location} 에서 출발")
            else:
                prev = route[i - 1]
                self._log(f"[{label}] {prev.location} → {step.location} ({step.action})")
        self._log(describe_route_sentence(label, route))

    def _return_used_vehicles_to_standby(self, used_vehicle_indices) -> None:
        """이번 배차에서 실제로 쓰인 차량들을 전부 각자의 대기 위치로 복귀시킵니다.
        (실차 연동 시에는 이 시점에 각 차량에게 대기 위치 좌표로 이동하라는
        명령(send_nav_goal)을 내리면 됩니다 - location_to_nav_goal.py 참고)"""
        any_returned = False
        for vehicle_idx in sorted(set(used_vehicle_indices)):
            current = self.vehicle_positions[vehicle_idx]
            if "창고 하역장" in current:
                self._log(f"[차량 {vehicle_idx + 1}] 창고 하역장 정차 유지 (대기장소 복귀 생략)")
                continue

            route = build_return_to_standby_route(current, vehicle_idx, self.rules)
            if not route:
                continue  # 이미 자기 대기 위치에 있어서 복귀할 필요 없음
            self._log(f"[차량 {vehicle_idx + 1}] 모든 작업 종료 - 대기 위치로 복귀 지시")
            self._run_route_log(f"차량 {vehicle_idx + 1} 복귀", route)
            self.vehicle_positions[vehicle_idx] = vehicle_home_location(vehicle_idx)
            any_returned = True

        if any_returned:
            save_vehicle_state(self.vehicle_positions, self.next_vehicle_index)

    # ------------------------------------------------------------------
    def _log(self, text: str) -> None:
        self.log_box.configure(state="normal")
        timestamp = time.strftime("%H:%M:%S")
        self.log_box.insert("end", f"[{timestamp}] {text}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")


def open_command_popup(master, on_cargo_updated=None, initial_text=None):
    """각 화면의 "명령" 버튼에서 호출하는 진입점. 매번 새 팝업을 띄웁니다.
    on_cargo_updated를 넘기면, 화물 위치가 실제로 바뀔 때마다 그 함수가 호출됩니다.
    initial_text를 넘기면, 팝업이 열릴 때 명령 입력창에 그 문구가 미리 채워집니다
    (예: 화물 목록에서 특정 화물의 "🗣️" 버튼을 눌렀을 때 "화물A를 " 를 미리 채워주는 식)."""
    # 권한 체크 (master가 AGVControlCenter 내부에 있을 때만 작동)
    try:
        app = master.winfo_toplevel()
        if hasattr(app, "current_user_id") and app.current_user_id:
            role = app.USERS.get(app.current_user_id, {}).get("role", "")
            if role != "최고 관리자 (Admin)":
                from tkinter import messagebox
                messagebox.showerror("접근 거부", "자율주행 및 로봇 제어 명령 권한이 없습니다.\n(최고 관리자 전용 기능)", parent=master)
                return None
    except Exception:
        pass
        
    return CommandPopup(master, on_cargo_updated=on_cargo_updated, initial_text=initial_text)


# -----------------------------------------------------------------------------
# 단독 실행 (테스트용 - 팝업만 따로 띄워보기)
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")

    root = ctk.CTk()
    root.title("명령 팝업 테스트")
    root.geometry("400x200")
    ctk.CTkButton(root, text="명령", command=lambda: open_command_popup(root)).pack(pady=80)

    root.mainloop()
