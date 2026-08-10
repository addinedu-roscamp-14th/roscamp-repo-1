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
    load_named_locations,
    location_coord_text,
)
from llm_command_parser import (
    LLMParseError,
    parse_command_with_llm,
    resolve_execution_mode,
)
from ros_control_bridge import RosControlBridge
from realtime_llm_agent import RealtimeLLMAgent
from visual_navigation import (
    VisualNavigationError,
    compact_detections,
    is_reciprocal_zone_exchange,
    resolve_detection_approach,
    select_nearest_visible_vehicle,
    validate_pixel_navigation,
    zone_mode_for_label,
)
from yolo_detection_client import (
    YoloDetectionClient,
    YoloDetectionError,
)

PointXY = Tuple[float, float]

VEHICLE_IDS = ("agv1", "agv2")


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
            text="비전 LLM이 현재 카메라와 YOLO 결과를 우선 해석합니다.\n"
                 "예: \"A-3 구역의 컨테이너를 B-1로 옮겨\" / \"노란 차를 항구로 보내\"",
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
                RealtimeLLMAgent.get_instance().set_objective(
                    command,
                    llm_result.get('actions'),
                )
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

    def _extract_vehicle_id(self, action: Dict) -> str:
        """LLM이 지정한 vehicle_id를 검증합니다. agv1/agv2가 아니면 자동배차(빈 문자열)."""
        vehicle_id = str(action.get("vehicle_id") or "").strip().lower()
        if vehicle_id not in VEHICLE_IDS:
            if vehicle_id:
                self._log(
                    f'[VLM 차량 지정 무시] "{vehicle_id}"는 agv1/agv2가 아니라 자동배차로 전환'
                )
            return ""
        return vehicle_id

    def _handle_llm_result(
        self,
        command: str,
        result: Dict,
        detection_summary=None,
        image_width=640,
        image_height=480,
    ) -> bool:
        """LLM 응답을 검증하고 의존성에 따라 병렬 또는 순차 전송합니다.
        한 문장에 지시가 여러 개 섞여 있으면 actions에 여러 개가 들어오고, 전부 실행합니다.
        하나도 실행하지 못했으면 False를 반환해서 규칙 기반 파서로 넘어가게 합니다."""
        actions = result.get("actions")
        if not isinstance(actions, list) or not actions:
            return False

        any_handled = False
        plan_context = {'predecessor_command_id': ''}
        execution_mode = resolve_execution_mode(command, result)
        exchange_plan = is_reciprocal_zone_exchange(
            actions,
            detection_summary,
            RosControlBridge.get_instance().snapshot().b1_zone,
        )
        parallel_plan = exchange_plan or execution_mode == 'parallel'
        if exchange_plan:
            self._log(
                '[VLM 구역 교환 계획] 두 차량이 서로의 구역을 점유 중이므로 '
                '대기점 이탈을 병렬 실행하고 최종 정차까지 계속합니다.'
            )
        elif parallel_plan and len(actions) > 1:
            self._log(
                f'[VLM 동시 계획] 독립적인 {len(actions)}개 차량 명령을 '
                '선행 명령 연결 없이 동시에 전송합니다.'
            )
        elif len(actions) > 1:
            self._log(
                f'[VLM 순차 계획] {len(actions)}개 단계를 앞 단계 성공 후 '
                '순서대로 실행합니다.'
            )
        for step_index, action in enumerate(actions, start=1):
            if len(actions) > 1:
                self._log(
                    f'[VLM 계획 단계 {step_index}/{len(actions)}] {action}'
                )
            if (
                isinstance(action, dict)
                and self._handle_single_action(
                    command,
                    action,
                    detection_summary,
                    image_width,
                    image_height,
                    (
                        {'predecessor_command_id': ''}
                        if parallel_plan
                        else plan_context
                    ),
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
        plan_context=None,
    ) -> bool:
        """actions 배열 안의 액션 하나를 검증하고 실행합니다.
        검증에 실패하면 왜 실패했는지 로그에 남깁니다 - 여러 액션 중 하나가 조용히
        빠지면 사용자가 원인을 알기 어려우므로, 실패 이유를 항상 보이게 합니다."""
        action_type = action.get("type")
        if plan_context is None:
            plan_context = {'predecessor_command_id': ''}

        if action_type == "cargo_single":
            item = action.get("item")
            destination = action.get("destination")
            if not item:
                self._log(f"[액션 무시] 화물명이 비어있습니다: {action}")
                return False
            if destination not in self.locations:
                self._log(f"[액션 무시] 목적지 '{destination}'이 등록된 위치 목록에 없습니다: {action}")
                return False

            if item not in self.cargo_registry:
                # 미등록 화물 → 목적지에 자동 등록하고 바로 배치 완료 처리
                self._log(f"[자동 등록] 화물 '{item}'이 등록부에 없어 '{destination}'에 신규 등록합니다.")
                self.cargo_registry[item] = destination
                save_cargo_registry(self.cargo_registry)
                if self.on_cargo_updated:
                    self.on_cargo_updated()
                self.result_label.configure(
                    text=f"[신규 등록 완료] 화물 '{item}' → '{destination}'에 등록되었습니다.",
                    text_color="#28C76F",
                )
                return True

            self._log(f"[LLM 해석] '{command}' -> 화물={item}, 목적지={destination}")
            self._execute_cargo_single(item, destination)
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

        if action_type == "travel":
            stops = [s for s in (action.get("stops") or []) if s in self.locations]
            if not stops:
                self._log(f"[액션 무시] 등록된 위치와 일치하는 지점이 없습니다: {action}")
                return False
            self._log(f"[LLM 해석] '{command}' -> 위치 이동: {' → '.join(stops)}")
            self._execute_travel(stops)
            return True

        if action_type == "visual_transfer":
            return self._execute_visual_transfer(
                command,
                action,
                detection_summary,
                image_width,
                image_height,
                plan_context,
            )

        if action_type == "visual_navigation":
            if detection_summary is None:
                self._log(
                    '[VLM 객체 접근 거부] 최신 YOLO 검출 JSON이 없습니다.'
                )
                return True
            vehicle_id = self._extract_vehicle_id(action)
            try:
                target, heading, selected = resolve_detection_approach(
                    action,
                    detection_summary,
                    image_width,
                    image_height,
                )
                mode = zone_mode_for_label(selected['label'])
                # resolve_detection_approach() already ran the overlap
                # check above (validate_pixel_navigation): if we got here
                # without VisualNavigationError, no other detection sits on
                # this zone's target point in the current frame.
                zone_visually_empty = mode in ('parking_b1', 'parking_a')
                response = CentralControlClient().send_pixel_goal(
                    target,
                    heading,
                    predecessor_command_id=plan_context.get(
                        'predecessor_command_id', ''
                    ),
                    mode=mode,
                    vehicle_id=vehicle_id,
                    zone_visually_empty=zone_visually_empty,
                    queue_if_busy=bool(
                        plan_context.get('predecessor_command_id')
                    ),
                )
            except (VisualNavigationError, CentralControlApiError) as exc:
                self._log(f'[VLM 객체 접근 실패] {exc}')
                self.result_label.configure(
                    text=f'[VLM 객체 접근 실패] {exc}',
                    text_color='#EA5455',
                )
                return True

            mode_text = {
                'parking_b1': '주차',
                'parking_a': '화물 적재 대기',
            }.get(mode, action.get('approach_side'))
            command_id = response.get('command_id', 'unknown')
            if command_id != 'unknown':
                plan_context['predecessor_command_id'] = command_id
            self._log(
                '[VLM 객체 접근 좌표 계산] '
                f'index={selected["detection_index"]}, '
                f'label={selected["label"]}, '
                f'mode={"approach" if mode == "direct" else mode}, '
                f'side={action.get("approach_side")}, '
                f'vehicle={vehicle_id or "AUTO"}, '
                f'target={target}, heading={heading}, '
                f'command_id={command_id}'
            )
            self.result_label.configure(
                text=(
                    '[VLM 객체 접근 명령 전송 완료]\n'
                    f'객체={selected["label"]}, '
                    f'방식={mode_text}, '
                    f'차량={vehicle_id or "자동배차"}\n'
                    f'목표={target}, 방향={heading}'
                ),
                text_color='#28C76F',
            )
            return True

        if action_type == "pixel_navigation":
            target = action.get("target")
            heading = action.get("heading")
            vehicle_id = self._extract_vehicle_id(action)
            try:
                validate_pixel_navigation(
                    target,
                    heading,
                    image_width,
                    image_height,
                    detection_summary,
                )
                response = CentralControlClient().send_pixel_goal(
                    target,
                    heading,
                    predecessor_command_id=plan_context.get(
                        'predecessor_command_id', ''
                    ),
                    vehicle_id=vehicle_id,
                    queue_if_busy=bool(
                        plan_context.get('predecessor_command_id')
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
            if command_id != 'unknown':
                plan_context['predecessor_command_id'] = command_id
            duplicate = bool(response.get("duplicate", False))
            self._log(
                f"[VLM 좌표 전송] vehicle={vehicle_id or 'AUTO'}, "
                f"target={target}, heading={heading}, "
                f"command_id={command_id}, duplicate={duplicate}"
            )
            self.result_label.configure(
                text=(
                    "[VLM 차량 이동 명령 전송 완료]\n"
                    f"차량={vehicle_id or '자동배차'}\n"
                    f"목표={target}, 방향={heading}\n"
                    f"명령 ID={command_id}"
                ),
                text_color="#28C76F",
            )
            return True

        if action_type == "park_command":
            vehicle_id = self._extract_vehicle_id(action)
            try:
                response = CentralControlClient().send_park(
                    vehicle_id=vehicle_id
                )
            except CentralControlApiError as exc:
                self._log(f"[자동 주차 실패] {exc}")
                self.result_label.configure(
                    text=f"[자동 주차 실패] {exc}",
                    text_color="#EA5455",
                )
                return True

            command_id = response.get("command_id", "unknown")
            self._log(
                f"[자동 주차 명령 전송] vehicle={vehicle_id or 'AUTO'}, "
                f"command_id={command_id}"
            )
            self.result_label.configure(
                text=(
                    "[자동 주차 명령 전송 완료]\n"
                    f"차량={vehicle_id or '자동배차'}"
                ),
                text_color="#28C76F",
            )
            return True

        # type == "unknown" 이거나 예상 밖의 값이면 처리 못 한 것으로 간주
        return False

    def _execute_visual_transfer(
        self,
        command,
        action,
        detection_summary,
        image_width,
        image_height,
        plan_context,
    ):
        """Dispatch one live AGV through the visible source and destination."""
        if detection_summary is None:
            self._log('[실시간 운송 거부] 최신 YOLO 검출 JSON이 없습니다.')
            return True

        source_action = {
            'detection_index': action.get('source_detection_index'),
            'approach_side': 'bottom',
        }
        destination_action = {
            'detection_index': action.get('destination_detection_index'),
            'approach_side': 'bottom',
        }
        try:
            source_target, source_heading, source = resolve_detection_approach(
                source_action,
                detection_summary,
                image_width,
                image_height,
            )
            destination_target, destination_heading, destination = (
                resolve_detection_approach(
                    destination_action,
                    detection_summary,
                    image_width,
                    image_height,
                )
            )
            vehicle_id = self._select_live_transfer_vehicle(
                action,
                source,
                detection_summary,
            )
            if not vehicle_id:
                raise VisualNavigationError(
                    '현재 프레임과 Fleet 상태에서 운송 차량을 선택할 수 없습니다'
                )

            client = CentralControlClient()
            source_response = client.send_pixel_goal(
                source_target,
                source_heading,
                predecessor_command_id=plan_context.get(
                    'predecessor_command_id', ''
                ),
                mode=zone_mode_for_label(source['label']),
                vehicle_id=vehicle_id,
                zone_visually_empty=True,
                queue_if_busy=bool(
                    plan_context.get('predecessor_command_id')
                ),
            )
            source_command_id = source_response.get('command_id', '')
            if not source_command_id:
                raise CentralControlApiError(
                    '출발 구역 이동 명령 ID를 받지 못했습니다'
                )
            destination_response = client.send_pixel_goal(
                destination_target,
                destination_heading,
                predecessor_command_id=source_command_id,
                mode=zone_mode_for_label(destination['label']),
                vehicle_id=vehicle_id,
                zone_visually_empty=True,
                queue_if_busy=True,
            )
        except (VisualNavigationError, CentralControlApiError) as exc:
            self._log(f'[실시간 운송 계획 실패] {exc}')
            self.result_label.configure(
                text=f'[실시간 운송 계획 실패] {exc}',
                text_color='#EA5455',
            )
            return True

        destination_command_id = destination_response.get(
            'command_id', 'unknown'
        )
        if destination_command_id != 'unknown':
            plan_context['predecessor_command_id'] = destination_command_id
        self._log(
            '[실시간 운송 계획 전송] '
            f'현재 차량={vehicle_id}, '
            f'{source["label"]}({source_command_id}) → '
            f'{destination["label"]}({destination_command_id})'
        )
        self.result_label.configure(
            text=(
                '[현재 화면 기준 운송 경로 전송 완료]\n'
                f'차량={vehicle_id}, '
                f'{source["label"]} 도착 후 {destination["label"]} 이동\n'
                '저장된 화물 위치 데이터는 변경하지 않았습니다.'
            ),
            text_color='#28C76F',
        )
        return True

    def _select_live_transfer_vehicle(
        self,
        action,
        source_detection,
        detection_summary,
    ):
        requested = self._extract_vehicle_id(action)
        if requested:
            return requested

        snapshot = RosControlBridge.get_instance().snapshot()
        states = {
            vehicle.vehicle_id: vehicle.state_text
            for vehicle in snapshot.fleet_states
            if not vehicle.emergency_stopped
        }
        ready = {
            vehicle_id for vehicle_id, state in states.items()
            if state == 'READY'
        }
        operational = {
            vehicle_id for vehicle_id, state in states.items()
            if state in {'READY', 'BUSY'}
        }
        eligible = ready or operational or None
        selected = select_nearest_visible_vehicle(
            source_detection,
            detection_summary,
            eligible,
        )
        if not selected and ready:
            selected = sorted(ready)[0]
        if not selected and operational:
            selected = sorted(operational)[0]
        if not selected and not states:
            selected = select_nearest_visible_vehicle(
                source_detection,
                detection_summary,
            )
        if selected:
            self._log(
                '[현재 상태 차량 선택] '
                f'{selected}, fleet_state={states.get(selected, "영상 기준")}'
            )
        return selected

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
        self._execute_cargo_single(item, destination)

    def _assign_next_vehicle(self) -> Tuple[int, str]:
        """다음 작업을 맡을 차량 번호와, 그 차량이 지금 있는 위치(출발점)를 정합니다.
        차량 대수(NUM_VEHICLES)만큼은 각자 마지막으로 도착한 곳에서 순서대로 돌아가며
        배차되고, 처음 두 대(대수만큼)는 전부 대기장소에서 시작합니다."""
        vehicle_idx = self.next_vehicle_index
        start_location = self.vehicle_positions[vehicle_idx]
        self.next_vehicle_index = (self.next_vehicle_index + 1) % len(self.vehicle_positions)
        return vehicle_idx, start_location

    def _execute_cargo_single(self, item: str, destination: str) -> None:
        """화물 하나를 목적지까지 이동시키고, 즉시 화물 등록부를 갱신/저장합니다."""
        vehicle_idx, start_location = self._assign_next_vehicle()
        try:
            route = build_route(item, destination, self.cargo_registry,
                                standby_location=start_location, waypoint_rules=self.rules)
        except ValueError as exc:
            self.result_label.configure(text=str(exc), text_color="#EA5455")
            self.next_vehicle_index = vehicle_idx  # 롤백: 차량 안 씀
            return

        is_crane_only = any(step.action == "크레인 전용 이동" for step in route)
        if is_crane_only:
            self.next_vehicle_index = vehicle_idx  # 배차 취소
            self._run_route_log(item, route)
            self.cargo_registry[item] = route[-1].location
            save_cargo_registry(self.cargo_registry)
            if self.on_cargo_updated:
                self.on_cargo_updated()
            
            route_text = " → ".join(
                f"{location_coord_text(step.location, self.locations)}[{step.action}]" for step in route
            )
            self.result_label.configure(
                text=f"[크레인 이동 완료] 화물 '{item}' → '{route[-1].location}'\n경로: {route_text}",
                text_color="#28C76F",
            )
            return

        self._log(f"[차량 {vehicle_idx + 1}] '{start_location}'에서 출발해 배차됨")
        self._run_route_log(item, route)

        agv_final_loc = route[-1].location
        for step in reversed(route):
            if step.action not in ["크레인 최종 이동", "크레인 전용 이동"]:
                agv_final_loc = step.location
                break

        self.cargo_registry[item] = route[-1].location
        self.vehicle_positions[vehicle_idx] = agv_final_loc
        save_cargo_registry(self.cargo_registry)
        save_vehicle_state(self.vehicle_positions, self.next_vehicle_index)
        record_vehicle_job(vehicle_idx, f"'{item}' → '{route[-1].location}' 이동 완료")
        self._log(f"[적용 완료] cargo_locations.json에 '{item}': '{route[-1].location}' 로 저장됨")

        # 이 배차(단건)가 끝났으니, 방금 쓴 차량을 대기장소로 복귀시킴
        self._return_used_vehicles_to_standby([vehicle_idx])

        if self.on_cargo_updated:
            self.on_cargo_updated()

        route_text = " → ".join(
            f"{location_coord_text(step.location, self.locations)}[{step.action}]" for step in route
        )
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


def open_command_popup(master, on_cargo_updated=None, initial_text=None) -> CommandPopup:
    """각 화면의 "명령" 버튼에서 호출하는 진입점. 매번 새 팝업을 띄웁니다.
    on_cargo_updated를 넘기면, 화물 위치가 실제로 바뀔 때마다 그 함수가 호출됩니다.
    initial_text를 넘기면, 팝업이 열릴 때 명령 입력창에 그 문구가 미리 채워집니다
    (예: 화물 목록에서 특정 화물의 "🗣️" 버튼을 눌렀을 때 "화물A를 " 를 미리 채워주는 식)."""
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
