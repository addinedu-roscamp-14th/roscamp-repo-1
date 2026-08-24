"""
dashboard_view.py

항만 전체 현황을 한눈에 보는 대시보드입니다. 팀원이 만든 dashboard_view.py의
구성(실시간 영상 + 상태 카드 + 기상청/해양조사원 API 연동)을 그대로 가져오되,
자연어 명령창은 팀원의 별도 LLM 서버(llm_bridge.py) 대신 우리 프로젝트에서
쓰던 "🗣️ 명령" 팝업(command_center.py)으로 통일했습니다.

CCTVMonitorView.SHARED_FRAME을 그대로 읽어와서 보여주기 때문에, "📷 CCTV 실시간
모니터링" 탭을 따로 열어두지 않아도(또는 열어뒀으면 그 화면과 동시에) 최신 영상이
여기에도 표시됩니다.
"""

import math
import threading
from datetime import datetime, timedelta

import customtkinter as ctk
import cv2
import requests
from PIL import Image

from cctv_monitor_view import CCTVMonitorView
from central_control_client import CentralControlClient
from realtime_llm_agent import RealtimeLLMAgent
from ros_control_bridge import (
    AMR_DISPLAY_NAMES,
    ARM_DISPLAY_NAMES,
    RosControlBridge,
)

# Tailwind Color Palette
BG_SURFACE = "#131314"
BG_PANEL = "#1b1b1c"
BG_CARD = "#2a2a2b"
BG_CARD_INNER = "#353436"
BORDER_COLOR = "#404850"
TEXT_PRIMARY = "#e5e2e3"
TEXT_SECONDARY = "#c0c7d1"
ACCENT_BLUE = "#92ccff"
ACCENT_GREEN = "#61de8a"
ACCENT_ORANGE = "#ee671c"
ACCENT_ON_BLUE = "#003351"
ALERT_RED = "#ff6b6b"

# Bottom-right vessel berth in the shared top-down camera image.
DEFAULT_ARRIVAL_ROI = [
    0.8424657534246576,
    0.6316695352839932,
    0.9575342465753425,
    0.8399311531841652,
]

# 기상청(KMA) / 해양조사원(KHOA) API 키 - 팀원이 쓰던 것을 그대로 재사용합니다.
_WEATHER_API_KEY = "7d6c81a1615ce0a0dccd7e35568f7c3714d3f37ae515ce17e25a065005419ae3"


# state_text values published by fleet_dispatcher, with what each one means
# for an operator. Keep in sync with fleet_dispatcher._publish_vehicle_states.
_STATE_DESCRIPTIONS = {
    "READY": ("대기 중 (명령 수신 가능)", ACCENT_GREEN),
    "BUSY": ("작업 수행 중", ACCENT_BLUE),
    "EMERGENCY_STOPPED": ("비상정지 상태", ALERT_RED),
    "OFFLINE": ("오프라인 (텔레메트리 끊김)", ALERT_RED),
    "WAITING_FOR_INITIAL_POSE": ("초기 위치 대기 (AMCL 미수렴)", ACCENT_ORANGE),
    "NAV2_INACTIVE": ("Nav2 비활성 (주행 불가)", ACCENT_ORANGE),
}

_ARM_STATE_DESCRIPTIONS = {
    0: ("미구성", ACCENT_ORANGE),
    1: ("연결 끊김", ALERT_RED),
    2: ("준비 완료", ACCENT_GREEN),
    3: ("작업 중", ACCENT_BLUE),
    4: ("정지됨", ACCENT_ORANGE),
    5: ("작업자 확인 대기", ACCENT_ORANGE),
    6: ("오류", ALERT_RED),
}


def _describe_state(vehicle):
    """Map a raw state_text to an operator-readable label and colour."""
    description, color = _STATE_DESCRIPTIONS.get(
        vehicle.state_text, (vehicle.state_text or "알 수 없음", TEXT_PRIMARY)
    )
    return f"{description}", color


def _stop_reasons(vehicle, snapshot, held_by_collision):
    """List every distinct reason this vehicle is currently held.

    The operator latch and the collision supervisor's safety_hold are separate
    mechanisms on the vehicle's cmd_vel gate, so both can be active at once and
    each needs its own release.
    """
    reasons = []
    if vehicle.emergency_stopped:
        reasons.append("🚨 운영자 비상정지 (수동 해제 필요)")
    if held_by_collision:
        distance = snapshot.collision_distance_m
        gap = f" · 간격 {distance:.2f}m" if distance is not None else ""
        transport = snapshot.collision_transport or "safety_hold"
        reasons.append(f"⛔ 충돌 회피 정지 ({transport}{gap})")
    if vehicle.state_text == "OFFLINE":
        reasons.append(
            f"📡 텔레메트리 {vehicle.telemetry_age_sec:.1f}초 지연"
        )
    return reasons


def _vehicle_detail(vehicle):
    """Secondary line: position, zone lock, Nav2 and the running command."""
    parts = [f"위치 ({vehicle.x:.2f}, {vehicle.y:.2f})"]
    parts.append("Nav2 준비" if vehicle.nav2_ready else "Nav2 미준비")
    if vehicle.locked_zone:
        parts.append(f"점유 구역 {vehicle.locked_zone}")
    if vehicle.current_command_id:
        parts.append(f"명령 {vehicle.current_command_id}")
    return " · ".join(parts)


class DashboardView(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color=BG_SURFACE, **kwargs)

        # CCTV 백그라운드 캡처가 아직 안 돌고 있으면 시작 (대시보드에서도 영상이 보이도록)
        CCTVMonitorView.ensure_capture_running()
        self.realtime_agent = RealtimeLLMAgent.get_instance()
        self.realtime_agent.start()

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=20, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=16, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=14)
        self.font_body_bold = ctk.CTkFont(family="Malgun Gothic", size=18, weight="bold") # For large stats
        self.font_mini = ctk.CTkFont(family="Malgun Gothic", size=12)
        self.font_data = ctk.CTkFont(family="Consolas", size=18, weight="bold")

        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0, minsize=320)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=0, minsize=176)

        # ==========================================
        # 1. 중앙: 실시간 영상 (CCTV 탭과 프레임 공유)
        # ==========================================
        self.map_frame = ctk.CTkFrame(self, fg_color="black", border_width=1, border_color=BORDER_COLOR, corner_radius=12)
        self.map_frame.grid(row=0, column=0, padx=(15, 10), pady=(15, 10), sticky="nsew")
        self.map_frame.grid_columnconfigure(0, weight=1)
        self.map_frame.grid_rowconfigure(1, weight=1)

        roi_toolbar = ctk.CTkFrame(
            self.map_frame,
            fg_color=BG_PANEL,
            corner_radius=0,
            height=44,
        )
        roi_toolbar.grid(row=0, column=0, sticky="ew")
        roi_toolbar.grid_propagate(False)
        ctk.CTkLabel(
            roi_toolbar,
            text="입항 감지 영역",
            text_color=TEXT_PRIMARY,
            font=self.font_body_bold,
        ).pack(side="left", padx=14)

        self.live_feed_label = ctk.CTkLabel(
            self.map_frame,
            text="[📷 실시간 영상 연동 대기 중]\n'설정 > 스트림 설정'에서 카메라 URL을 확인해주세요.",
            text_color=TEXT_SECONDARY, font=self.font_subtitle,
        )
        self.live_feed_label.grid(
            row=1, column=0, padx=5, pady=5, sticky="nsew"
        )
        self._arrival_roi = list(DEFAULT_ARRIVAL_ROI)
        self._roi_editing = False
        self._roi_drag_start = None
        self._roi_drag_current = None
        self._central_client = CentralControlClient()
        self.roi_button = ctk.CTkButton(
            roi_toolbar,
            text="선박 ROI 설정",
            width=120,
            height=30,
            command=self._toggle_roi_editor,
        )
        self.roi_button.pack(side="right", padx=10, pady=7)
        self.live_feed_label.bind("<ButtonPress-1>", self._roi_press)
        self.live_feed_label.bind("<B1-Motion>", self._roi_motion)
        self.live_feed_label.bind("<ButtonRelease-1>", self._roi_release)

        self._ui_alive = True
        self.update_live_feed()  # 주기적으로 CCTVMonitorView.SHARED_FRAME을 읽어와 표시

        # ==========================================
        # 2. 우측: 작동 현황 카드 + 명령 버튼
        # ==========================================
        status_frame = ctk.CTkFrame(self, fg_color=BG_PANEL, border_width=1, border_color=BORDER_COLOR, corner_radius=12)
        status_frame.grid(row=0, column=1, rowspan=2, padx=(0, 15), pady=(15, 10), sticky="nsew")

        title_frame = ctk.CTkFrame(status_frame, fg_color="transparent")
        title_frame.pack(side="top", fill="x", padx=15, pady=(15, 10))
        ctk.CTkLabel(title_frame, text="📊 실시간 작동 현황", font=self.font_subtitle, text_color=TEXT_PRIMARY).pack(side="left")

        # Reserve the command row before the cards. pack() hands out space in
        # call order, so packing this last let a tall status card push the
        # button clean off the panel.
        btn_row = ctk.CTkFrame(status_frame, fg_color="transparent")
        btn_row.pack(side="bottom", fill="x", padx=15, pady=(0, 15))

        self.btn_cmd = ctk.CTkButton(btn_row, text="🗣️ 명령", font=self.font_subtitle, fg_color=ACCENT_BLUE, text_color=ACCENT_ON_BLUE,
                                     hover_color="#cce5ff", height=40, command=self.open_command_popup)
        self.btn_cmd.pack(side="left", expand=True, fill="x", padx=(0, 8))

        self.btn_autonomy = ctk.CTkButton(
            btn_row,
            text="자율 관제모드",
            font=self.font_subtitle,
            height=40,
            command=self.toggle_autonomy_mode,
        )
        self.btn_autonomy.pack(side="left", expand=True, fill="x")
        self.update_autonomy_button()

        divider = ctk.CTkFrame(status_frame, height=1, fg_color=BORDER_COLOR)
        divider.pack(side="bottom", fill="x", padx=15, pady=(0, 10))

        # Cards scroll, so adding detail to a card can never squeeze the
        # button out again.
        cards_area = ctk.CTkScrollableFrame(status_frame, fg_color="transparent")
        cards_area.pack(side="top", fill="both", expand=True)

        self._build_amr_status_card(cards_area)
        self._build_arm_status_card(cards_area)
        self._build_autonomy_status_card(cards_area)

        cards = [
            ("당일 선박 입항", "총 5척 정박 완료"),
        ]
        for title, val in cards:
            card = ctk.CTkFrame(cards_area, fg_color=BG_CARD, border_width=1, border_color=BORDER_COLOR, corner_radius=8)
            card.pack(fill="x", padx=5, pady=5, ipady=8)
            ctk.CTkLabel(card, text=title, font=self.font_mini, text_color=TEXT_SECONDARY).pack(anchor="w", padx=10, pady=(0, 2))
            ctk.CTkLabel(card, text=val, font=self.font_body_bold if "가동" in val or "완료" in val else self.font_body, text_color=TEXT_PRIMARY).pack(anchor="w", padx=10)

        # ==========================================
        # 3. 하단: 기상 및 해양 상황 패널
        # ==========================================
        weather_frame = ctk.CTkFrame(self, fg_color=BG_PANEL, border_width=1, border_color=BORDER_COLOR, corner_radius=12)
        weather_frame.grid(row=1, column=0, padx=(15, 10), pady=(0, 10), sticky="nsew")

        header_frame = ctk.CTkFrame(weather_frame, fg_color="transparent")
        header_frame.pack(fill="x", padx=15, pady=(12, 8))
        ctk.CTkLabel(header_frame, text="🌊 실시간 해양 기상 정보", font=self.font_subtitle, text_color=TEXT_PRIMARY).pack(side="left")

        self.lbl_kma_status = ctk.CTkLabel(header_frame, text="[KMA 요청 중...]", font=self.font_mini, text_color="#F59E0B")
        self.lbl_kma_status.pack(side="left", padx=(15, 5))
        self.lbl_khoa_status = ctk.CTkLabel(header_frame, text="[KHOA 요청 중...]", font=self.font_mini, text_color="#F59E0B")
        self.lbl_khoa_status.pack(side="left", padx=5)

        w_data_frame = ctk.CTkFrame(weather_frame, fg_color="transparent")
        w_data_frame.pack(fill="both", expand=True, padx=15, pady=(0, 15))

        w_items = [("기온", "-- °C", ACCENT_GREEN), ("풍속", "-- m/s", ACCENT_ORANGE), ("풍향", "-- °", TEXT_PRIMARY), ("파고 및 조위", "조위: -- cm\n파고: -- m", ACCENT_BLUE)]
        
        # Configure columns to stretch equally
        for i in range(4):
            w_data_frame.grid_columnconfigure(i, weight=1)

        self.weather_labels = {}
        for idx, (title, val, color) in enumerate(w_items):
            box = ctk.CTkFrame(w_data_frame, fg_color=BG_CARD_INNER, border_width=1, border_color=BORDER_COLOR, corner_radius=8)
            box.grid(row=0, column=idx, padx=5, sticky="nsew")
            
            # Center content in the box
            inner_frame = ctk.CTkFrame(box, fg_color="transparent")
            inner_frame.pack(expand=True)
            
            ctk.CTkLabel(inner_frame, text=title, font=self.font_mini, text_color=TEXT_SECONDARY).pack(pady=(5, 2))
            value_label = ctk.CTkLabel(inner_frame, text=val, font=self.font_data if title != "파고 및 조위" else self.font_body, text_color=color)
            value_label.pack(pady=(0, 5))
            self.weather_labels[title] = value_label

        threading.Thread(target=self.fetch_weather_kma, daemon=True).start()
        threading.Thread(target=self.fetch_ocean_khoa, daemon=True).start()

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # 자율주행 차량(AMR) 실시간 상태 카드
    # ------------------------------------------------------------------
    def _build_amr_status_card(self, parent) -> None:
        """Live per-AMR state, battery and emergency-stop indicator.

        Fed by RosControlBridge, which subscribes to
        /central/fleet/<vehicle_id>/state, so this reflects what the fleet
        dispatcher actually sees rather than a hardcoded summary.
        """
        self.ros_bridge = RosControlBridge.get_instance()

        card = ctk.CTkFrame(parent, fg_color=BG_CARD, border_width=1, border_color=BORDER_COLOR, corner_radius=8)
        card.pack(fill="x", padx=5, pady=5, ipady=8)

        header = ctk.CTkFrame(card, fg_color="transparent")
        header.pack(fill="x", padx=10, pady=(0, 4))
        ctk.CTkLabel(header, text="자율주행 차량 (AMR)", font=self.font_mini, text_color=TEXT_SECONDARY).pack(side="left")
        self.lbl_amr_link = ctk.CTkLabel(header, text="ROS 연결 중", font=self.font_mini, text_color=ACCENT_ORANGE)
        self.lbl_amr_link.pack(side="right")

        self.amr_state_labels = {}
        self.amr_stop_labels = {}
        self.amr_detail_labels = {}
        for vehicle_id, display_name in AMR_DISPLAY_NAMES.items():
            row = ctk.CTkFrame(card, fg_color=BG_CARD_INNER, corner_radius=6)
            row.pack(fill="x", padx=10, pady=3, ipady=4)
            ctk.CTkLabel(row, text=display_name, font=self.font_mini, text_color=TEXT_SECONDARY).pack(anchor="w", padx=8)

            state_label = ctk.CTkLabel(row, text="상태 수신 대기", font=self.font_body, text_color=TEXT_SECONDARY)
            state_label.pack(anchor="w", padx=8)
            self.amr_state_labels[vehicle_id] = state_label

            stop_label = ctk.CTkLabel(row, text="", font=self.font_mini, text_color=ALERT_RED, justify="left")
            self.amr_stop_labels[vehicle_id] = stop_label

            detail_label = ctk.CTkLabel(row, text="", font=self.font_mini, text_color=TEXT_SECONDARY, justify="left")
            detail_label.pack(anchor="w", padx=8)
            self.amr_detail_labels[vehicle_id] = detail_label

        self.lbl_estop = ctk.CTkLabel(card, text="비상정지 --", font=self.font_mini, text_color=TEXT_SECONDARY)
        self.lbl_estop.pack(anchor="w", padx=10, pady=(4, 0))

        self.update_amr_status()

    def _build_arm_status_card(self, parent) -> None:
        """Build live connection and task rows for ARM1 and ARM2."""
        card = ctk.CTkFrame(
            parent,
            fg_color=BG_CARD,
            border_width=1,
            border_color=BORDER_COLOR,
            corner_radius=8,
        )
        card.pack(fill="x", padx=5, pady=5, ipady=8)

        header = ctk.CTkFrame(card, fg_color="transparent")
        header.pack(fill="x", padx=10, pady=(0, 4))
        ctk.CTkLabel(
            header,
            text="로봇팔 (ARM)",
            font=self.font_mini,
            text_color=TEXT_SECONDARY,
        ).pack(side="left")
        self.lbl_arm_link = ctk.CTkLabel(
            header,
            text="상태 수신 대기",
            font=self.font_mini,
            text_color=ACCENT_ORANGE,
        )
        self.lbl_arm_link.pack(side="right")

        self.arm_state_labels = {}
        self.arm_detail_labels = {}
        self.arm_error_labels = {}
        for arm_id, display_name in ARM_DISPLAY_NAMES.items():
            row = ctk.CTkFrame(card, fg_color=BG_CARD_INNER, corner_radius=6)
            row.pack(fill="x", padx=10, pady=3, ipady=4)
            ctk.CTkLabel(
                row,
                text=display_name,
                font=self.font_mini,
                text_color=TEXT_SECONDARY,
            ).pack(anchor="w", padx=8)
            state_label = ctk.CTkLabel(
                row,
                text="연결 상태 확인 중",
                font=self.font_body,
                text_color=TEXT_SECONDARY,
                justify="left",
                wraplength=270,
            )
            state_label.pack(anchor="w", padx=8)
            self.arm_state_labels[arm_id] = state_label

            detail_label = ctk.CTkLabel(
                row,
                text="",
                font=self.font_mini,
                text_color=TEXT_SECONDARY,
                justify="left",
                wraplength=270,
            )
            detail_label.pack(anchor="w", padx=8)
            self.arm_detail_labels[arm_id] = detail_label

            error_label = ctk.CTkLabel(
                row,
                text="",
                font=self.font_mini,
                text_color=ALERT_RED,
                justify="left",
                wraplength=270,
            )
            self.arm_error_labels[arm_id] = error_label

        self.update_arm_status()

    def _build_autonomy_status_card(self, parent) -> None:
        """Show the durable DB gate and current autonomous cycle."""
        card = ctk.CTkFrame(
            parent, fg_color=BG_CARD, border_width=1,
            border_color=BORDER_COLOR, corner_radius=8,
        )
        card.pack(fill='x', padx=5, pady=5, ipady=8)
        ctk.CTkLabel(
            card, text='DB 기반 자율 관제', font=self.font_mini,
            text_color=TEXT_SECONDARY,
        ).pack(anchor='w', padx=10)
        self.lbl_autonomy_detail = ctk.CTkLabel(
            card, text='중지됨', font=self.font_mini,
            text_color=TEXT_SECONDARY, justify='left', wraplength=270,
        )
        self.lbl_autonomy_detail.pack(anchor='w', padx=10, pady=(3, 0))
        self.txt_autonomy_commands = ctk.CTkTextbox(
            card,
            height=240,
            font=self.font_mini,
            fg_color=BG_SURFACE,
            text_color=TEXT_SECONDARY,
            wrap='word',
        )
        self.txt_autonomy_commands.pack(
            fill='x', padx=10, pady=(8, 0)
        )
        self.txt_autonomy_commands.insert(
            '1.0', 'LLM 판단 및 전송 명령 대기 중'
        )
        self.txt_autonomy_commands.configure(state='disabled')
        self._last_autonomy_command_text = ''

    def update_arm_status(self) -> None:
        """Render structured central ARM telemetry every 500 ms."""
        if not self._ui_alive:
            return
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        snapshot = self.ros_bridge.snapshot()
        states = {arm.arm_id: arm for arm in snapshot.arm_states}
        if states:
            self.lbl_arm_link.configure(
                text="중앙 상태 수신됨", text_color=ACCENT_GREEN
            )
        elif snapshot.error:
            self.lbl_arm_link.configure(
                text="ROS 연결 실패", text_color=ALERT_RED
            )
        else:
            self.lbl_arm_link.configure(
                text="상태 수신 대기", text_color=ACCENT_ORANGE
            )

        raw_status = {
            "arm1": snapshot.arm_status,
            "arm2": snapshot.arm2_status,
        }
        for arm_id in ARM_DISPLAY_NAMES:
            state_label = self.arm_state_labels[arm_id]
            detail_label = self.arm_detail_labels[arm_id]
            error_label = self.arm_error_labels[arm_id]
            arm = states.get(arm_id)
            if arm is None:
                raw = raw_status.get(arm_id)
                if raw:
                    state_label.configure(
                        text="직접 상태 토픽 연결됨",
                        text_color=ACCENT_GREEN,
                    )
                    detail_label.configure(text=str(raw))
                else:
                    state_label.configure(
                        text="연결 상태 수신 없음",
                        text_color=TEXT_SECONDARY,
                    )
                    detail_label.configure(text="")
                error_label.configure(text="")
                error_label.pack_forget()
                continue

            label, color = _ARM_STATE_DESCRIPTIONS.get(
                arm.state,
                (arm.state_text or "알 수 없음", TEXT_PRIMARY),
            )
            raw_state = arm.state_text.strip()
            state_text = label
            if raw_state and raw_state.lower() != label.lower():
                state_text = f"{label} · {raw_state}"
            raw = raw_status.get(arm_id)
            if arm.state == 0 and raw:
                state_text = "직접 상태 연결 · 중앙 제어 미구성"
            state_label.configure(text=state_text, text_color=color)

            details = []
            if arm.current_operation:
                details.append(f"작업: {arm.current_operation}")
            else:
                details.append("작업: 없음")
            if arm.phase:
                details.append(f"단계: {arm.phase}")
            if arm.current_operation or arm.state == 3:
                progress = min(100.0, max(0.0, arm.progress))
                details.append(f"진행률: {progress:.0f}%")
            if arm.current_command_id:
                details.append(f"명령: {arm.current_command_id}")
            if arm.current_mission_id:
                details.append(f"미션: {arm.current_mission_id}")
            if math.isfinite(arm.telemetry_age_sec):
                details.append(f"텔레메트리: {arm.telemetry_age_sec:.1f}초 전")
            if raw and (arm.state in {0, 1} or not arm.current_operation):
                details.append(f"직접 상태: {raw}")
            detail_label.configure(text="\n".join(details))

            if arm.last_error:
                error_label.configure(text=f"오류: {arm.last_error}")
                error_label.pack(anchor="w", padx=8)
            else:
                error_label.configure(text="")
                error_label.pack_forget()

        self.after(500, self.update_arm_status)

    def update_amr_status(self) -> None:
        if not self._ui_alive:
            return
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        snapshot = self.ros_bridge.snapshot()
        if snapshot.ready:
            self.lbl_amr_link.configure(text="ROS 연결됨", text_color=ACCENT_GREEN)
        elif snapshot.error:
            self.lbl_amr_link.configure(text="ROS 연결 실패", text_color=ALERT_RED)
        else:
            self.lbl_amr_link.configure(text="ROS 연결 중", text_color=ACCENT_ORANGE)

        states = {
            vehicle.vehicle_id: vehicle for vehicle in snapshot.fleet_states
        }
        stopped_names = []
        for vehicle_id, state_label in self.amr_state_labels.items():
            stop_label = self.amr_stop_labels[vehicle_id]
            detail_label = self.amr_detail_labels[vehicle_id]
            vehicle = states.get(vehicle_id)
            if vehicle is None:
                state_label.configure(
                    text="상태 수신 없음", text_color=TEXT_SECONDARY
                )
                stop_label.configure(text="")
                stop_label.pack_forget()
                detail_label.configure(text="")
                continue

            held_by_collision = (
                snapshot.collision_held_vehicle == vehicle_id
            )
            if vehicle.emergency_stopped:
                stopped_names.append(AMR_DISPLAY_NAMES[vehicle_id])

            state_text, state_color = _describe_state(vehicle)
            state_label.configure(
                text=(
                    f"{state_text} · 배터리 {vehicle.battery_percent:.0f}% "
                    f"({vehicle.battery_voltage:.1f}V)"
                ),
                text_color=state_color,
            )

            reasons = _stop_reasons(vehicle, snapshot, held_by_collision)
            if reasons:
                stop_label.configure(text="\n".join(reasons))
                stop_label.pack(anchor="w", padx=8)
            else:
                # Clear as well as hide: a stale reason must not reappear if
                # this row is ever re-packed.
                stop_label.configure(text="")
                stop_label.pack_forget()

            detail_label.configure(text=_vehicle_detail(vehicle))

        summary = []
        if snapshot.emergency_active:
            # emergency_active without any flagged vehicle means the fleet-wide
            # latch is on but per-vehicle telemetry has not caught up yet.
            target = ", ".join(stopped_names) if stopped_names else "전체"
            summary.append(f"🚨 운영자 비상정지 - {target}")
        if snapshot.collision_held_vehicle:
            held_name = AMR_DISPLAY_NAMES.get(
                snapshot.collision_held_vehicle,
                snapshot.collision_held_vehicle,
            )
            summary.append(f"⛔ 충돌 회피 정지 - {held_name}")

        if summary:
            self.lbl_estop.configure(
                text=" / ".join(summary), text_color=ALERT_RED
            )
        elif snapshot.collision_state:
            self.lbl_estop.configure(
                text=f"정지 없음 (충돌 감시 {snapshot.collision_state})",
                text_color=ACCENT_GREEN,
            )
        else:
            self.lbl_estop.configure(
                text="정지 없음 (충돌 감시 미연결)", text_color=TEXT_SECONDARY
            )

        self.after(500, self.update_amr_status)

    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        open_command_popup(self)

    def toggle_autonomy_mode(self) -> None:
        """Grant or revoke autonomous DB-backed operating approval."""
        snapshot = self.realtime_agent.snapshot()
        if snapshot.enabled and snapshot.mode in {
            'autonomous', 'inventory_execute'
        }:
            self.realtime_agent.stop_autonomous_policy()
        else:
            self.realtime_agent.start_autonomous_policy()
        self.update_autonomy_button(schedule_next=False)

    def update_autonomy_button(self, schedule_next=True) -> None:
        """Keep the dashboard control synchronized with agent state."""
        if not getattr(self, '_ui_alive', False):
            return
        try:
            if not self.winfo_exists():
                return
            snapshot = self.realtime_agent.snapshot()
            if hasattr(self, 'lbl_autonomy_detail'):
                detail = (
                    f'회차: {snapshot.cycle_id or "-"}\n'
                    f'단계: {snapshot.phase or "-"}\n'
                    f'DB: {snapshot.db_sync_state} · 보류 '
                    f'{snapshot.db_pending_count}건\n'
                    f'스냅샷: {snapshot.db_snapshot_id or "-"}\n'
                    f'다음 이동: {snapshot.next_move or "없음"}\n'
                    f'실행: {snapshot.active_command or "없음"}\n'
                    f'재계획: {snapshot.replan_count}회'
                )
                if snapshot.last_error:
                    detail += f'\n오류: {snapshot.last_error}'
                self.lbl_autonomy_detail.configure(
                    text=detail,
                    text_color=(
                        ALERT_RED if snapshot.last_error else TEXT_SECONDARY
                    ),
                )
            if hasattr(self, 'txt_autonomy_commands'):
                command_detail = (
                    'LLM 최종 판단 JSON\n'
                    f'{snapshot.llm_plan_json or "대기 중"}\n\n'
                    '결정론적 전체 실행 순서\n'
                    f'{snapshot.execution_steps_json or "대기 중"}\n\n'
                    '현재 실행 단계\n'
                    f'{snapshot.current_step_json or "없음"}\n\n'
                    '중앙관제 실제 전송 명령\n'
                    f'{snapshot.command_payload_json or "없음"}'
                )
                if command_detail != self._last_autonomy_command_text:
                    self.txt_autonomy_commands.configure(state='normal')
                    self.txt_autonomy_commands.delete('1.0', 'end')
                    self.txt_autonomy_commands.insert('1.0', command_detail)
                    self.txt_autonomy_commands.configure(state='disabled')
                    self._last_autonomy_command_text = command_detail
            if snapshot.enabled and snapshot.mode in {
                'autonomous', 'inventory_execute'
            }:
                state_suffix = {
                    'WAITING_FOR_OBJECTIVE': ' · 목표 대기',
                    'EVALUATING': ' · 판단 중',
                    'EXECUTING': ' · 작업 실행 중',
                    'WAITING_FOR_DB_SYNC': ' · DB 동기화 대기',
                    'WAITING_FOR_VEHICLE': ' · 차량 대기',
                    'WAITING_FOR_ARM_CACHE': ' · ARM1 캐시 대기',
                    'WAITING_FOR_ARM_SCAN': ' · ARM 스캔 대기',
                    'WAITING_OPERATOR': ' · 운영자 확인 필요',
                    'ERROR': ' · 오류',
                }.get(snapshot.state, '')
                running_label = (
                    'DB 계획 실행 중지'
                    if snapshot.mode == 'inventory_execute'
                    else '자율 관제모드 중지'
                )
                self.btn_autonomy.configure(
                    text=f'⏹ {running_label}{state_suffix}',
                    fg_color=ALERT_RED if snapshot.state == 'ERROR' else ACCENT_GREEN,
                    hover_color='#c94f4f' if snapshot.state == 'ERROR' else '#45b96f',
                    text_color=BG_SURFACE,
                )
            else:
                self.btn_autonomy.configure(
                    text='▶ 자율 관제모드 시작',
                    fg_color=ACCENT_ORANGE,
                    hover_color='#c65317',
                    text_color=TEXT_PRIMARY,
                )
        except Exception:
            return
        if schedule_next:
            self.after(500, self.update_autonomy_button)

    def _toggle_roi_editor(self):
        self._roi_editing = not self._roi_editing
        self._roi_drag_start = None
        self._roi_drag_current = None
        self.roi_button.configure(
            text="ROI 선택 중" if self._roi_editing else "선박 ROI 설정",
            fg_color=ACCENT_ORANGE if self._roi_editing else ACCENT_BLUE,
        )

    def _event_normalized(self, event):
        width = max(1, self.live_feed_label.winfo_width())
        height = max(1, self.live_feed_label.winfo_height())
        return (
            min(1.0, max(0.0, event.x / width)),
            min(1.0, max(0.0, event.y / height)),
        )

    def _roi_press(self, event):
        if not self._roi_editing:
            return
        self._roi_drag_start = self._event_normalized(event)
        self._roi_drag_current = self._roi_drag_start

    def _roi_motion(self, event):
        if self._roi_editing and self._roi_drag_start is not None:
            self._roi_drag_current = self._event_normalized(event)

    def _roi_release(self, event):
        if not self._roi_editing or self._roi_drag_start is None:
            return
        end = self._event_normalized(event)
        start_x, start_y = self._roi_drag_start
        end_x, end_y = end
        roi = [
            min(start_x, end_x), min(start_y, end_y),
            max(start_x, end_x), max(start_y, end_y),
        ]
        self._roi_drag_start = None
        self._roi_drag_current = None
        if roi[2] - roi[0] < 0.02 or roi[3] - roi[1] < 0.02:
            return
        self._arrival_roi = roi
        self._roi_editing = False
        self.roi_button.configure(text="저장 중", fg_color=ACCENT_ORANGE)
        threading.Thread(
            target=self._save_arrival_roi,
            args=(roi,),
            daemon=True,
        ).start()

    def _save_arrival_roi(self, roi):
        try:
            self._central_client.update_arrival_roi(roi)
            text, color = "ROI 저장됨", ACCENT_GREEN
        except Exception:
            text, color = "ROI 실패", ALERT_RED
        self.after(0, lambda: self.roi_button.configure(text=text, fg_color=color))
        self.after(
            1800,
            lambda: self.roi_button.configure(
                text="선박 ROI 설정", fg_color=ACCENT_BLUE
            ),
        )

    # ------------------------------------------------------------------
    # 실시간 영상 갱신 (CCTVMonitorView와 프레임 공유)
    # ------------------------------------------------------------------
    def update_live_feed(self) -> None:
        if not self._ui_alive:
            return
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        frame = CCTVMonitorView.SHARED_FRAME
        if frame is not None:
            try:
                color_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_height, frame_width = color_frame.shape[:2]
                roi = self._arrival_roi
                if self._roi_drag_start and self._roi_drag_current:
                    sx, sy = self._roi_drag_start
                    ex, ey = self._roi_drag_current
                    roi = [min(sx, ex), min(sy, ey), max(sx, ex), max(sy, ey)]
                cv2.rectangle(
                    color_frame,
                    (int(roi[0] * frame_width), int(roi[1] * frame_height)),
                    (int(roi[2] * frame_width), int(roi[3] * frame_height)),
                    (97, 222, 138),
                    2,
                )
                pil_img = Image.fromarray(color_frame)

                w = max(self.map_frame.winfo_width() - 10, 100)
                h = max(self.map_frame.winfo_height() - 10, 100)

                ctk_img = ctk.CTkImage(light_image=pil_img, dark_image=pil_img, size=(w, h))
                self.live_feed_label.configure(image=ctk_img, text="")
                self.live_feed_label.image = ctk_img
            except Exception:
                pass

        self.after(30, self.update_live_feed)

    # ------------------------------------------------------------------
    # 기상청(KMA) / 해양조사원(KHOA) API 연동
    # ------------------------------------------------------------------
    def fetch_weather_kma(self) -> None:
        try:
            now = datetime.now()
            if now.minute < 40:
                now = now - timedelta(hours=1)
            url = (
                "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtNcst"
                f"?ServiceKey={_WEATHER_API_KEY}&pageNo=1&numOfRows=10&dataType=JSON"
                f"&base_date={now.strftime('%Y%m%d')}&base_time={now.strftime('%H00')}&nx=54&ny=125"
            )
            response = requests.get(url, timeout=10)
            response.raise_for_status()
            items = response.json()["response"]["body"]["items"]["item"]

            temp, wind_speed, wind_direction = "--", "--", "--"
            for item in items:
                if item["category"] == "T1H":
                    temp = item["obsrValue"]
                elif item["category"] == "WSD":
                    wind_speed = item["obsrValue"]
                elif item["category"] == "VEC":
                    wind_direction = item["obsrValue"]

            self.after(0, self.update_kma_ui, temp, wind_speed, wind_direction)
        except Exception:
            self.after(0, self.show_error, "KMA")

    def fetch_ocean_khoa(self) -> None:
        try:
            today = datetime.now().strftime("%Y%m%d")
            tide_level, wave_height = "--", "--"

            tide_url = (
                f"http://www.khoa.go.kr/oceangrid/grid/api/tideObs/search.do"
                f"?ServiceKey={_WEATHER_API_KEY}&ObsCode=DT_0001&Date={today}&ResultType=json"
            )
            tide_res = requests.get(tide_url, timeout=10)
            if tide_res.status_code == 200 and "data" in tide_res.json().get("result", {}):
                obs_data = tide_res.json()["result"]["data"]
                if obs_data:
                    tide_level = obs_data[-1].get("tide_level", "--")

            wave_url = (
                f"http://www.khoa.go.kr/oceangrid/grid/api/obsWaveHight/search.do"
                f"?ServiceKey={_WEATHER_API_KEY}&ObsCode=KG_0025&Date={today}&ResultType=json"
            )
            wave_res = requests.get(wave_url, timeout=10)
            if wave_res.status_code == 200 and "data" in wave_res.json().get("result", {}):
                obs_data = wave_res.json()["result"]["data"]
                if obs_data:
                    wave_height = obs_data[-1].get("wave_h", "--")

            self.after(0, self.update_khoa_ui, tide_level, wave_height)
        except Exception:
            self.after(0, self.show_error, "KHOA")

    def update_kma_ui(self, temp, wind_speed, wind_direction) -> None:
        if not self._ui_alive:
            return
        try:
            self.weather_labels["기온"].configure(text=f"{temp} °C", text_color=ACCENT_GREEN)
            self.weather_labels["풍속"].configure(text=f"{wind_speed} m/s", text_color=ACCENT_ORANGE)
            self.weather_labels["풍향"].configure(text=f"{wind_direction} °", text_color=TEXT_PRIMARY)
            self.lbl_kma_status.configure(text="[KMA 정상 ✅]", text_color=ACCENT_GREEN)
        except Exception:
            pass

    def update_khoa_ui(self, tide_level, wave_height) -> None:
        if not self._ui_alive:
            return
        try:
            self.weather_labels["파고 및 조위"].configure(text=f"조위: {tide_level} cm\n파고: {wave_height} m", text_color=ACCENT_BLUE)
            self.lbl_khoa_status.configure(text="[KHOA 정상 ✅]", text_color=ACCENT_GREEN)
        except Exception:
            pass

    def show_error(self, api_type: str) -> None:
        if not self._ui_alive:
            return
        try:
            if api_type == "KMA":
                self.lbl_kma_status.configure(text="[KMA 실패 ❌]", text_color="#EA5455")
            elif api_type == "KHOA":
                self.lbl_khoa_status.configure(text="[KHOA 실패 ❌]", text_color="#EA5455")
        except Exception:
            pass

    def destroy(self) -> None:
        self._ui_alive = False
        super().destroy()
