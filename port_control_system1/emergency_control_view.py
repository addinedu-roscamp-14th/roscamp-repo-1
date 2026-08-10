"""
emergency_control_view.py

크레인과 AGV를 원격으로 수동 조작하는 비상 제어 화면입니다. 팀원이 만든
emergency_view.py의 구성(장비 선택 드롭다운 -> 크레인/AGV 각각 다른 제어
패널로 전환)을 그대로 가져오되, 피그마(Figma) 디자인 시안에 맞추어
제어 화면 레이아웃과 색상을 통일했습니다.
"""

import customtkinter as ctk

from ros_control_bridge import RosControlBridge

DEVICE_OPTIONS = [
    "crane_cargo (창고 크레인)",
    "crane_yard (항구 크레인)",
    "Pinky_Pro AGV (Yellow)",
    "Pinky_Pro AGV (Blue)",
]

# Tailwind Colors
BG_MAIN = "#242424"
BG_FRAME = "#2b2b2b"
BTN_BLUE = "#1f538d"
BTN_BLUE_HOVER = "#14375e"
TEXT_MAIN = "#DCE4EE"
TEXT_MUTED = "#a6a6a6"
ALERT_RED = "#8b2525"
ALERT_RED_HOVER = "#6e1d1d"
GREEN = "#2fa572"
GREEN_HOVER = "#25825a"
BORDER_COLOR = "#333333"
BORDER_LIGHT = "#444444"

class EmergencyControlView(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color=BG_MAIN, **kwargs)
        self.ros_bridge = RosControlBridge.get_instance()

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=22, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=16, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=13)
        self.font_mono = ctk.CTkFont(family="Consolas", size=12)
        self.font_mono_small = ctk.CTkFont(family="Consolas", size=10)

        self.main_menu_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.control_panel_frame = ctk.CTkFrame(self, fg_color="transparent")

        self._build_main_menu()
        self.main_menu_frame.pack(fill="both", expand=True)

    # ------------------------------------------------------------------
    # 메인 메뉴: 장비 선택 + 전체 비상 정지
    # ------------------------------------------------------------------
    def _build_main_menu(self) -> None:
        menu_container = ctk.CTkFrame(self.main_menu_frame, fg_color=BG_FRAME, border_width=1, border_color=BORDER_COLOR, corner_radius=12)
        menu_container.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(menu_container, text="⚠️ 원격 비상 제어 시스템", font=self.font_title, text_color=TEXT_MAIN).pack(pady=(30, 10))
        ctk.CTkLabel(menu_container, text="장비 선택 및 제어권 인계", font=self.font_subtitle, text_color=TEXT_MUTED).pack(padx=30, pady=(0, 20))

        self.device_selector = ctk.CTkOptionMenu(menu_container, values=DEVICE_OPTIONS, width=300, fg_color=BG_MAIN, button_color=BORDER_LIGHT, button_hover_color=BORDER_COLOR)
        self.device_selector.pack(padx=30, pady=10)

        ctk.CTkButton(menu_container, text="선택 장비 제어권 연결", font=self.font_subtitle, fg_color=BTN_BLUE, hover_color=BTN_BLUE_HOVER,
                     command=self.open_control_panel, height=45).pack(padx=30, pady=10, fill="x")

        ctk.CTkButton(menu_container, text="🚨 전체 시스템 비상 정지", font=self.font_subtitle,
                     fg_color=ALERT_RED, hover_color=ALERT_RED_HOVER, height=50,
                     command=self._activate_emergency_stop).pack(padx=30, pady=(30, 8), fill="x")
        ctk.CTkButton(
            menu_container,
            text="비상 정지 해제",
            font=self.font_subtitle,
            fg_color=BORDER_LIGHT,
            hover_color=BORDER_COLOR,
            height=40,
            command=self._release_emergency_stop,
        ).pack(padx=30, pady=(0, 30), fill="x")

        bottom_row = ctk.CTkFrame(self.main_menu_frame, fg_color="transparent")
        bottom_row.pack(fill="x", side="bottom", pady=15, padx=20)
        ctk.CTkButton(bottom_row, text="🗣️ 명령", font=self.font_subtitle, fg_color="#92ccff", text_color="#003351",
                      hover_color="#cce5ff", height=40, width=100, command=self.open_command_popup).pack(side="right", padx=10)

    def open_control_panel(self) -> None:
        try:
            app = self.winfo_toplevel()
            if hasattr(app, "current_user_id") and app.current_user_id:
                role = app.USERS.get(app.current_user_id, {}).get("role", "")
                if role != "최고 관리자 (Admin)":
                    from tkinter import messagebox
                    messagebox.showerror("접근 거부", "장비 제어권 인계 권한이 없습니다.\n(최고 관리자 전용 기능)")
                    return
        except Exception:
            pass

        selected = self.device_selector.get()
        for widget in self.control_panel_frame.winfo_children():
            widget.destroy()

        if "crane_cargo" in selected:
            self._build_warehouse_crane_panel(selected)
        elif "crane_yard" in selected:
            self._build_port_crane_panel(selected)
        else:
            self._build_agv_panel(selected)

        self.main_menu_frame.pack_forget()
        self.control_panel_frame.pack(fill="both", expand=True)

    # ------------------------------------------------------------------
    # 공통 제어 화면 레이아웃 (헤더 & 카메라 뷰어)
    # ------------------------------------------------------------------
    def _build_control_base(self, device_name: str) -> ctk.CTkFrame:
        # Header (App Bar)
        header = ctk.CTkFrame(self.control_panel_frame, height=64, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        header.pack(fill="x", padx=16, pady=(16, 8))
        header.pack_propagate(False)

        # Header Left
        h_left = ctk.CTkFrame(header, fg_color="transparent")
        h_left.pack(side="left", fill="y", padx=16)
        
        btn_back = ctk.CTkButton(h_left, text="← Back to Menu", width=100, fg_color="transparent", text_color=TEXT_MUTED, hover_color=BORDER_LIGHT, font=self.font_subtitle, command=self.stop_control)
        btn_back.pack(side="left", padx=(0, 16))

        divider = ctk.CTkFrame(h_left, width=1, fg_color=BORDER_LIGHT)
        divider.pack(side="left", fill="y", pady=16)

        title_lbl = ctk.CTkLabel(h_left, text=f"⚠️ {device_name}", font=self.font_subtitle, text_color=TEXT_MAIN)
        title_lbl.pack(side="left", padx=(16, 8))
        
        tag_lbl = ctk.CTkLabel(h_left, text="EMERGENCY OVERRIDE ACTIVE", font=self.font_mono, text_color=ALERT_RED, fg_color="#3a1c1c", corner_radius=4)
        tag_lbl.pack(side="left")

        # Header Right
        h_right = ctk.CTkFrame(header, fg_color="transparent")
        h_right.pack(side="right", fill="y", padx=16)
        ctk.CTkLabel(h_right, text="Operator OP-084", text_color=TEXT_MUTED, font=self.font_body).pack(anchor="e", pady=(10, 0))
        ctk.CTkLabel(h_right, text="🟢 SECURE CONNECTION", text_color=GREEN, font=self.font_mono_small).pack(anchor="e")

        # Main Layout (Flex row: Camera vs Controls)
        content = ctk.CTkFrame(self.control_panel_frame, fg_color="transparent")
        content.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        # Camera Panel (Left)
        cam_panel = ctk.CTkFrame(content, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        cam_panel.pack(side="left", fill="both", expand=True, padx=(0, 8))

        # Camera Top Bar
        cam_top = ctk.CTkFrame(cam_panel, fg_color="transparent")
        cam_top.pack(fill="x", padx=16, pady=16)
        cam_lbl = ctk.CTkLabel(cam_top, text="🟢 CAM_01_FWD", font=self.font_mono, fg_color="#1e1e1e", corner_radius=4)
        cam_lbl.pack(side="left")
        sig_lbl = ctk.CTkLabel(cam_top, text="❌ NO SIGNAL", font=self.font_mono, text_color=ALERT_RED, fg_color="#1e1e1e", corner_radius=4)
        sig_lbl.pack(side="left", padx=8)

        # Camera Center
        cam_center = ctk.CTkFrame(cam_panel, fg_color="#1a1a1a", border_width=1, border_color=BORDER_COLOR)
        cam_center.pack(fill="both", expand=True, padx=16, pady=(0, 16))
        ctk.CTkLabel(cam_center, text="[ Waiting for camera link ]\nVisual feed temporarily unavailable.", text_color="#888", font=self.font_mono).place(relx=0.5, rely=0.5, anchor="center")

        # Telemetry Bar
        tele_bar = ctk.CTkFrame(cam_panel, height=48, fg_color=BG_MAIN, corner_radius=4, border_width=1, border_color=BORDER_COLOR)
        tele_bar.pack(fill="x", padx=16, pady=(0, 16))
        tele_bar.pack_propagate(False)
        ctk.CTkLabel(tele_bar, text="LAT: 34.0522 N   LON: 118.2437 W   ALT: 12.4m", font=self.font_mono, text_color=TEXT_MUTED).pack(side="left", padx=16)
        ctk.CTkLabel(tele_bar, text="❌ LIDAR OFFLINE", font=self.font_mono, text_color=ALERT_RED).pack(side="right", padx=16)

        # Controls Panel Container (Right)
        right_panel = ctk.CTkFrame(content, width=380, fg_color="transparent")
        right_panel.pack(side="right", fill="y", padx=(8, 0))
        right_panel.pack_propagate(False)

        return right_panel

    # ------------------------------------------------------------------
    # JetCobot (항구 크레인 - crane_yard) 제어 패널
    # ------------------------------------------------------------------
    def _build_port_crane_panel(self, device_name: str) -> None:
        right_panel = self._build_control_base(device_name)
        
        # Spatial Movement (X, Y)
        xy_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        xy_frame.pack(fill="x", pady=(0, 16))
        ctk.CTkLabel(xy_frame, text="Space Movement (X, Y)", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
        ctk.CTkFrame(xy_frame, height=1, fg_color=BORDER_LIGHT).pack(fill="x", padx=16)
        
        pad = ctk.CTkFrame(xy_frame, fg_color="transparent")
        pad.pack(pady=16)
        btn_kwargs = {"font": self.font_mono, "width": 80, "height": 60, "corner_radius": 4, "fg_color": BTN_BLUE, "hover_color": BTN_BLUE_HOVER, "text_color": TEXT_MAIN}
        
        ctk.CTkButton(pad, text="▲ X+\n(Fwd)", **btn_kwargs).grid(row=0, column=1, padx=5, pady=5)
        ctk.CTkButton(pad, text="◀ Y+\n(L)", **btn_kwargs).grid(row=1, column=0, padx=5, pady=5)
        ctk.CTkLabel(pad, text="●", text_color=TEXT_MUTED).grid(row=1, column=1)
        ctk.CTkButton(pad, text="▶ Y-\n(R)", **btn_kwargs).grid(row=1, column=2, padx=5, pady=5)
        ctk.CTkButton(pad, text="▼ X-\n(Bwd)", **btn_kwargs).grid(row=2, column=1, padx=5, pady=5)

        # Height Control (Z)
        z_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        z_frame.pack(fill="x", pady=(0, 16))
        ctk.CTkLabel(z_frame, text="Height Control (Z)", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
        ctk.CTkFrame(z_frame, height=1, fg_color=BORDER_LIGHT).pack(fill="x", padx=16)
        
        z_btns = ctk.CTkFrame(z_frame, fg_color="transparent")
        z_btns.pack(fill="x", padx=16, pady=16)
        ctk.CTkButton(z_btns, text="⬆ Z+ Rise", font=self.font_mono, height=50, fg_color=BTN_BLUE, hover_color=BTN_BLUE_HOVER).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ctk.CTkButton(z_btns, text="⬇ Z- Descend", font=self.font_mono, height=50, fg_color=BTN_BLUE, hover_color=BTN_BLUE_HOVER).pack(side="left", fill="x", expand=True, padx=(4, 0))

        # Gripper Module
        grip_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        grip_frame.pack(fill="both", expand=True)
        ctk.CTkLabel(grip_frame, text="Gripper Module", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
        ctk.CTkFrame(grip_frame, height=1, fg_color=BORDER_LIGHT).pack(fill="x", padx=16)
        
        ctk.CTkButton(
            grip_frame,
            text="컨테이너 파지",
            height=50,
            font=self.font_subtitle,
            fg_color=GREEN,
            hover_color=GREEN_HOVER,
            command=lambda: self._call_crane_service(
                device_name, "pick_container"
            ),
        ).pack(fill="x", padx=16, pady=(16, 8))
        ctk.CTkButton(
            grip_frame,
            text="ID 1 위 적재",
            height=50,
            font=self.font_subtitle,
            fg_color=BORDER_LIGHT,
            hover_color="#555555",
            command=lambda: self._call_crane_service(
                device_name, "stack_container"
            ),
        ).pack(fill="x", padx=16, pady=(0, 16))

        load_frame = ctk.CTkFrame(grip_frame, fg_color=BG_MAIN, corner_radius=4)
        load_frame.pack(fill="x", padx=16, pady=(0, 16), ipady=4)
        ctk.CTkLabel(load_frame, text="GRIPPER LOAD", font=self.font_mono, text_color=TEXT_MUTED).pack(side="left", padx=10)
        ctk.CTkLabel(load_frame, text="0.0 kg", font=self.font_mono, text_color=TEXT_MAIN).pack(side="right", padx=10)

    # ------------------------------------------------------------------
    # JetCobot (창고 크레인 - crane_cargo) 제어 패널
    # ------------------------------------------------------------------
    def _build_warehouse_crane_panel(self, device_name: str) -> None:
        right_panel = self._build_control_base(device_name)

        # Power & Connection
        pwr_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        pwr_frame.pack(fill="x", pady=(0, 16))
        
        ctk.CTkLabel(pwr_frame, text="Power & Connection", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
        ctk.CTkFrame(pwr_frame, height=1, fg_color=BORDER_LIGHT).pack(fill="x", padx=16)

        pwr_btns = ctk.CTkFrame(pwr_frame, fg_color="transparent")
        pwr_btns.pack(fill="x", padx=16, pady=16)
        
        # 추후 명령어를 할당할 제어 연결 버튼
        ctk.CTkButton(pwr_btns, text="🔌 제어 연결", height=36, font=self.font_mono, fg_color=BTN_BLUE, hover_color=BTN_BLUE_HOVER, command=lambda: print("제어 연결 로직 대기중")).pack(side="left", fill="x", expand=True, padx=(0, 4))

        # Position Commands
        pos_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        pos_frame.pack(fill="x", pady=(0, 16))
        ctk.CTkLabel(pos_frame, text="Position Control", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
        ctk.CTkFrame(pos_frame, height=1, fg_color=BORDER_LIGHT).pack(fill="x", padx=16)
        
        btn_kwargs = {"font": self.font_subtitle, "height": 40, "fg_color": BORDER_LIGHT, "hover_color": "#555555"}

        pos_grid = ctk.CTkFrame(pos_frame, fg_color="transparent")
        pos_grid.pack(fill="x", padx=16, pady=16)
        
        ctk.CTkButton(pos_grid, text="초기위치 이동", command=lambda: self._call_crane_service(device_name, "go_initial_pose"), **btn_kwargs).grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        ctk.CTkButton(pos_grid, text="적재층수 초기화", command=lambda: self._call_crane_service(device_name, "reset_stack_level"), **btn_kwargs).grid(row=0, column=1, sticky="ew", padx=4, pady=4)

        ctk.CTkButton(pos_grid, text="A-1 위치이동", command=lambda: self._call_crane_service(device_name, "go_a1_pose"), **btn_kwargs).grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
        ctk.CTkButton(pos_grid, text="A-2 위치이동", command=lambda: self._call_crane_service(device_name, "go_a2_pose"), **btn_kwargs).grid(row=2, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
        ctk.CTkButton(pos_grid, text="A-3 위치이동", command=lambda: self._call_crane_service(device_name, "go_a3_pose"), **btn_kwargs).grid(row=3, column=0, columnspan=2, sticky="ew", padx=4, pady=4)

        pos_grid.grid_columnconfigure(0, weight=1)
        pos_grid.grid_columnconfigure(1, weight=1)

        # Emergency Stop for Arm
        emg_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        emg_frame.pack(fill="x", side="bottom", pady=(0, 16))
        
        ctk.CTkButton(emg_frame, text="🚨 긴급 정지 (Stop Pick)", font=self.font_subtitle, height=50, fg_color=ALERT_RED, hover_color=ALERT_RED_HOVER, command=lambda: self._call_crane_service(device_name, "stop_pick")).pack(fill="x", padx=16, pady=16)

    # ------------------------------------------------------------------
    # Pinky_Pro (AGV) 제어 패널
    # ------------------------------------------------------------------
    def _build_agv_panel(self, device_name: str) -> None:
        right_panel = self._build_control_base(device_name)
        
        # Power & Connection
        pwr_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        pwr_frame.pack(fill="x", pady=(0, 16))
        
        ctk.CTkLabel(pwr_frame, text="Power & Connection", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
        ctk.CTkFrame(pwr_frame, height=1, fg_color=BORDER_LIGHT).pack(fill="x", padx=16)

        pwr_btns = ctk.CTkFrame(pwr_frame, fg_color="transparent")
        pwr_btns.pack(fill="x", padx=16, pady=16)
        
        ctk.CTkButton(pwr_btns, text="🔌 제어 연결", height=36, font=self.font_mono, fg_color=BTN_BLUE, hover_color=BTN_BLUE_HOVER, command=self._connect_pinky_control).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ctk.CTkButton(pwr_btns, text="⚡ 시동 (Engine ON)", height=36, font=self.font_mono, fg_color=GREEN, hover_color=GREEN_HOVER, command=self._start_pinky_engine).pack(side="left", fill="x", expand=True, padx=(4, 0))
        
        # Drive Control
        drive_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        drive_frame.pack(fill="x", pady=(0, 16))
        ctk.CTkLabel(drive_frame, text="AGV Drive Control", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
        ctk.CTkFrame(drive_frame, height=1, fg_color=BORDER_LIGHT).pack(fill="x", padx=16)
        
        pad = ctk.CTkFrame(drive_frame, fg_color="transparent")
        pad.pack(pady=20)
        btn_kwargs = {"font": self.font_mono, "width": 80, "height": 70, "corner_radius": 4, "fg_color": BTN_BLUE, "hover_color": BTN_BLUE_HOVER, "text_color": TEXT_MAIN}
        
        ctk.CTkButton(pad, text="↖\nFwd-L", **btn_kwargs, command=lambda: self._send_teleop_cmd('u')).grid(row=0, column=0, padx=6, pady=6)
        ctk.CTkButton(pad, text="▲\nForward", **btn_kwargs, command=lambda: self._send_teleop_cmd('i')).grid(row=0, column=1, padx=6, pady=6)
        ctk.CTkButton(pad, text="↗\nFwd-R", **btn_kwargs, command=lambda: self._send_teleop_cmd('o')).grid(row=0, column=2, padx=6, pady=6)
        
        ctk.CTkButton(pad, text="◀\nLeft", **btn_kwargs, command=lambda: self._send_teleop_cmd('j')).grid(row=1, column=0, padx=6, pady=6)
        ctk.CTkButton(pad, text="■\nSTOP", font=self.font_subtitle, width=80, height=70, corner_radius=4, fg_color=ALERT_RED, hover_color=ALERT_RED_HOVER, command=lambda: self._send_teleop_cmd('k')).grid(row=1, column=1, padx=6, pady=6)
        ctk.CTkButton(pad, text="▶\nRight", **btn_kwargs, command=lambda: self._send_teleop_cmd('l')).grid(row=1, column=2, padx=6, pady=6)
        
        ctk.CTkButton(pad, text="↙\nRev-L", **btn_kwargs, command=lambda: self._send_teleop_cmd('m')).grid(row=2, column=0, padx=6, pady=6)
        ctk.CTkButton(pad, text="▼\nReverse", **btn_kwargs, command=lambda: self._send_teleop_cmd(',')).grid(row=2, column=1, padx=6, pady=6)
        ctk.CTkButton(pad, text="↘\nRev-R", **btn_kwargs, command=lambda: self._send_teleop_cmd('.')).grid(row=2, column=2, padx=6, pady=6)

        # Speed Control
        spd_frame = ctk.CTkFrame(right_panel, fg_color=BG_FRAME, corner_radius=8, border_width=1, border_color=BORDER_COLOR)
        spd_frame.pack(fill="both", expand=True)
        ctk.CTkLabel(spd_frame, text="Speed Override", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
        ctk.CTkFrame(spd_frame, height=1, fg_color=BORDER_LIGHT).pack(fill="x", padx=16)

        # Linear Speed
        ctk.CTkLabel(spd_frame, text="Linear Speed (직진 속도)", font=self.font_mono_small, text_color=TEXT_MUTED).pack(anchor="w", padx=16, pady=(12, 0))
        l_spd_btns = ctk.CTkFrame(spd_frame, fg_color="transparent")
        l_spd_btns.pack(fill="x", padx=16, pady=(4, 12))
        ctk.CTkButton(l_spd_btns, text="SLOW (-10%)", height=32, fg_color=BORDER_LIGHT, hover_color="#555555", command=lambda: self._send_teleop_cmd('x')).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ctk.CTkButton(l_spd_btns, text="FAST (+10%)", height=32, fg_color=BORDER_LIGHT, hover_color="#555555", command=lambda: self._send_teleop_cmd('w')).pack(side="left", fill="x", expand=True, padx=(4, 0))

        # Angular Speed
        ctk.CTkLabel(spd_frame, text="Angular Speed (각속도)", font=self.font_mono_small, text_color=TEXT_MUTED).pack(anchor="w", padx=16, pady=(4, 0))
        a_spd_btns = ctk.CTkFrame(spd_frame, fg_color="transparent")
        a_spd_btns.pack(fill="x", padx=16, pady=(4, 12))
        ctk.CTkButton(a_spd_btns, text="SLOW (-10%)", height=32, fg_color=BORDER_LIGHT, hover_color="#555555", command=lambda: self._send_teleop_cmd('c')).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ctk.CTkButton(a_spd_btns, text="FAST (+10%)", height=32, fg_color=BORDER_LIGHT, hover_color="#555555", command=lambda: self._send_teleop_cmd('e')).pack(side="left", fill="x", expand=True, padx=(4, 0))

    def stop_control(self) -> None:
        self.control_panel_frame.pack_forget()
        self.main_menu_frame.pack(fill="both", expand=True)
        self._disconnect_pinky()

    def destroy(self) -> None:
        self._disconnect_pinky()
        super().destroy()

    def _disconnect_pinky(self) -> None:
        import subprocess
        try:
            # 창을 나가거나 종료할 때, 수동 제어(teleop) 세션만 안전하게 닫습니다.
            # 시동(bringup) 세션은 유지하여 LLM 시동 상태가 꺼지지 않게 합니다.
            subprocess.run(["/usr/bin/tmux", "kill-session", "-t", "pinky_teleop_yellow"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["/usr/bin/tmux", "kill-session", "-t", "pinky_teleop_blue"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def _run_automated_ssh(self, session_name: str, command: str, title: str, ip_address: str) -> None:
        import subprocess
        import threading
        import time
        
        # 기존 동일 세션 종료
        subprocess.run(["/usr/bin/tmux", "kill-session", "-t", session_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # 새 터미널 창을 띄우지 않고 백그라운드(숨김) 모드로 SSH 접속 시작
        subprocess.run(["/usr/bin/tmux", "new-session", "-d", "-s", session_name, f"ssh -tt pinky@{ip_address}"])
        
        def auto_type():
            time.sleep(5.0) # SSH 암호 프롬프트 대기
            # 비밀번호 '1' 자동 입력
            subprocess.run(["/usr/bin/tmux", "send-keys", "-t", session_name, "1", "Enter"])
            time.sleep(5.0) # 로그인 대기
            
            if command:
                subprocess.run(["/usr/bin/tmux", "send-keys", "-t", session_name, command, "Enter"])
                
        threading.Thread(target=auto_type, daemon=True).start()

    def _start_pinky_engine(self, auto: bool = False) -> None:
        from tkinter import messagebox
        selected = self.device_selector.get()
        robot_color = "blue" if "Blue" in selected else "yellow"
        domain_id = 12 if "Blue" in selected else 13
        ip_address = "192.168.0.102" if "Blue" in selected else "192.168.0.96"
        self._run_automated_ssh(
            session_name=f"pinky_bringup_{robot_color}",
            command=f"export ROS_DOMAIN_ID={domain_id}; ros2 launch pinky bringup_robot.launch.xml",
            title=f"Pinky Bringup (Domain: {domain_id})",
            ip_address=ip_address
        )
        if not auto:
            messagebox.showinfo("시동 (Engine ON)", f"{ip_address}(도메인 {domain_id})으로 자동 SSH 접속 후 Bringup 명령을 실행합니다.\n잠시 기다려주세요.")

    def _connect_pinky_control(self) -> None:
        from tkinter import messagebox
        selected = self.device_selector.get()
        robot_color = "blue" if "Blue" in selected else "yellow"
        domain_id = 12 if "Blue" in selected else 13
        ip_address = "192.168.0.102" if "Blue" in selected else "192.168.0.96"
        self._run_automated_ssh(
            session_name=f"pinky_teleop_{robot_color}",
            command=f"export ROS_DOMAIN_ID={domain_id}; ros2 run teleop_twist_keyboard teleop_twist_keyboard",
            title=f"Pinky Teleop (Domain: {domain_id})",
            ip_address=ip_address
        )
        messagebox.showinfo("제어 연결", f"{ip_address}(도메인 {domain_id})으로 자동 SSH 접속 후 Teleop 제어 노드를 실행합니다.\n터미널에 명령어가 입력된 후 방향키 제어를 사용할 수 있습니다.")

    def _send_teleop_cmd(self, char: str) -> None:
        velocity_commands = {
            'u': (0.10, 0.8),
            'i': (0.12, 0.0),
            'o': (0.10, -0.8),
            'j': (0.0, 1.0),
            'k': (0.0, 0.0),
            'l': (0.0, -1.0),
            'm': (-0.10, -0.8),
            ',': (-0.12, 0.0),
            '.': (-0.10, 0.8),
        }
        if char in velocity_commands and self.ros_bridge.snapshot().ready:
            linear, angular = velocity_commands[char]
            self.ros_bridge.send_velocity(linear, angular)
            return

        import subprocess
        selected = self.device_selector.get()
        robot_color = "blue" if "Blue" in selected else "yellow"
        try:
            subprocess.Popen(["/usr/bin/tmux", "send-keys", "-t", f"pinky_teleop_{robot_color}", char], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print(f"Failed to send command: {e}")

    def _activate_emergency_stop(self) -> None:
        from tkinter import messagebox
        try:
            app = self.winfo_toplevel()
            if hasattr(app, "current_user_id") and app.current_user_id:
                role = app.USERS.get(app.current_user_id, {}).get("role", "")
                if role != "최고 관리자 (Admin)":
                    messagebox.showerror("접근 거부", "비상 정지 권한이 없습니다.\n(최고 관리자 전용 기능)")
                    return
        except Exception:
            pass

        if self.ros_bridge.emergency_stop():
            messagebox.showwarning(
                "비상 정지",
                "현재 ROS 도메인의 /cmd_vel 정지 신호를 100Hz로 유지합니다.",
            )
        else:
            messagebox.showerror(
                "비상 정지 실패",
                "ROS 브리지가 연결되지 않았습니다.",
            )

    def _release_emergency_stop(self) -> None:
        from tkinter import messagebox
        try:
            app = self.winfo_toplevel()
            if hasattr(app, "current_user_id") and app.current_user_id:
                role = app.USERS.get(app.current_user_id, {}).get("role", "")
                if role != "최고 관리자 (Admin)":
                    messagebox.showerror("접근 거부", "비상 정지 해제 권한이 없습니다.\n(최고 관리자 전용 기능)")
                    return
        except Exception:
            pass

        if self.ros_bridge.release_emergency_stop():
            messagebox.showinfo(
                "비상 정지 해제",
                "정지 유지 신호를 해제했습니다. 이동 전에 주변 안전을 확인하세요.",
            )
        else:
            messagebox.showerror(
                "해제 실패",
                "ROS 브리지가 연결되지 않았습니다.",
            )

    def _call_crane_service(
        self,
        device_name: str,
        operation: str,
    ) -> None:
        from tkinter import messagebox

        namespace = "/arm2" if "cargo" in device_name.lower() else "/arm"
        service_name = f"{namespace}/{operation}"
        if self.ros_bridge.call_trigger(service_name):
            messagebox.showinfo(
                "로봇팔 명령 전송",
                f"{service_name} 요청을 전송했습니다.\n"
                "로봇팔 상태 토픽과 실행 로그를 확인하세요.",
            )
        else:
            messagebox.showerror(
                "로봇팔 명령 실패",
                f"{service_name} 서비스를 찾을 수 없습니다.",
            )

    # ------------------------------------------------------------------
    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        open_command_popup(self)
