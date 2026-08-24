"""
emergency_control_view.py

크레인과 AGV를 원격으로 수동 조작하는 비상 제어 화면입니다. 팀원이 만든
emergency_view.py의 구성(장비 선택 드롭다운 -> 크레인/AGV 각각 다른 제어
패널로 전환)을 그대로 가져오되, 피그마(Figma) 디자인 시안에 맞추어
제어 화면 레이아웃과 색상을 통일했습니다.
"""

import shutil

import cv2
from PIL import Image
import customtkinter as ctk

from cctv_monitor_view import CCTVMonitorView
from ros_control_bridge import AMR_DISPLAY_NAMES, RosControlBridge


def _tmux_path():
    """Locate tmux, or return None when it is not installed.

    The SSH terminal helpers below are only a fallback for when the ROS
    bridge is unavailable, so tmux is optional. Hardcoding /usr/bin/tmux
    crashed the whole emergency screen on machines without it.
    """
    return shutil.which("tmux")

AMR_1_LABEL = AMR_DISPLAY_NAMES["agv1"]
AMR_2_LABEL = AMR_DISPLAY_NAMES["agv2"]

DEVICE_OPTIONS = [
    "JetCobot #01 (크레인)",
    "JetCobot #02 (크레인)",
    AMR_1_LABEL,
    AMR_2_LABEL,
]

# Operator-facing name -> real vehicle wiring. AMR 1 carries the blue cargo
# box and AMR 2 the yellow one, matching the cargo_box colours in
# pinky.urdf.xacro. Only the displayed name changed: the ROS namespace is
# still agv1/agv2, so topics, services and actions keep their current names.
AMR_DEVICES = {
    AMR_1_LABEL: {
        "vehicle_id": "agv1",
        "host": "192.168.5.2",
        "domain_id": 13,
    },
    AMR_2_LABEL: {
        "vehicle_id": "agv2",
        "host": "192.168.5.3",
        "domain_id": 14,
    },
}

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

    def _selected_amr(self) -> dict:
        """Resolve the selected AMR to its vehicle id, host and ROS domain.

        Returns an empty dict when a crane is selected. Matching on the label
        rather than a colour substring keeps the display name free to change
        without silently re-targeting a different robot.
        """
        return AMR_DEVICES.get(self.device_selector.get(), {})

    def open_control_panel(self) -> None:
        selected = self.device_selector.get()
        for widget in self.control_panel_frame.winfo_children():
            widget.destroy()

        if "크레인" in selected:
            self._build_crane_panel(selected)
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
        self._cam_lbl = ctk.CTkLabel(cam_top, text="🟢 CAM_01_FWD", font=self.font_mono, fg_color="#1e1e1e", corner_radius=4)
        self._cam_lbl.pack(side="left")
        self._cam_sig_lbl = ctk.CTkLabel(cam_top, text="❌ NO SIGNAL", font=self.font_mono, text_color=ALERT_RED, fg_color="#1e1e1e", corner_radius=4)
        self._cam_sig_lbl.pack(side="left", padx=8)

        # Camera Center
        cam_center = ctk.CTkFrame(cam_panel, fg_color="#1a1a1a", border_width=1, border_color=BORDER_COLOR)
        cam_center.pack(fill="both", expand=True, padx=16, pady=(0, 16))
        self._cam_video_label = ctk.CTkLabel(
            cam_center,
            text="[ Waiting for camera link ]\nVisual feed temporarily unavailable.",
            text_color="#888",
            font=self.font_mono,
        )
        self._cam_video_label.place(relx=0.5, rely=0.5, anchor="center")

        # Telemetry Bar
        tele_bar = ctk.CTkFrame(cam_panel, height=48, fg_color=BG_MAIN, corner_radius=4, border_width=1, border_color=BORDER_COLOR)
        tele_bar.pack(fill="x", padx=16, pady=(0, 16))
        tele_bar.pack_propagate(False)
        ctk.CTkLabel(tele_bar, text="LAT: 34.0522 N   LON: 118.2437 W   ALT: 12.4m", font=self.font_mono, text_color=TEXT_MUTED).pack(side="left", padx=16)
        ctk.CTkLabel(tele_bar, text="❌ LIDAR OFFLINE", font=self.font_mono, text_color=ALERT_RED).pack(side="right", padx=16)

        # CCTV 백그라운드 캡처가 실행 중이 아니면 시작
        CCTVMonitorView.ensure_capture_running()
        self._cam_ui_alive = True
        self._update_camera_loop()

        # Controls Panel Container (Right)
        right_panel = ctk.CTkFrame(content, width=320, fg_color="transparent")
        right_panel.pack(side="right", fill="y", padx=(8, 0))
        right_panel.pack_propagate(False)

        return right_panel

    # ------------------------------------------------------------------
    # JetCobot (크레인) 제어 패널
    # ------------------------------------------------------------------
    def _build_crane_panel(self, device_name: str) -> None:
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
        ctk.CTkLabel(drive_frame, text="AMR Drive Control", font=self.font_subtitle, text_color=TEXT_MAIN).pack(anchor="w", padx=16, pady=12)
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
        self._cam_ui_alive = False
        self.control_panel_frame.pack_forget()
        self.main_menu_frame.pack(fill="both", expand=True)
        self._disconnect_pinky()

    def _update_camera_loop(self) -> None:
        """CCTVMonitorView.SHARED_FRAME을 비상제어 카메라 패널에 표시."""
        if not getattr(self, '_cam_ui_alive', False):
            return
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        frame = CCTVMonitorView.SHARED_FRAME
        if frame is not None:
            try:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)
                # 카메라 패널이 최대한 큰 영상을 표시하도록 최소 크기를
                # 넉넉히 잡고, 사용 가능한 공간 전체를 사용합니다.
                w = max(self._cam_video_label.winfo_width(), 640)
                h = max(self._cam_video_label.winfo_height(), 480)
                # 비율 유지하면서 패널에 맞춤
                img_w, img_h = img.size
                scale = min(w / img_w, h / img_h)
                if scale < 1.0:
                    img = img.resize(
                        (int(img_w * scale), int(img_h * scale)),
                        Image.LANCZOS,
                    )
                ctk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(w, h))
                self._cam_video_label.configure(image=ctk_img, text="")
                self._cam_video_label.image = ctk_img
                self._cam_sig_lbl.configure(text="● LIVE", text_color=GREEN)
            except Exception:
                pass
        else:
            self._cam_sig_lbl.configure(text="❌ NO SIGNAL", text_color=ALERT_RED)

        self.after(30, self._update_camera_loop)

    def destroy(self) -> None:
        self._cam_ui_alive = False
        self._disconnect_pinky()
        super().destroy()

    def _disconnect_pinky(self) -> None:
        import os
        import subprocess
        tmux = _tmux_path()
        try:
            if tmux:
                for session in ("pinky_teleop", "pinky_bringup"):
                    subprocess.run([tmux, "kill-session", "-t", session], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            for device in AMR_DEVICES.values():
                os.system(f"pkill -9 -f 'ssh -tt pinky@{device['host']}'")
            os.system("pkill -9 -f 'pinky_teleop'")
            os.system("pkill -9 -f 'pinky_bringup'")
        except Exception:
            pass

    def _run_automated_ssh(self, session_name: str, command: str, title: str, ip_address: str) -> bool:
        """Open a terminal, SSH into the AMR and run one command.

        Returns False when tmux is missing instead of raising: this is only
        the fallback path, and a missing optional tool must not crash the
        emergency screen.
        """
        import subprocess
        import threading
        import time

        tmux = _tmux_path()
        if not tmux:
            return False

        # 기존 동일 세션 종료
        subprocess.run([tmux, "kill-session", "-t", session_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 새 터미널에서 SSH 접속 시작
        subprocess.Popen([
            "gnome-terminal",
            "--title", title,
            "--",
            "bash", "-c",
            f"echo -e '\\e[1;32m[{title}]\\e[0m 자동 연결을 진행합니다...'; {tmux} new-session -s {session_name} 'ssh -tt pinky@{ip_address}'"
        ])
        
        def auto_type():
            time.sleep(2.0) # SSH 암호 프롬프트 대기
            # 비밀번호 '1' 자동 입력
            subprocess.run([tmux, "send-keys", "-t", session_name, "1", "Enter"])
            time.sleep(1.0) # 로그인 대기

            if command:
                subprocess.run([tmux, "send-keys", "-t", session_name, command, "Enter"])

        threading.Thread(target=auto_type, daemon=True).start()
        return True

    def _start_pinky_engine(self, auto: bool = False) -> None:
        from tkinter import messagebox
        device = self._selected_amr()
        if not device:
            messagebox.showerror("시동 실패", "AMR을 먼저 선택하세요.")
            return
        domain_id = device["domain_id"]
        ip_address = device["host"]
        if not self._run_automated_ssh(
            session_name="pinky_bringup",
            command=f"export ROS_DOMAIN_ID={domain_id}; ros2 launch pinky bringup_robot.launch.xml",
            title=f"AMR Bringup (Domain: {domain_id})",
            ip_address=ip_address
        ):
            messagebox.showerror(
                "시동 실패",
                "이 기능은 tmux가 필요한데 설치되어 있지 않습니다.\n"
                "'sudo apt install tmux' 후 다시 시도하거나, 차량에서 직접\n"
                f"ROS_DOMAIN_ID={domain_id} 으로 bringup을 실행하세요.",
            )
            return
        if not auto:
            messagebox.showinfo("시동 (Engine ON)", f"{ip_address}(도메인 {domain_id})으로 자동 SSH 접속 후 Bringup 명령을 실행합니다.\n잠시 기다려주세요.")

    def _connect_pinky_control(self) -> None:
        from tkinter import messagebox
        device = self._selected_amr()
        if not device:
            messagebox.showerror("제어 연결 실패", "AMR을 먼저 선택하세요.")
            return

        # Preferred path: publish straight to /<vehicle_id>/cmd_vel_manual over
        # the zenoh bridge. The SSH/teleop terminal below stays as a fallback
        # for when the bridge is down.
        if self.ros_bridge.snapshot().ready:
            messagebox.showinfo(
                "제어 연결",
                f"{device['vehicle_id']} 수동 제어가 연결되었습니다.\n"
                "방향 버튼으로 바로 조작하세요.",
            )
            return

        domain_id = device["domain_id"]
        ip_address = device["host"]
        if self._run_automated_ssh(
            session_name="pinky_teleop",
            command=f"export ROS_DOMAIN_ID={domain_id}; ros2 run teleop_twist_keyboard teleop_twist_keyboard",
            title=f"AMR Teleop (Domain: {domain_id})",
            ip_address=ip_address
        ):
            messagebox.showinfo("제어 연결", f"ROS 브리지가 준비되지 않아 {ip_address}(도메인 {domain_id})으로 SSH Teleop을 실행합니다.")
        else:
            messagebox.showerror(
                "제어 연결 실패",
                "ROS 브리지가 준비되지 않았고, 대체 경로인 tmux도 없습니다.\n"
                "중앙 관제 스택과 zenoh 브릿지가 실행 중인지 확인하세요.",
            )

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
        device = self._selected_amr()
        if char in velocity_commands and device and self.ros_bridge.snapshot().ready:
            linear, angular = velocity_commands[char]
            # Publishes /<vehicle_id>/cmd_vel_manual, which the vehicle's
            # cmd_vel_safety_gate feeds into /<vehicle_id>/cmd_vel. The topic
            # must be listed in the zenoh allow-lists or it never leaves the
            # laptop - see config/network/zenoh_*.json5.
            if self.ros_bridge.send_velocity(
                linear, angular, vehicle_id=device["vehicle_id"]
            ):
                return

        import subprocess
        tmux = _tmux_path()
        if not tmux:
            print(
                "Manual drive unavailable: ROS bridge not ready and tmux is "
                "not installed."
            )
            return
        try:
            subprocess.Popen([tmux, "send-keys", "-t", "pinky_teleop", char], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print(f"Failed to send command: {e}")

    def _activate_emergency_stop(self) -> None:
        from tkinter import messagebox

        # This button is the fleet-wide stop, so it must latch every vehicle.
        # It used to derive a single vehicle_id from the dropdown, which left
        # the unselected AMR running.
        if self.ros_bridge.emergency_stop("fleet"):
            messagebox.showwarning(
                "비상 정지",
                "전체 AMR 정지 신호를 유지하고 ARM1·ARM2 작업 큐를 초기화합니다.\n"
                "해제 후 이전 작업은 자동으로 재개되지 않습니다.",
            )
        else:
            messagebox.showerror(
                "비상 정지 실패",
                "ROS 브리지가 연결되지 않았습니다.",
            )

    def _release_emergency_stop(self) -> None:
        from tkinter import messagebox

        # A vehicle stays stopped while ANY latch is set, and they live in
        # different nodes. Release all of them, like
        # scripts/clear_all_holds.sh --cancel-goals.
        if not self.ros_bridge.snapshot().ready:
            messagebox.showerror(
                "해제 실패",
                "ROS 브리지가 연결되지 않았습니다.",
            )
            return

        import threading

        def worker():
            results = self.ros_bridge.clear_all_holds(cancel_goals=True)
            self.after(0, lambda: self._show_hold_release_result(results))

        threading.Thread(target=worker, daemon=True).start()

    def _show_hold_release_result(self, results) -> None:
        from tkinter import messagebox

        if results is None:
            messagebox.showerror(
                "해제 실패",
                "ROS 브리지가 준비되지 않아 잠금을 해제하지 못했습니다.",
            )
            return

        failed = [
            f"  · {label}: {detail or '실패'}"
            for label, ok, detail in results
            if not ok
        ]
        cleared = len(results) - len(failed)
        if failed:
            messagebox.showwarning(
                "일부 해제 실패",
                f"{cleared}/{len(results)}개 잠금을 해제했습니다.\n\n"
                "실패 항목:\n" + "\n".join(failed) + "\n\n"
                "해당 노드가 실행 중인지 확인하세요.",
            )
        else:
            messagebox.showinfo(
                "비상 정지 해제",
                f"모든 잠금 {cleared}개를 해제했습니다.\n"
                "(충돌 정지 · 비상정지 · 구역 잠금 · 진행 중 주행 목표)\n\n"
                "이동 전에 주변 안전을 확인하세요.",
            )

    def _call_crane_service(
        self,
        device_name: str,
        operation: str,
    ) -> None:
        from tkinter import messagebox

        namespace = "/arm2" if "#02" in device_name else "/arm"
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
