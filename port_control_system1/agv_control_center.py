"""
agv_control_center.py

다섯 가지 화면을 사이드바 탭으로 전환하며 쓰는 하나의 화면으로 통합했습니다.

  📊 대시보드                  -> dashboard_view.py (DashboardView)
  📷 CCTV 실시간 모니터링       -> cctv_monitor_view.py (CCTVMonitorView)
  🌐 통합 관제 (레이더뷰)       -> integrated_radar_view.py (IntegratedRadarView)
  🚚 화물 위치 / 배차          -> cargo_dispatch_tool.py (CargoDispatchTool)
  ⚠️ 비상상황 대처             -> emergency_control_view.py (EmergencyControlView)

  ⚙️ 설정 (팝업)              -> settings_popup.py (SettingsPopup)
    - 🎨 인터페이스 설정       : 테마(다크/라이트/시스템), UI 배율, 창 크기
    - 📍 위치 마킹 / 캘리브레이션 : dual_view_calibrator.py (DualViewCalibrator)
    - 🔀 경유지 규칙 설정       : waypoint_rules_editor.py (WaypointRulesEditor)

(대시보드/CCTV/통합 관제/비상상황 4개는 팀원이 만든 스마트 항만 통합 관제
시스템의 기능을 분석해서 우리 프로젝트 스타일로 다시 만든 것입니다. 팀원
쪽은 별도 FastAPI LLM 서버(llm_bridge.py -> llm_server.py)로 자연어를
처리했지만, 여기서는 우리가 이미 쓰던 command_center.py의 "🗣️ 명령" 팝업
하나로 전부 통일했습니다 - 화물/위치/경유지 규칙까지 다 아는 우리 쪽 LLM
파서가 훨씬 정교해서, 그걸 그대로 재사용하는 게 낫다고 판단했습니다.)

자연어 명령은 별도 사이드바 탭이 아니라, 각 화면 우측 하단에 있는
"🗣️ 명령" 버튼을 누르면 뜨는 팝업(command_center.py의 CommandPopup)에서
처리합니다. 어느 화면에서 눌러도 같은 팝업이 뜨고, 화물 명령을 실행하면
그 즉시 화물 위치가 cargo_locations.json에 반영됩니다.

위치 마킹/캘리브레이션과 경유지 규칙 설정은 사이드바 하단의 "⚙️ 설정" 버튼을
누르면 뜨는 팝업(settings_popup.py의 SettingsPopup) 안에서 탭으로 전환하며
사용합니다. 인터페이스 테마/크기 설정도 같은 팝업에서 관리합니다.

기존 main.py(스마트 항만 통합 관제 시스템)의 "사이드바 + content_frame에 뷰를
갈아끼우는" 구조를 그대로 따랐습니다.

실행:
    python agv_control_center.py

같은 폴더에 다음 파일들이 함께 있어야 합니다:
    command_center.py, dual_view_calibrator.py, waypoint_rules_editor.py,
    cargo_dispatch_tool.py, pixel_to_map.py, slam_map_pixel_to_world.py,
    waypoint_rules.py, generate_cargo_template.py, llm_command_parser.py,
    dashboard_view.py, cctv_monitor_view.py, integrated_radar_view.py,
    emergency_control_view.py, settings_popup.py,
    frame_00014.jpg, current_map.yaml, current_map.png

추가 패키지: requests (대시보드의 기상청/해양조사원 API 호출용)
"""

import customtkinter as ctk

from dashboard_view import DashboardView
from cctv_monitor_view import CCTVMonitorView
from integrated_radar_view import IntegratedRadarView
from cargo_dispatch_tool import CargoDispatchTool
from emergency_control_view import EmergencyControlView
from stream_view import StreamView
from slam_stream_processor import SlamStreamProcessor
from settings_view import SettingsView
from ros_control_bridge import RosControlBridge

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")


class AGVControlCenter(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("스마트 항만 통합 관제 시스템")
        self.geometry("1400x900")
        self.configure(fg_color="#131314")
        self.ros_bridge = RosControlBridge.get_instance()
        self.ros_bridge.start()

        # 전체 창을 [좌: 사이드바(고정폭) | 우: 내용 영역(가변폭)] 2열 그리드로 구성
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        self.nav_buttons = {}
        self.setup_sidebar()

        # 오른쪽 큰 영역 (메인 콘텐츠 영역)
        self.main_area = ctk.CTkFrame(self, fg_color="#131314", corner_radius=0)
        self.main_area.grid(row=0, column=1, sticky="nsew")
        self.main_area.grid_columnconfigure(0, weight=1)
        self.main_area.grid_rowconfigure(1, weight=1)
        
        # 상단 바 (Header)
        self.header = ctk.CTkFrame(self.main_area, height=64, fg_color="transparent", corner_radius=0)
        self.header.grid(row=0, column=0, sticky="ew")
        self.header.pack_propagate(False)
        
        header_border = ctk.CTkFrame(self.main_area, height=1, fg_color="#39393a", corner_radius=0)
        header_border.grid(row=0, column=0, sticky="sew")
        
        title_label = ctk.CTkLabel(self.header, text="Smart Port Integrated Management", 
                                   font=ctk.CTkFont(family="Inter", size=16, weight="bold"), text_color="#e6e6e6")
        title_label.pack(side="left", padx=24)
        
        user_info = ctk.CTkFrame(self.header, fg_color="transparent")
        user_info.pack(side="right", padx=24)
        self.ros_status_label = ctk.CTkLabel(
            user_info,
            text="ROS 연결 중",
            font=ctk.CTkFont(family="Inter", size=12),
            text_color="#f0ad4e",
        )
        self.ros_status_label.pack(side="left", padx=8)
        ctk.CTkLabel(user_info, text="Operator OP-084", font=ctk.CTkFont(family="Inter", size=13, weight="normal"), text_color="#c4c4c5").pack(side="left", padx=16)
        avatar = ctk.CTkFrame(user_info, width=32, height=32, corner_radius=16, fg_color="#39393a", border_width=1, border_color="#39393a")
        avatar.pack(side="left")

        # 실제 콘텐츠가 들어갈 영역
        self.content_frame = ctk.CTkFrame(self.main_area, fg_color="transparent")
        self.content_frame.grid(row=1, column=0, sticky="nsew")
        self.content_frame.grid_columnconfigure(0, weight=1)
        self.content_frame.grid_rowconfigure(0, weight=1)

        self.current_view = None
        self.show_view("dashboard")
        self.after(500, self._update_ros_status)

    def setup_sidebar(self) -> None:
        self.sidebar = ctk.CTkFrame(self, width=280, fg_color="#1b1b1c", corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar.grid_rowconfigure(1, weight=1)
        self.sidebar.grid_columnconfigure(0, weight=1)
        self.sidebar.grid_propagate(False)

        border = ctk.CTkFrame(self.sidebar, width=1, fg_color="#39393a", corner_radius=0)
        border.place(relx=1.0, rely=0.0, relheight=1.0, anchor="ne")

        logo_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        logo_frame.grid(row=0, column=0, sticky="ew", padx=16, pady=(24, 20))
        ctk.CTkLabel(logo_frame, text="PORT CONTROL",
                    font=ctk.CTkFont(family="Inter", size=24, weight="bold"), text_color="#e6e6e6").pack(anchor="w")

        nav_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        nav_frame.grid(row=1, column=0, sticky="nsew", padx=8)

        menu_kwargs = {
            "font": ctk.CTkFont(family="Inter", size=14, weight="bold"),
            "anchor": "w", "height": 48, "corner_radius": 8,
            "fg_color": "transparent", "hover_color": "#39393a", "text_color": "#c4c4c5"
        }

        def create_nav_btn(key, text):
            btn = ctk.CTkButton(nav_frame, text=text, command=lambda: self.show_view(key), **menu_kwargs)
            btn.pack(fill="x", pady=2, padx=8)
            self.nav_buttons[key] = btn

        create_nav_btn("dashboard", "  대시보드")
        create_nav_btn("cctv", "  CCTV 실시간 모니터링")
        create_nav_btn("radar", "  통합 관제 (레이더뷰)")
        create_nav_btn("cargo", "  화물 위치 / 배차")
        create_nav_btn("emergency", "  비상상황 대처")
        create_nav_btn("stream", "  SLAM 모니터")
        create_nav_btn("settings", "  설정 (Settings)")

        bottom_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        bottom_frame.grid(row=2, column=0, sticky="ew")
        
        ctk.CTkFrame(bottom_frame, height=1, fg_color="#39393a", corner_radius=0).pack(fill="x")
        
        action_frame = ctk.CTkFrame(bottom_frame, fg_color="transparent")
        action_frame.pack(fill="x", padx=16, pady=16)

        ctk.CTkLabel(action_frame, text="어느 화면에서든 우측 하단\n🗣️ 명령 버튼 이용",
                    font=ctk.CTkFont(family="Malgun Gothic", size=12), text_color="#c4c4c5",
                    justify="center").pack(fill="x", pady=(0, 16))

        ctk.CTkButton(action_frame, text="EMERGENCY STOP", font=ctk.CTkFont(family="Inter", size=14, weight="bold"), 
                     fg_color="#e74c3c", hover_color="#c0392b", text_color="white", corner_radius=8,
                     command=self.activate_emergency_stop, height=48).pack(fill="x")

    def _update_ros_status(self) -> None:
        if not self.winfo_exists():
            return
        snapshot = self.ros_bridge.snapshot()
        if snapshot.ready:
            if snapshot.fleet_states:
                vehicles = " | ".join(
                    f"{vehicle_id}:{state}({x:.2f},{y:.2f})"
                    for vehicle_id, state, _battery, _emergency, x, y
                    in snapshot.fleet_states
                )
                text = f"{vehicles} | {snapshot.b1_zone}"
            else:
                text = (
                    "ROS 비상정지"
                    if snapshot.emergency_active
                    else "ROS 연결됨"
                )
            color = "#ff6b6b" if snapshot.emergency_active else "#61de8a"
        elif snapshot.error:
            text = "ROS 연결 실패"
            color = "#ff6b6b"
        else:
            text = "ROS 연결 중"
            color = "#f0ad4e"
        self.ros_status_label.configure(text=text, text_color=color)
        self.after(500, self._update_ros_status)

    def activate_emergency_stop(self) -> None:
        """Latch a local ROS cmd_vel stop and show the operator alert."""
        if self.ros_bridge.emergency_stop():
            self.show_emergency_popup(
                "EMERGENCY_STOP",
                "중앙관제에서 /cmd_vel 비상 정지를 활성화했습니다.",
            )
            return
        self.show_emergency_popup(
            "ROS_OFFLINE",
            "ROS 브리지가 연결되지 않아 정지 명령을 발행하지 못했습니다.",
        )

    def show_view(self, view_key: str) -> None:
        for key, btn in self.nav_buttons.items():
            if key == view_key:
                btn.configure(fg_color="#2e86c1", text_color="white", hover_color="#3498db")
            else:
                btn.configure(fg_color="transparent", text_color="#c4c4c5", hover_color="#39393a")

        if self.current_view is not None:
            if hasattr(self.current_view, "stop"):
                self.current_view.stop()
            self.current_view.destroy()

        if view_key == "dashboard":
            self.current_view = DashboardView(self.content_frame)
        elif view_key == "cctv":
            self.current_view = CCTVMonitorView(self.content_frame)
        elif view_key == "radar":
            self.current_view = IntegratedRadarView(self.content_frame)
        elif view_key == "cargo":
            # 화물 위치 등록 + 엑셀 일괄등록/재배치를 처리하는 화면
            self.current_view = CargoDispatchTool(self.content_frame)
        elif view_key == "emergency":
            self.current_view = EmergencyControlView(self.content_frame)
        elif view_key == "stream":
            self.current_view = StreamView(self.content_frame)
        elif view_key == "settings":
            self.current_view = SettingsView(self.content_frame)
        else:
            raise ValueError(f"알 수 없는 화면: {view_key}")

        # content_frame(오른쪽 큰 영역) 전체를 채우도록 배치
        self.current_view.grid(row=0, column=0, sticky="nsew")

    def simulate_emergency(self) -> None:
        """위험물 감지, 해무 등 비상상황 발생을 시뮬레이션합니다."""
        import random
        emergencies = [
            ("ERR-HAZARD-01", "🔥 구역 B 위험물 유출 및 화재 감지!"),
            ("ERR-FOG-002", "🌫️ 항구 앞바다 짙은 해무 발생 (가시거리 50m 이하)"),
            ("ERR-AGV-OBS", "🚧 AGV 주행 경로상 미확인 장애물 감지"),
        ]
        code, desc = random.choice(emergencies)
        self.show_emergency_popup(code, desc)

    def show_emergency_popup(self, code: str, description: str) -> None:
        """어떤 화면을 보고 있더라도 화면 중앙에 최상단으로 뜨는 비상상황 팝업"""
        popup = ctk.CTkToplevel(self)
        popup.title("🚨 비상상황 발생")
        popup.geometry("450x220")
        popup.attributes("-topmost", True)
        
        # 화면 중앙에 배치
        self.update_idletasks()
        x = self.winfo_x() + (self.winfo_width() - 450) // 2
        y = self.winfo_y() + (self.winfo_height() - 220) // 2
        popup.geometry(f"+{x}+{y}")
        
        # 내용
        ctk.CTkLabel(popup, text=f"⚠️ {code}", font=ctk.CTkFont(family="Arial Black", size=24), text_color="#EA5455").pack(pady=(25, 5))
        ctk.CTkLabel(popup, text=description, font=ctk.CTkFont(family="Malgun Gothic", size=15)).pack(pady=(5, 20))
        
        btn_frame = ctk.CTkFrame(popup, fg_color="transparent")
        btn_frame.pack()
        
        def on_handle():
            popup.destroy()
            self.show_view("emergency")  # 비상상황 대처 화면으로 즉시 전환
            
        def on_close():
            popup.destroy()
            
        ctk.CTkButton(btn_frame, text="대처 (Handle)", font=ctk.CTkFont(family="Malgun Gothic", size=14, weight="bold"),
                     fg_color="#EA5455", hover_color="#DC2626", command=on_handle, width=120, height=40).pack(side="left", padx=10)
        ctk.CTkButton(btn_frame, text="닫기 (Close)", font=ctk.CTkFont(family="Malgun Gothic", size=14),
                     fg_color="gray", hover_color="#4B5563", command=on_close, width=120, height=40).pack(side="left", padx=10)
        
        # 시스템 경고음
        popup.bell()


if __name__ == "__main__":
    app = AGVControlCenter()

    def on_closing():
        """앱 종료 시 SLAM 프로세서 등 백그라운드 자원을 정리합니다."""
        try:
            SlamStreamProcessor.get_instance().stop()
        except Exception:
            pass
        try:
            CCTVMonitorView.stop_capture()
        except Exception:
            pass
        try:
            RosControlBridge.get_instance().stop()
        except Exception:
            pass
        if hasattr(app, 'current_view') and app.current_view is not None:
            if hasattr(app.current_view, 'stop'):
                app.current_view.stop()
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", on_closing)
    app.mainloop()
