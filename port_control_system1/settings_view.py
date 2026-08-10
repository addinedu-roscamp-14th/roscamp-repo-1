"""
settings_view.py

기존 팝업(settings_popup.py) 대신 메인 화면에 표시되는 설정 화면입니다.
내부에 세 개의 탭이 있습니다:
  🎨 인터페이스    - 테마(다크/라이트/시스템), UI 크기 조절 등 일반 설정
  📡 스트림 주소   - CCTV 및 SLAM 스트림 URL 변경
  🔀 경유지 규칙   - 기존 WaypointRulesEditor 임베드
"""

import customtkinter as ctk
import json
import os
from tkinter import messagebox
from waypoint_rules_editor import WaypointRulesEditor

class SettingsView(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color="#242424", **kwargs)
        
        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=24, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=16, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=13)
        self.font_mono = ctk.CTkFont(family="Consolas", size=13)
        
        self._embedded_views = []
        self._build_ui()
        self.switch_tab("interface")

    def _build_ui(self) -> None:
        # 상단 헤더
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.pack(fill="x", padx=30, pady=(25, 15))
        
        title_frame = ctk.CTkFrame(header, fg_color="transparent")
        title_frame.pack(side="left")
        ctk.CTkLabel(title_frame, text="⚙️ 설정 (Settings)", font=self.font_title, text_color="#dce4ee").pack(side="left")
        
        ctk.CTkLabel(header, text="인터페이스 설정, 스트림 주소 설정, 경유지 규칙을 관리합니다.",
                     font=self.font_body, text_color="#909090").pack(side="left", padx=20, pady=(8, 0))

        # 탭 뷰 컨테이너 (bg-ctk-frame)
        self.main_container = ctk.CTkFrame(self, fg_color="#2b2b2b", corner_radius=6)
        self.main_container.pack(fill="both", expand=True, padx=30, pady=(0, 30))
        
        # 탭 버튼 바 (bg-ctk-widget)
        self.tab_bar = ctk.CTkFrame(self.main_container, fg_color="#343638", corner_radius=0)
        self.tab_bar.pack(fill="x")
        
        # 탭 버튼 프레임 내부 컨테이너 (gap-2 구현)
        self.tab_btn_container = ctk.CTkFrame(self.tab_bar, fg_color="transparent")
        self.tab_btn_container.pack(side="left", padx=10, pady=10)

        self.tab_buttons = {}
        self.tabs = {}
        
        # 탭 버튼 생성
        self._create_tab_button("interface", "🎨 인터페이스 (Interface)")
        self._create_tab_button("stream", "📡 스트림 주소 설정 (Stream Config)")
        self._create_tab_button("waypoint", "🔀 경유지 규칙 설정 (Waypoint Rules)")
        
        # 탭 내용 프레임들 컨테이너
        self.content_area = ctk.CTkFrame(self.main_container, fg_color="transparent")
        self.content_area.pack(fill="both", expand=True, padx=30, pady=30)
        self.content_area.grid_columnconfigure(0, weight=1)
        self.content_area.grid_rowconfigure(0, weight=1)

        # 각 탭 내용 생성
        self._build_interface_tab()
        self._build_stream_tab()
        self._build_waypoint_tab()

    def _create_tab_button(self, tab_id: str, text: str):
        btn = ctk.CTkButton(
            self.tab_btn_container, text=text, font=self.font_body,
            fg_color="transparent", text_color="#dce4ee", hover_color="#242424",
            command=lambda: self.switch_tab(tab_id),
            height=36, corner_radius=6
        )
        btn.pack(side="left", padx=5)
        self.tab_buttons[tab_id] = btn

    def switch_tab(self, tab_id: str):
        # 모든 버튼 색상 초기화
        for tid, btn in self.tab_buttons.items():
            btn.configure(fg_color="transparent", text_color="#dce4ee")
        # 선택된 버튼 색상 적용
        self.tab_buttons[tab_id].configure(fg_color="#2e86c1", text_color="white")
        
        # 모든 탭 컨텐츠 숨기기
        for tid, frame in self.tabs.items():
            frame.grid_remove()
        
        # 선택된 탭 보이기
        if tab_id in self.tabs:
            self.tabs[tab_id].grid(row=0, column=0, sticky="nsew")

    # ------------------------------------------------------------------
    # 탭 1: 인터페이스 설정
    # ------------------------------------------------------------------
    def _build_interface_tab(self):
        tab = ctk.CTkFrame(self.content_area, fg_color="transparent")
        self.tabs["interface"] = tab
        
        # 테마 설정 섹션
        theme_sec = ctk.CTkFrame(tab, fg_color="transparent")
        theme_sec.pack(fill="x", pady=(0, 30))
        ctk.CTkLabel(theme_sec, text="🎨 테마 설정 (Theme)", font=self.font_subtitle, text_color="#dce4ee").pack(anchor="w", pady=(0, 15))
        
        theme_grid = ctk.CTkFrame(theme_sec, fg_color="transparent")
        theme_grid.pack(fill="x", padx=30)
        theme_grid.grid_columnconfigure((0,1), weight=1)
        
        # 외관 모드
        app_frame = ctk.CTkFrame(theme_grid, fg_color="transparent")
        app_frame.grid(row=0, column=0, sticky="ew", padx=(0, 20))
        ctk.CTkLabel(app_frame, text="외관 모드 (Appearance)", font=self.font_body, text_color="#909090").pack(anchor="w", pady=(0, 5))
        self.appearance_var = ctk.StringVar(value=ctk.get_appearance_mode())
        ctk.CTkOptionMenu(
            app_frame, variable=self.appearance_var, values=["Dark", "Light", "System"],
            command=self._on_appearance_change,
            fg_color="#343638", button_color="#343638", button_hover_color="#2e86c1", text_color="#dce4ee"
        ).pack(fill="x")
        
        # 색상 테마
        color_frame = ctk.CTkFrame(theme_grid, fg_color="transparent")
        color_frame.grid(row=0, column=1, sticky="ew", padx=(20, 0))
        ctk.CTkLabel(color_frame, text="색상 테마 (Color Accent)", font=self.font_body, text_color="#909090").pack(anchor="w", pady=(0, 5))
        self.color_theme_var = ctk.StringVar(value="blue")
        ctk.CTkOptionMenu(
            color_frame, variable=self.color_theme_var, values=["blue (ICS Standard)", "dark-blue", "green"],
            command=self._on_color_theme_change,
            fg_color="#343638", button_color="#343638", button_hover_color="#2e86c1", text_color="#dce4ee"
        ).pack(fill="x")

        # UI 크기 설정 섹션
        scale_sec = ctk.CTkFrame(tab, fg_color="transparent")
        scale_sec.pack(fill="x", pady=(0, 30))
        ctk.CTkLabel(scale_sec, text="🔲 UI 크기 설정 (Scale)", font=self.font_subtitle, text_color="#dce4ee").pack(anchor="w", pady=(0, 15))
        
        scale_grid = ctk.CTkFrame(scale_sec, fg_color="transparent")
        scale_grid.pack(fill="x", padx=30)
        scale_grid.grid_columnconfigure((0,1), weight=1)
        
        # 스케일 조절
        scale_frame = ctk.CTkFrame(scale_grid, fg_color="transparent")
        scale_frame.grid(row=0, column=0, sticky="ew", padx=(0, 20))
        
        scale_header = ctk.CTkFrame(scale_frame, fg_color="transparent")
        scale_header.pack(fill="x", pady=(0, 5))
        ctk.CTkLabel(scale_header, text="UI 배율 (Scale)", font=self.font_body, text_color="#909090").pack(side="left")
        self.scale_label = ctk.CTkLabel(scale_header, text="100%", font=self.font_body, text_color="#dce4ee")
        self.scale_label.pack(side="right")
        
        self.scale_var = ctk.DoubleVar(value=1.0)
        ctk.CTkSlider(
            scale_frame, from_=0.8, to=1.5, number_of_steps=7,
            variable=self.scale_var, command=self._on_scale_change,
            fg_color="#343638", button_color="#2e86c1", button_hover_color="#1f6aa5"
        ).pack(fill="x", pady=(5, 0))
        
        # 창 크기
        win_frame = ctk.CTkFrame(scale_grid, fg_color="transparent")
        win_frame.grid(row=0, column=1, sticky="ew", padx=(20, 0))
        ctk.CTkLabel(win_frame, text="창 크기 (Window Size)", font=self.font_body, text_color="#909090").pack(anchor="w", pady=(0, 5))
        self.window_size_var = ctk.StringVar(value="1400x900 (Default)")
        ctk.CTkOptionMenu(
            win_frame, variable=self.window_size_var, values=["1200x800", "1400x900 (Default)", "1600x1000", "1920x1080 (FHD)"],
            command=self._on_window_size_change,
            fg_color="#343638", button_color="#343638", button_hover_color="#2e86c1", text_color="#dce4ee"
        ).pack(fill="x")

        # 시스템 정보 섹션
        info_sec = ctk.CTkFrame(tab, fg_color="transparent")
        info_sec.pack(fill="x")
        ctk.CTkLabel(info_sec, text="ℹ️ 시스템 정보 (System Info)", font=self.font_subtitle, text_color="#dce4ee").pack(anchor="w", pady=(0, 15))
        
        info_text = "스마트 항만 통합 관제 시스템 v1.0 (Smart Port ICS)\nCustomTkinter\n자율주행차(AGV) 제어 및 화물 배차 관리 시스템\n\nBuild: 2023.10.24-prod"
        ctk.CTkLabel(info_sec, text=info_text, font=self.font_mono, text_color="#909090", justify="left").pack(anchor="w", padx=30)

    def _on_appearance_change(self, mode: str) -> None:
        ctk.set_appearance_mode(mode)

    def _on_color_theme_change(self, theme: str) -> None:
        if theme.startswith("blue"): theme = "blue"
        ctk.set_default_color_theme(theme)
        messagebox.showinfo("색상 테마 변경", f"색상 테마가 '{theme}'로 설정되었습니다.\n완전한 적용을 위해 프로그램을 재시작해주세요.")

    def _on_scale_change(self, value: float) -> None:
        percent = int(round(value * 100))
        self.scale_label.configure(text=f"{percent}%")
        ctk.set_widget_scaling(value)
        ctk.set_window_scaling(value)

    def _on_window_size_change(self, size_str: str) -> None:
        size_str = size_str.split(" ")[0]
        root = self.winfo_toplevel()
        if isinstance(root, ctk.CTk):
            root.geometry(size_str)

    # ------------------------------------------------------------------
    # 탭 2: 스트림 주소 설정
    # ------------------------------------------------------------------
    def _build_stream_tab(self):
        tab = ctk.CTkFrame(self.content_area, fg_color="transparent")
        self.tabs["stream"] = tab
        
        config_path = "stream_config.json"
        cctv_url = "http://192.168.0.60:8000/video"
        slam_url = "http://192.168.0.60:8000/slam/video"
        
        if os.path.exists(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    config = json.load(f)
                    cctv_url = config.get("cctv_url", cctv_url)
                    slam_url = config.get("slam_url", slam_url)
            except Exception as e:
                print(f"설정 로드 실패: {e}")

        form_frame = ctk.CTkFrame(tab, fg_color="transparent")
        form_frame.pack(fill="x", padx=20, pady=20)
        form_frame.grid_columnconfigure(1, weight=1)

        # CCTV Stream
        ctk.CTkLabel(form_frame, text="CCTV 스트림 주소:", font=self.font_body, text_color="#909090").grid(row=0, column=0, padx=(0, 20), pady=15, sticky="e")
        self.cctv_url_entry = ctk.CTkEntry(form_frame, font=self.font_mono, fg_color="#343638", border_width=1, border_color="#343638", text_color="#dce4ee", height=36)
        self.cctv_url_entry.insert(0, cctv_url)
        self.cctv_url_entry.grid(row=0, column=1, sticky="ew", pady=15)

        # SLAM Stream
        ctk.CTkLabel(form_frame, text="SLAM 스트림 주소:", font=self.font_body, text_color="#909090").grid(row=1, column=0, padx=(0, 20), pady=15, sticky="e")
        self.slam_url_entry = ctk.CTkEntry(form_frame, font=self.font_mono, fg_color="#343638", border_width=1, border_color="#343638", text_color="#dce4ee", height=36)
        self.slam_url_entry.insert(0, slam_url)
        self.slam_url_entry.grid(row=1, column=1, sticky="ew", pady=15)

        # 저장 버튼
        btn_frame = ctk.CTkFrame(tab, fg_color="transparent")
        btn_frame.pack(fill="x", pady=40)
        ctk.CTkButton(btn_frame, text="💾 저장 (Save Configuration)", font=self.font_subtitle, 
                      fg_color="#2e86c1", hover_color="#1f6aa5", height=45, width=250,
                      command=self._save_stream_config).pack()

    def _save_stream_config(self) -> None:
        new_cctv_url = self.cctv_url_entry.get().strip()
        new_slam_url = self.slam_url_entry.get().strip()
        config = {"cctv_url": new_cctv_url, "slam_url": new_slam_url}
        try:
            with open("stream_config.json", "w", encoding="utf-8") as f:
                json.dump(config, f, indent=4)
                
            from slam_stream_processor import SlamStreamProcessor
            processor = SlamStreamProcessor.get_instance()
            if processor.is_running:
                processor.start(new_slam_url)
                
            import cctv_monitor_view
            cctv_monitor_view.CAMERA_SOURCES["맵 스트림 (API)"] = new_cctv_url
            
            # 메인 앱(AGVControlCenter) 뷰 찾기 및 갱신
            main_window = self.winfo_toplevel()
            if hasattr(main_window, "current_view"):
                view = main_window.current_view
                if isinstance(view, cctv_monitor_view.CCTVMonitorView):
                    if view.camera_selector.get() == "맵 스트림 (API)":
                        view._switch_source(new_cctv_url)
                        
                from stream_view import StreamView
                if isinstance(view, StreamView):
                    view.url_entry.delete(0, 'end')
                    view.url_entry.insert(0, new_slam_url)
                    if processor.is_running:
                        view._connect()

            messagebox.showinfo("저장 완료", "스트림 주소가 저장되고 실시간으로 반영되었습니다.")
        except Exception as e:
            messagebox.showerror("저장 실패", f"설정 저장 중 오류가 발생했습니다:\n{e}")

    # ------------------------------------------------------------------
    # 탭 3: 경유지 규칙 설정
    # ------------------------------------------------------------------
    def _build_waypoint_tab(self):
        tab = ctk.CTkFrame(self.content_area, fg_color="transparent")
        self.tabs["waypoint"] = tab
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(0, weight=1)
        
        # WaypointRulesEditor가 자체 타이틀을 가지고 있으므로,
        # settings_view.py 안에서는 어울리게 조금 조정해야 합니다. 
        # (waypoint_rules_editor.py를 같이 수정할 것입니다)
        editor = WaypointRulesEditor(tab)
        editor.grid(row=0, column=0, sticky="nsew")
        self._embedded_views.append(editor)

    def stop(self):
        for view in self._embedded_views:
            if hasattr(view, "stop"):
                view.stop()
