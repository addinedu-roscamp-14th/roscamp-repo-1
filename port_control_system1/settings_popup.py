"""
settings_popup.py

좌측 사이드바 하단의 "⚙️ 설정" 버튼을 누르면 뜨는 팝업 창입니다.
내부에 세 개의 탭이 있습니다:

  🎨 인터페이스    - 테마(다크/라이트/시스템), UI 크기 조절 등 일반 설정
  📡 스트림 주소   - CCTV 및 SLAM 스트림 URL 변경
  🔀 경유지 규칙   - 기존 WaypointRulesEditor를 팝업 안에 그대로 임베드
"""

import customtkinter as ctk

from waypoint_rules_editor import WaypointRulesEditor


class SettingsPopup(ctk.CTkToplevel):
    """설정 팝업 - 인터페이스 설정 + 스트림 주소 설정 + 경유지 규칙을 탭으로 구성."""

    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.title("⚙️ 설정")
        self.geometry("1300x850")
        self.attributes("-topmost", True)

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=20, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=14, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=12)

        self._embedded_views = []  # 탭 전환 시 정리할 뷰 목록
        self._build_ui()

    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.configure(fg_color="#242424")

        # 상단 타이틀
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.grid(row=0, column=0, sticky="ew", padx=20, pady=(15, 5))
        ctk.CTkLabel(header, text="⚙️ 설정 (Settings)", font=self.font_title, text_color="#dce4ee").pack(anchor="w")
        ctk.CTkLabel(header, text="인터페이스 설정, 스트림 주소 설정, 경유지 규칙을 관리합니다.",
                     font=self.font_body, text_color="#909090").pack(anchor="w", pady=(2, 0))

        # 탭뷰 (customtkinter의 CTkTabview)
        self.tabview = ctk.CTkTabview(self, corner_radius=6, fg_color="#2b2b2b", segmented_button_fg_color="#343638", segmented_button_selected_color="#2e86c1", segmented_button_selected_hover_color="#1f6aa5", text_color="#dce4ee")
        self.tabview.grid(row=1, column=0, sticky="nsew", padx=15, pady=(5, 15))

        # 탭 추가
        self.tabview.add("🎨 인터페이스")
        self.tabview.add("📡 스트림 주소 설정")
        self.tabview.add("🔀 경유지 규칙 설정")

        # 각 탭 내용 구성
        self._build_interface_tab()
        self._build_stream_tab()
        self._build_waypoint_tab()

        # 기본 탭을 인터페이스로 설정
        self.tabview.set("🎨 인터페이스")

    # ------------------------------------------------------------------
    # 탭 1: 인터페이스 설정
    # ------------------------------------------------------------------
    def _build_interface_tab(self) -> None:
        tab = self.tabview.tab("🎨 인터페이스")
        tab.grid_columnconfigure(0, weight=1)

        # --- 테마 설정 ---
        theme_frame = ctk.CTkFrame(tab, corner_radius=6, fg_color="transparent")
        theme_frame.pack(fill="x", padx=10, pady=(10, 5))
        theme_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(theme_frame, text="테마 설정 (Theme)", font=self.font_subtitle, text_color="#dce4ee").grid(
            row=0, column=0, columnspan=2, sticky="w", padx=15, pady=(15, 10))

        ctk.CTkLabel(theme_frame, text="외관 모드 (Appearance):", font=self.font_body, text_color="#909090").grid(
            row=1, column=0, sticky="w", padx=15, pady=(0, 5))
        self.appearance_var = ctk.StringVar(value=ctk.get_appearance_mode())
        appearance_menu = ctk.CTkOptionMenu(
            theme_frame, variable=self.appearance_var,
            values=["Dark", "Light", "System"],
            command=self._on_appearance_change,
            width=200, fg_color="#343638", button_color="#343638", button_hover_color="#2e86c1", text_color="#dce4ee"
        )
        appearance_menu.grid(row=1, column=1, sticky="w", padx=15, pady=(0, 5))

        ctk.CTkLabel(theme_frame, text="색상 테마 (Color Accent):", font=self.font_body, text_color="#909090").grid(
            row=2, column=0, sticky="w", padx=15, pady=(0, 15))
        self.color_theme_var = ctk.StringVar(value="blue")
        color_menu = ctk.CTkOptionMenu(
            theme_frame, variable=self.color_theme_var,
            values=["blue (ICS Standard)", "dark-blue", "green"],
            command=self._on_color_theme_change,
            width=200, fg_color="#343638", button_color="#343638", button_hover_color="#2e86c1", text_color="#dce4ee"
        )
        color_menu.grid(row=2, column=1, sticky="w", padx=15, pady=(0, 15))

        # --- UI 크기 설정 ---
        scale_frame = ctk.CTkFrame(tab, corner_radius=6, fg_color="transparent")
        scale_frame.pack(fill="x", padx=10, pady=5)
        scale_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(scale_frame, text="UI 크기 설정 (Scale)", font=self.font_subtitle, text_color="#dce4ee").grid(
            row=0, column=0, columnspan=3, sticky="w", padx=15, pady=(15, 10))

        ctk.CTkLabel(scale_frame, text="UI 배율 (Scale):", font=self.font_body, text_color="#909090").grid(
            row=1, column=0, sticky="w", padx=15, pady=(0, 5))

        self.scale_var = ctk.DoubleVar(value=1.0)
        self.scale_label = ctk.CTkLabel(scale_frame, text="100%", font=self.font_body, width=50, text_color="#dce4ee")
        self.scale_label.grid(row=1, column=2, sticky="w", padx=(5, 15), pady=(0, 5))

        scale_slider = ctk.CTkSlider(
            scale_frame, from_=0.8, to=1.5, number_of_steps=7,
            variable=self.scale_var, command=self._on_scale_change,
            width=300, fg_color="#343638", button_color="#2e86c1", button_hover_color="#1f6aa5"
        )
        scale_slider.grid(row=1, column=1, sticky="w", padx=15, pady=(0, 5))

        ctk.CTkLabel(scale_frame, text="창 크기 (Window Size):", font=self.font_body, text_color="#909090").grid(
            row=2, column=0, sticky="w", padx=15, pady=(0, 15))
        self.window_size_var = ctk.StringVar(value="1400x900")
        window_size_menu = ctk.CTkOptionMenu(
            scale_frame, variable=self.window_size_var,
            values=["1200x800", "1400x900 (Default)", "1600x1000", "1920x1080 (FHD)"],
            command=self._on_window_size_change,
            width=200, fg_color="#343638", button_color="#343638", button_hover_color="#2e86c1", text_color="#dce4ee"
        )
        window_size_menu.grid(row=2, column=1, sticky="w", padx=15, pady=(0, 15))

        # --- 정보 ---
        info_frame = ctk.CTkFrame(tab, corner_radius=6, fg_color="transparent")
        info_frame.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(info_frame, text="시스템 정보 (System Info)", font=self.font_subtitle, text_color="#dce4ee").pack(
            anchor="w", padx=15, pady=(15, 10))
        ctk.CTkLabel(info_frame,
                     text="스마트 항만 통합 관제 시스템 v1.0 (Smart Port ICS)\n"
                          "CustomTkinter\n"
                          "자율주행차(AMR) 제어 및 화물 배차 관리 시스템\n\n"
                          "Build: 2023.10.24-prod",
                     font=("Consolas", 12), text_color="#909090", justify="left").pack(
            anchor="w", padx=30, pady=(0, 15))

    def _on_appearance_change(self, mode: str) -> None:
        ctk.set_appearance_mode(mode)

    def _on_color_theme_change(self, theme: str) -> None:
        if theme.startswith("blue"): theme = "blue"
        from tkinter import messagebox
        ctk.set_default_color_theme(theme)
        messagebox.showinfo("색상 테마 변경", f"색상 테마가 '{theme}'로 설정되었습니다.\n"
                            "완전한 적용을 위해 프로그램을 재시작해주세요.")

    def _on_scale_change(self, value: float) -> None:
        percent = int(value * 100)
        self.scale_label.configure(text=f"{percent}%")
        ctk.set_widget_scaling(value)

    def _on_window_size_change(self, size_str: str) -> None:
        size_str = size_str.split(" ")[0]
        root = self.winfo_toplevel()
        main_window = self.master
        while main_window is not None:
            if isinstance(main_window, ctk.CTk):
                main_window.geometry(size_str)
                break
            main_window = getattr(main_window, "master", None)

    # ------------------------------------------------------------------
    # 탭 2: 스트림 주소 설정
    # ------------------------------------------------------------------
    def _build_stream_tab(self) -> None:
        tab = self.tabview.tab("📡 스트림 주소 설정")
        tab.grid_columnconfigure(1, weight=1)
        tab.configure(fg_color="transparent")

        import json
        import os

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

        ctk.CTkLabel(tab, text="CCTV 스트림 주소:", font=self.font_body, text_color="#909090").grid(row=0, column=0, padx=20, pady=(30, 10), sticky="e")
        self.cctv_url_entry = ctk.CTkEntry(tab, font=("Consolas", 13), width=400, fg_color="#343638", border_width=1, border_color="#2b2b2b", text_color="#dce4ee")
        self.cctv_url_entry.insert(0, cctv_url)
        self.cctv_url_entry.grid(row=0, column=1, padx=20, pady=(30, 10), sticky="ew")

        ctk.CTkLabel(tab, text="SLAM 스트림 주소:", font=self.font_body, text_color="#909090").grid(row=1, column=0, padx=20, pady=10, sticky="e")
        self.slam_url_entry = ctk.CTkEntry(tab, font=("Consolas", 13), width=400, fg_color="#343638", border_width=1, border_color="#2b2b2b", text_color="#dce4ee")
        self.slam_url_entry.insert(0, slam_url)
        self.slam_url_entry.grid(row=1, column=1, padx=20, pady=10, sticky="ew")

        save_btn = ctk.CTkButton(tab, text="저장 (Save Configuration)", font=self.font_subtitle, fg_color="#2e86c1", hover_color="#1f6aa5", height=40, command=self._save_stream_config)
        save_btn.grid(row=2, column=0, columnspan=2, pady=30)
    def _save_stream_config(self) -> None:
        import json
        from tkinter import messagebox
        new_cctv_url = self.cctv_url_entry.get().strip()
        new_slam_url = self.slam_url_entry.get().strip()
        config = {
            "cctv_url": new_cctv_url,
            "slam_url": new_slam_url
        }
        try:
            with open("stream_config.json", "w", encoding="utf-8") as f:
                json.dump(config, f, indent=4)
                
            # SLAM 실시간 반영
            from slam_stream_processor import SlamStreamProcessor
            processor = SlamStreamProcessor.get_instance()
            if processor.is_running:
                processor.start(new_slam_url)
                
            # CCTV 실시간 반영
            import cctv_monitor_view
            cctv_monitor_view.CAMERA_SOURCES["맵 스트림 (API)"] = new_cctv_url
            
            # 메인 앱(AGVControlCenter) 뷰 찾기 및 갱신
            main_window = self.master
            while main_window is not None:
                if hasattr(main_window, "current_view"):
                    break
                main_window = getattr(main_window, "master", None)
                
            if main_window and hasattr(main_window, "current_view"):
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
    def _build_waypoint_tab(self) -> None:
        tab = self.tabview.tab("🔀 경유지 규칙 설정")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(0, weight=1)

        editor = WaypointRulesEditor(tab)
        editor.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        self._embedded_views.append(editor)

    # ------------------------------------------------------------------
    def destroy(self) -> None:
        # 임베드된 뷰 중 stop() 메서드가 있으면 호출 (CCTV 스레드 정리 등)
        for view in self._embedded_views:
            if hasattr(view, "stop"):
                view.stop()
        super().destroy()


def open_settings_popup(master) -> SettingsPopup:
    """사이드바의 설정 버튼에서 호출하는 진입점."""
    return SettingsPopup(master)
