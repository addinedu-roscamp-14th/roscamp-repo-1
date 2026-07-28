"""
stream_view.py

SLAM 스트림 모니터 + 탐지 결과 시각화 전용 화면입니다.

좌측에 SLAM 영상 표시를 하고, 우측에 스트림 시작/중지 및 설정 제어를 합니다.

SlamStreamProcessor 싱글톤의 시작/중지를 이 화면에서 제어합니다.
"""

import customtkinter as ctk
import cv2
from PIL import Image

from slam_stream_processor import SlamStreamProcessor, DEFAULT_SLAM_URL


class StreamView(ctk.CTkFrame):
    """SLAM 스트림 모니터 뷰."""

    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=20, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=14, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=14)
        self.font_mini = ctk.CTkFont(family="Malgun Gothic", size=12, weight="bold")
        self.font_mono = ctk.CTkFont(family="Consolas", size=13)

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # ------------------------------------------------------------------
        # 상단 Header: 타이틀 + URL 입력 + 연결/중지 버튼
        # ------------------------------------------------------------------
        header = ctk.CTkFrame(self, fg_color="#1b1b1c", corner_radius=0, border_width=1, border_color="#404850")
        header.grid(row=0, column=0, sticky="ew")
        header.grid_columnconfigure(1, weight=1)

        title_frame = ctk.CTkFrame(header, fg_color="transparent")
        title_frame.grid(row=0, column=0, sticky="w", padx=24, pady=15)
        ctk.CTkLabel(title_frame, text="📡 SLAM 스트림 모니터", font=self.font_title, text_color="#e5e2e3").pack(side="left")

        control_panel = ctk.CTkFrame(header, fg_color="#1f1f20", corner_radius=8, border_width=1, border_color="#404850")
        control_panel.grid(row=0, column=1, sticky="e", padx=24, pady=10)

        ctk.CTkLabel(control_panel, text="SLAM URL:", font=self.font_mini, text_color="#c0c7d1").pack(side="left", padx=(10, 5), pady=5)
        
        self.url_entry = ctk.CTkEntry(control_panel, font=self.font_mono, width=320, height=30,
                                       fg_color="#131314", border_width=1, border_color="#404850", text_color="#e5e2e3")
        self.url_entry.pack(side="left", padx=5, pady=5)
        self.url_entry.insert(0, DEFAULT_SLAM_URL)

        self.connect_btn = ctk.CTkButton(
            control_panel, text="▶ 연결", width=80, height=30, font=self.font_mini,
            fg_color="#4497d3", hover_color="#2c7da0", text_color="#002c47", command=self._connect)
        self.connect_btn.pack(side="left", padx=5, pady=5)
        
        self.disconnect_btn = ctk.CTkButton(
            control_panel, text="⏹ 중지", width=80, height=30, font=self.font_mini,
            fg_color="#ffb4ab", hover_color="#ff897d", text_color="#690005", command=self._disconnect)
        self.disconnect_btn.pack(side="left", padx=(0, 5), pady=5)

        # ------------------------------------------------------------------
        # 하단 Content Layout
        # ------------------------------------------------------------------
        content_frame = ctk.CTkFrame(self, fg_color="transparent")
        content_frame.grid(row=1, column=0, sticky="nsew", padx=24, pady=24)
        content_frame.grid_columnconfigure(0, weight=1)
        content_frame.grid_rowconfigure(0, weight=1)

        # 좌측: SLAM 영상
        video_frame = ctk.CTkFrame(content_frame, fg_color="#0a1122", corner_radius=8, border_width=1, border_color="#404850")
        video_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 24))

        self.video_label = ctk.CTkLabel(
            video_frame,
            text="SLAM 영상 대기 중...\nURL을 입력하고 '연결' 버튼을 누르세요.",
            text_color="#c0c7d1", font=self.font_body)
        self.video_label.pack(expand=True, fill="both")

        # 우측: 상태 표시 및 메뉴 (Right Sidebar Panel)
        right_panel = ctk.CTkFrame(content_frame, fg_color="transparent", width=300)
        right_panel.grid(row=0, column=1, sticky="nsew")
        right_panel.grid_rowconfigure(1, weight=1)

        # 상태 카드
        status_card = ctk.CTkFrame(right_panel, fg_color="#2a2a2b", corner_radius=8, border_width=1, border_color="#404850")
        status_card.grid(row=0, column=0, sticky="ew")

        ctk.CTkLabel(status_card, text="상태 정보", font=self.font_mini, text_color="#92ccff").pack(anchor="w", padx=16, pady=(16, 5))

        self.status_label = ctk.CTkLabel(
            status_card, text="⏸ 연결 대기 중", font=self.font_body, text_color="#e5e2e3")
        self.status_label.pack(anchor="w", padx=16, pady=(0, 16))

        # 구분선
        ctk.CTkFrame(status_card, fg_color="#404850", height=1).pack(fill="x", padx=16)

        # Dummy Stats
        stats_frame = ctk.CTkFrame(status_card, fg_color="transparent")
        stats_frame.pack(fill="x", padx=16, pady=16)
        
        self._add_stat_row(stats_frame, "FPS", "--")
        self._add_stat_row(stats_frame, "Resolution", "--")
        self._add_stat_row(stats_frame, "Latency", "-- ms")

        # 하단 버튼
        btn_row = ctk.CTkFrame(right_panel, fg_color="transparent")
        btn_row.grid(row=1, column=0, sticky="sew")
                      
        ctk.CTkButton(btn_row, text="🗣️ 명령", font=self.font_subtitle, fg_color="#92ccff", text_color="#003351",
                      hover_color="#cce5ff", height=40, width=100, command=self.open_command_popup).pack(side="right", padx=(0, 10))

        # 이미 실행 중인 프로세서가 있으면 상태 반영
        processor = SlamStreamProcessor.get_instance()
        if processor.is_running:
            self.status_label.configure(text="▶ 스트림 수신 중", text_color="#61de8a")

        # 주기적 UI 갱신 시작
        self._update_loop()

    def _add_stat_row(self, parent, label_text, value_text):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=4)
        ctk.CTkLabel(row, text=label_text, font=self.font_mini, text_color="#c0c7d1").pack(side="left")
        ctk.CTkLabel(row, text=value_text, font=self.font_mono, text_color="#92ccff").pack(side="right")

    # ------------------------------------------------------------------
    # 연결 / 해제
    # ------------------------------------------------------------------
    def _connect(self) -> None:
        """SLAM 스트림 수신을 시작합니다."""
        url = self.url_entry.get().strip()
        if not url:
            self.status_label.configure(text="⚠️ URL을 입력해주세요", text_color="#ffb4ab")
            return

        processor = SlamStreamProcessor.get_instance()
        processor.start(url)
        self.status_label.configure(text="▶ 스트림 연결 중...", text_color="#ee671c")

    def _disconnect(self) -> None:
        """SLAM 스트림 수신을 중단합니다."""
        processor = SlamStreamProcessor.get_instance()
        processor.stop()

        self.video_label.configure(
            text="SLAM 영상 대기 중...\nURL을 입력하고 '연결' 버튼을 누르세요.",
            text_color="#c0c7d1", image=None)
        self.status_label.configure(text="⏸ 연결 해제됨", text_color="#c0c7d1")

    # ------------------------------------------------------------------
    # 주기적 UI 갱신
    # ------------------------------------------------------------------
    def _update_loop(self) -> None:
        """프레임 표시 갱신."""
        if not self.winfo_exists():
            return

        processor = SlamStreamProcessor.get_instance()

        frame = SlamStreamProcessor.SHARED_SLAM_FRAME

        if frame is not None and processor.is_running:
            try:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)

                MAX_W, MAX_H = 800, 600
                available_w = max(self.video_label.winfo_width(), 320)
                available_h = max(self.video_label.winfo_height(), 240)
                w = min(available_w, MAX_W)
                h = min(available_h, MAX_H)

                ctk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(w, h))
                self.video_label.configure(image=ctk_img, text="")
                self.video_label.image = ctk_img
            except Exception:
                pass

            self.status_label.configure(
                text="▶ 수신 중", text_color="#61de8a")

        self.after(100, self._update_loop)

    # ------------------------------------------------------------------
    # 정리
    # ------------------------------------------------------------------
    def stop(self) -> None:
        """화면이 닫힐 때 호출됩니다. (프로세서는 싱글톤이므로 여기서 중지하지 않음)"""
        pass  # SlamStreamProcessor는 다른 화면에서도 참조하므로 여기서 stop하지 않음

    def destroy(self) -> None:
        super().destroy()

    # ------------------------------------------------------------------
    # 명령 / 설정
    # ------------------------------------------------------------------
    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        open_command_popup(self)
