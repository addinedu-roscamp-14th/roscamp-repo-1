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

import threading
from datetime import datetime, timedelta

import customtkinter as ctk
import cv2
import requests
import psycopg2
from PIL import Image

from cctv_monitor_view import CCTVMonitorView

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

# 기상청(KMA) / 해양조사원(KHOA) API 키 - 팀원이 쓰던 것을 그대로 재사용합니다.
_WEATHER_API_KEY = "7d6c81a1615ce0a0dccd7e35568f7c3714d3f37ae515ce17e25a065005419ae3"


class DashboardView(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color=BG_SURFACE, **kwargs)

        # CCTV 백그라운드 캡처가 아직 안 돌고 있으면 시작 (대시보드에서도 영상이 보이도록)
        CCTVMonitorView.ensure_capture_running()

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
        self.map_frame.pack_propagate(False)

        self.live_feed_label = ctk.CTkLabel(
            self.map_frame,
            text="[📷 실시간 영상 연동 대기 중]\n'설정 > 스트림 설정'에서 카메라 URL을 확인해주세요.",
            text_color=TEXT_SECONDARY, font=self.font_subtitle,
        )
        self.live_feed_label.pack(expand=True, fill="both", padx=5, pady=5)

        self._ui_alive = True
        self.update_live_feed()  # 주기적으로 CCTVMonitorView.SHARED_FRAME을 읽어와 표시

        # ==========================================
        # 2. 우측: 작동 현황 카드 + 명령 버튼
        # ==========================================
        status_frame = ctk.CTkFrame(self, fg_color=BG_PANEL, border_width=1, border_color=BORDER_COLOR, corner_radius=12)
        status_frame.grid(row=0, column=1, rowspan=2, padx=(0, 15), pady=(15, 10), sticky="nsew")

        title_frame = ctk.CTkFrame(status_frame, fg_color="transparent")
        title_frame.pack(fill="x", padx=15, pady=(15, 10))
        ctk.CTkLabel(title_frame, text="📊 실시간 작동 현황", font=self.font_subtitle, text_color=TEXT_PRIMARY).pack(side="left")

        cards = [
            ("자율주행 차량 (AGV)", "가동 현황은 '화물 위치 / 배차' 탭 참고"),
            ("안벽 크레인", "6기 가동 중"),
            ("야드 크레인", "12기 가동 중"),
            ("당일 선박 입항", "총 5척 정박 완료"),
        ]
        for title, val in cards:
            card = ctk.CTkFrame(status_frame, fg_color=BG_CARD, border_width=1, border_color=BORDER_COLOR, corner_radius=8)
            card.pack(fill="x", padx=15, pady=5, ipady=8)
            ctk.CTkLabel(card, text=title, font=self.font_mini, text_color=TEXT_SECONDARY).pack(anchor="w", padx=10, pady=(0, 2))
            ctk.CTkLabel(card, text=val, font=self.font_body_bold if "가동" in val or "완료" in val else self.font_body, text_color=TEXT_PRIMARY).pack(anchor="w", padx=10)

        # ----------------------------------------------------
        # DB 연동: 대기 중인 작업 (PENDING) 추가 및 현황
        # ----------------------------------------------------
        pending_label = ctk.CTkLabel(status_frame, text="📦 실시간 화물 목록 (Cargos)", font=self.font_subtitle, text_color=TEXT_PRIMARY)
        pending_label.pack(anchor="w", padx=15, pady=(10, 5))

        # 작업 추가(Insert) 영역
        insert_frame = ctk.CTkFrame(status_frame, fg_color="transparent")
        insert_frame.pack(fill="x", padx=15, pady=(0, 5))
        
        self.name_entry = ctk.CTkEntry(insert_frame, placeholder_text="화물명", font=self.font_mini, height=30, width=80)
        self.name_entry.pack(side="left", expand=True, fill="x", padx=(0, 5))

        self.loc_entry = ctk.CTkEntry(insert_frame, placeholder_text="위치", font=self.font_mini, height=30, width=80)
        self.loc_entry.pack(side="left", expand=True, fill="x", padx=(0, 5))
        
        self.aruco_entry = ctk.CTkEntry(insert_frame, placeholder_text="ArUco", font=self.font_mini, height=30, width=60)
        self.aruco_entry.pack(side="left", expand=True, fill="x", padx=(0, 5))
        
        self.submit_button = ctk.CTkButton(insert_frame, text="추가", font=self.font_mini, width=50, height=30, fg_color=ACCENT_BLUE, text_color=ACCENT_ON_BLUE, hover_color="#cce5ff", command=self.add_task_to_db)
        self.submit_button.pack(side="right")

        self.pending_textbox = ctk.CTkTextbox(status_frame, fg_color=BG_CARD_INNER, text_color=TEXT_SECONDARY, font=self.font_mini, border_width=1, border_color=BORDER_COLOR)
        self.pending_textbox.pack(expand=True, fill="both", padx=15, pady=(0, 5))
        self.pending_textbox.configure(state="disabled")

        # Push buttons to the bottom
        # (기존의 팽창하는 투명 프레임은 삭제 또는 유지 가능, 여기선 textbox가 expand=True이므로 생략 가능하나 버튼 레이아웃을 위해 아주 작은 여백만 남깁니다)
        ctk.CTkFrame(status_frame, fg_color="transparent", height=10).pack(fill="both")

        divider = ctk.CTkFrame(status_frame, height=1, fg_color=BORDER_COLOR)
        divider.pack(fill="x", padx=15, pady=(0, 10))

        btn_row = ctk.CTkFrame(status_frame, fg_color="transparent")
        btn_row.pack(fill="x", padx=15, pady=(0, 15))
        
        btn_cmd = ctk.CTkButton(btn_row, text="🗣️ 명령", font=self.font_subtitle, fg_color=ACCENT_BLUE, text_color=ACCENT_ON_BLUE,
                                hover_color="#cce5ff", height=40, command=self.open_command_popup)
        btn_cmd.pack(side="left", expand=True, fill="x", padx=(0, 8))

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

        # DB 업데이트 루프 시작
        self.fetch_pending_tasks_loop()

    # ------------------------------------------------------------------
    # DB 연동: PENDING 작업 대기열 조회
    # ------------------------------------------------------------------
    def fetch_pending_tasks_loop(self) -> None:
        if not getattr(self, "_ui_alive", True):
            return
            
        def _db_task():
            try:
                # TODO: 실제 DB 접속 정보로 변경하세요
                conn = psycopg2.connect(
                    host="localhost",
                    database="port_db",
                    user="postgres",
                    password="1234",
                    port="5432"
                )
                cursor = conn.cursor()
                cursor.execute("SELECT name, location, container_id, base_aruco_id, floor FROM cargos ORDER BY location, floor")
                records = cursor.fetchall()
                cursor.close()
                conn.close()

                display_text = ""
                if records:
                    current_loc = None
                    for row in records:
                        name, location, cid, base, floor = row
                        # 위치가 바뀌면 구분선 추가
                        if location != current_loc:
                            if current_loc is not None:
                                display_text += "\n"
                            display_text += f"📍 [{location}]\n"
                            current_loc = location
                        
                        aruco = f" [ArUco:{cid}]" if cid else ""
                        base_info = f" ← Base:{base}" if base else ""
                        display_text += f"  {'  ' * (floor - 1)}📦 {floor}층: {name}{aruco}{base_info}\n"
                else:
                    display_text = "현재 등록된 화물이 없습니다."
                
                self.after(0, self._update_pending_ui, display_text)
            except Exception as e:
                self.after(0, self._update_pending_ui, f"DB 오류: {e}")

        # 메인 스레드 블로킹을 막기 위해 조회는 백그라운드 스레드에서 실행
        threading.Thread(target=_db_task, daemon=True).start()
        
        # 1초 뒤 재귀 호출 (무한 루프)
        self.after(1000, self.fetch_pending_tasks_loop)

    def _update_pending_ui(self, text: str) -> None:
        if not getattr(self, "_ui_alive", True):
            return
        try:
            current_text = self.pending_textbox.get("1.0", "end-1c")
            if current_text.strip() != text.strip():
                try:
                    scroll_pos = self.pending_textbox.yview()
                except Exception:
                    scroll_pos = (0.0,)
                
                self.pending_textbox.configure(state="normal")
                self.pending_textbox.delete("1.0", "end")
                self.pending_textbox.insert("1.0", text)
                self.pending_textbox.configure(state="disabled")
                
                try:
                    self.pending_textbox.yview_moveto(scroll_pos[0])
                except Exception:
                    pass
        except Exception as e:
            print(f"UI 업데이트 에러: {e}")

    def add_task_to_db(self) -> None:
        name = self.name_entry.get().strip()
        location = self.loc_entry.get().strip()
        aruco = self.aruco_entry.get().strip()
        
        if not name:
            return
        if not location:
            location = "대기장소"

        def _insert_task():
            try:
                # TODO: 실제 DB 접속 정보로 변경하세요
                conn = psycopg2.connect(
                    host="localhost",
                    database="port_db",
                    user="postgres",
                    password="1234",
                    port="5432"
                )
                cursor = conn.cursor()
                
                # 같은 위치에 이미 화물이 있으면 자동으로 그 위 층수로 올림
                cursor.execute("""
                    SELECT COALESCE(MAX(floor), 0), 
                           (SELECT container_id FROM cargos WHERE location = %s ORDER BY floor DESC LIMIT 1)
                    FROM cargos WHERE location = %s
                """, (location, location))
                row = cursor.fetchone()
                auto_floor = (row[0] or 0) + 1
                auto_base = row[1] or "" if row[0] and row[0] > 0 else ""
                
                insert_query = """
                    INSERT INTO cargos (name, location, container_id, base_aruco_id, floor) VALUES (%s, %s, %s, %s, %s)
                    ON CONFLICT (name) DO UPDATE SET 
                    location = EXCLUDED.location,
                    container_id = CASE WHEN EXCLUDED.container_id = '' THEN cargos.container_id ELSE EXCLUDED.container_id END,
                    base_aruco_id = EXCLUDED.base_aruco_id,
                    floor = EXCLUDED.floor
                """
                cursor.execute(insert_query, (name, location, aruco, auto_base, auto_floor))
                conn.commit()
                cursor.close()
                conn.close()

                # UI 업데이트 (엔트리 비우기 등)는 메인 스레드에서 수행해야 합니다.
                self.after(0, self._on_insert_success)
            except Exception as e:
                print(f"DB Insert 오류: {e}")

        # 메인 스레드 블로킹 방지를 위해 Insert도 백그라운드 스레드에서 실행
        threading.Thread(target=_insert_task, daemon=True).start()

    def _on_insert_success(self) -> None:
        if not getattr(self, "_ui_alive", True):
            return
        try:
            self.name_entry.delete(0, "end")
            self.loc_entry.delete(0, "end")
            self.aruco_entry.delete(0, "end")
            # 새로고침은 fetch_pending_tasks_loop가 1초마다 돌고 있으므로 자연스럽게 반영됩니다.
        except Exception:
            pass

    # ------------------------------------------------------------------
    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        open_command_popup(self)

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
