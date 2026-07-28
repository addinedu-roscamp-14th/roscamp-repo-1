"""
integrated_radar_view.py

항만 전체를 한눈에 보는 레이더 스타일 지도 화면입니다. 팀원이 만든
integrated_view.py의 레이아웃(좌측 캔버스 지도 + 우측 아코디언 카드 + 표시
스위치)을 가져오되, 더미 데이터 대신 우리 시스템에 실제로 등록된 위치
(location_marks_verified.json), 화물(cargo_locations.json), 차량 위치
(vehicle_positions.json)를 그대로 그려줍니다.

좌표 배치 방식: "등록된 점들끼리의 범위"가 아니라 "SLAM 지도 이미지 자체의
가로/세로 크기"를 기준으로 비율을 계산합니다. 점들만의 범위로 늘려버리면
점들이 몰려있을 때 서로 간의 거리가 실제보다 훨씬 과장되게 벌어져서,
'위치 마킹' 탭에서 보던 지도 모습과 다르게 보이는 문제가 생기기 때문입니다.
"""

import customtkinter as ctk
from PIL import Image
from typing import Dict, List

from slam_stream_processor import SlamStreamProcessor
from ros_control_bridge import RosControlBridge
from cargo_dispatch_tool import (
    load_named_locations,
    load_cargo_registry,
    load_vehicle_state,
    load_vehicle_status,
    save_vehicle_status,
    load_robot_arm_status,
    save_robot_arm_status,
    NUM_VEHICLES,
)

DEFAULT_SLAM_MAP_IMAGE = "current_map.png"  # dual_view_calibrator.py가 화면 표시용으로 쓰는 파일과 동일


def _is_robot_arm(location_name: str) -> bool:
    """이름에 "크레인"이 들어간 등록 위치는 일반 위치가 아니라 크레인 설치 지점으로 간주합니다.
    (예: "항만 크레인1") 이렇게 이름으로만 구분하기 때문에, 새 로봇팔을
    추가하고 싶으면 '위치 마킹 / 캘리브레이션' 탭에서 이름에 "크레인"을 포함해서 등록하면 됩니다."""
    return "크레인" in location_name


def _load_map_image_size(path: str = DEFAULT_SLAM_MAP_IMAGE):
    """SLAM 지도 이미지의 실제 가로/세로 픽셀 크기를 읽어옵니다. 파일이 없으면 None."""
    try:
        with Image.open(path) as img:
            return img.size  # (width, height)
    except Exception:
        return None


class IntegratedRadarView(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=18, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=16, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=13)
        self.font_mini = ctk.CTkFont(family="Malgun Gothic", size=11, weight="bold")

        self.locations = load_named_locations()
        self.cargo_registry = load_cargo_registry()
        self.vehicle_positions, _ = load_vehicle_state()
        self.vehicle_status = load_vehicle_status()
        self.map_image_size = _load_map_image_size()
        self.ros_bridge = RosControlBridge.get_instance()
        self._last_ros_status_signature = None

        self.show_locations = ctk.BooleanVar(value=True)
        self.show_vehicles = ctk.BooleanVar(value=True)
        self.show_robot_arms = ctk.BooleanVar(value=True)
        self.show_slam_detection = ctk.BooleanVar(value=True)
        self.cards = {}
        self._hit_regions = []

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        main_container = ctk.CTkFrame(self, fg_color="transparent")
        main_container.grid(row=0, column=0, sticky="nsew", padx=24, pady=24)
        main_container.grid_columnconfigure(0, weight=3)
        main_container.grid_columnconfigure(1, weight=0, minsize=320)
        main_container.grid_rowconfigure(0, weight=1)

        # 좌측: 지도 캔버스
        self.map_frame = ctk.CTkFrame(main_container, fg_color="#2b2b2b", corner_radius=10, border_width=1, border_color="#3d3d3d")
        self.map_frame.grid(row=0, column=0, padx=(0, 24), sticky="nsew")
        self.map_frame.grid_rowconfigure(1, weight=1)
        self.map_frame.grid_columnconfigure(0, weight=1)

        header_panel = ctk.CTkFrame(self.map_frame, fg_color="#242526", corner_radius=0, height=50)
        header_panel.grid(row=0, column=0, sticky="ew")
        header_panel.pack_propagate(False)
        ctk.CTkLabel(header_panel, text="Integrated Port Radar View", font=self.font_title, text_color="#e5e2e3").pack(pady=10)
        ctk.CTkFrame(self.map_frame, fg_color="#404850", height=1).grid(row=0, column=0, sticky="sew")

        self.canvas = ctk.CTkCanvas(self.map_frame, bg="#0b1320", highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        bottom_bar = ctk.CTkFrame(self.map_frame, fg_color="#1b1b1c", corner_radius=0, height=40)
        bottom_bar.grid(row=2, column=0, sticky="ew")
        bottom_bar.pack_propagate(False)
        ctk.CTkFrame(self.map_frame, fg_color="#404850", height=1).grid(row=2, column=0, sticky="new")
        
        self.selection_label = ctk.CTkLabel(
            bottom_bar, text="Click on vehicle or robot arm icons to display operational status here.",
            font=self.font_body, text_color="#c0c7d1", anchor="w",
        )
        self.selection_label.pack(side="left", padx=15, pady=8)

        # 우측: 사이드바
        self.right_panel = ctk.CTkFrame(main_container, fg_color="transparent", width=320)
        self.right_panel.grid(row=0, column=1, sticky="nsew")
        self.right_panel.grid_rowconfigure(1, weight=1)
        self._build_right_panel()

        self.after(200, self._render_loop)
        self.after(1000, self._refresh_ros_status)

    # ------------------------------------------------------------------
    def _build_right_panel(self) -> None:
        controls_panel = ctk.CTkFrame(self.right_panel, fg_color="#2b2b2b", corner_radius=10, border_width=1, border_color="#3d3d3d")
        controls_panel.grid(row=0, column=0, sticky="ew", pady=(0, 24))

        ctk.CTkLabel(controls_panel, text="🌐 Smart Port Control", font=self.font_title, text_color="#92ccff").pack(anchor="w", padx=16, pady=(16, 10))

        switches = ctk.CTkFrame(controls_panel, fg_color="transparent")
        switches.pack(fill="x", padx=16, pady=5)
        
        ctk.CTkSwitch(switches, text="Show Position", variable=self.show_locations, font=self.font_body, text_color="#e5e2e3", progress_color="#92ccff").pack(anchor="w", pady=5)
        ctk.CTkSwitch(switches, text="Show Vehicles", variable=self.show_vehicles, font=self.font_body, text_color="#e5e2e3", progress_color="#92ccff").pack(anchor="w", pady=5)
        ctk.CTkSwitch(switches, text="Show Robots", variable=self.show_robot_arms, font=self.font_body, text_color="#e5e2e3", progress_color="#92ccff").pack(anchor="w", pady=5)

        self.slam_status_label = ctk.CTkLabel(
            controls_panel, text="📡 SLAM: Disconnected", font=self.font_mini,
            text_color="#c0c7d1")
        self.slam_status_label.pack(anchor="w", padx=16, pady=(10, 10))

        ctk.CTkButton(controls_panel, text="🔄 Refresh", command=self.refresh_from_disk,
                      fg_color="#1f6aa5", hover_color="#165181", text_color="white", 
                      height=36, font=self.font_body).pack(fill="x", padx=16, pady=(0, 16))

        status_panel = ctk.CTkFrame(self.right_panel, fg_color="#2b2b2b", corner_radius=10, border_width=1, border_color="#3d3d3d")
        status_panel.grid(row=1, column=0, sticky="nsew", pady=(0, 24))
        status_panel.grid_rowconfigure(1, weight=1)
        
        self.status_tab_var = ctk.StringVar(value="화물")
        self.status_tab_button = ctk.CTkSegmentedButton(
            status_panel, values=["화물", "차량", "로봇팔"],
            variable=self.status_tab_var, command=self._on_status_tab_change,
            selected_color="#1f6aa5", selected_hover_color="#165181", unselected_color="#3d3d3d", unselected_hover_color="#404850"
        )
        self.status_tab_button.grid(row=0, column=0, sticky="ew", padx=16, pady=16)

        self.cards_container = ctk.CTkScrollableFrame(status_panel, fg_color="transparent")
        self.cards_container.grid(row=1, column=0, sticky="nsew", padx=4, pady=(0, 16))
        self._on_status_tab_change("화물")

        btn_row = ctk.CTkFrame(self.right_panel, fg_color="transparent")
        btn_row.grid(row=2, column=0, sticky="e")
                      
        ctk.CTkButton(btn_row, text="🗣️ 명령", font=self.font_subtitle, fg_color="#92ccff", text_color="#003351",
                      hover_color="#cce5ff", height=40, width=100, command=self.open_command_popup).pack(side="left")

    def _on_status_tab_change(self, tab_name: str) -> None:
        if tab_name == "화물":
            self._rebuild_cargo_cards()
        elif tab_name == "차량":
            self._rebuild_vehicle_cards()
        elif tab_name == "로봇팔":
            self._rebuild_robot_arm_cards()

    def _clear_cards_container(self) -> None:
        for widget in self.cards_container.winfo_children():
            widget.destroy()
        self.cards = {}

    # ------------------------------------------------------------------
    # 탭 1: 위치별 화물 현황 (창고/항구 등 실제 화물을 보관하는 위치만)
    # ------------------------------------------------------------------
    def _rebuild_cargo_cards(self) -> None:
        self._clear_cards_container()
        ctk.CTkLabel(self.cards_container, text="📊 Cargo Status by Location", font=self.font_title, text_color="#e5e2e3").pack(anchor="w", padx=10, pady=(0, 10))

        storage_locations = set()
        for name in self.locations.keys():
            if not ("회차" in name or "대기" in name or _is_robot_arm(name)):
                storage_locations.add(name)
        for loc in self.cargo_registry.values():
            if loc:
                storage_locations.add(loc)

        for name in sorted(list(storage_locations)):
            entry = self.locations.get(name, {})
            items_here = [cargo for cargo, loc in self.cargo_registry.items() if loc == name]
            
            if not entry and not items_here:
                continue

            missing_coord = "map_image_pixel" not in entry
            title = f"📍 {name} ({len(items_here)}건)"
            if missing_coord:
                title += "  ⚠️ 좌표 미등록"

            container = ctk.CTkFrame(self.cards_container, fg_color="#2a2a2b", border_width=1, border_color="#404850", corner_radius=8)
            container.pack(fill="x", padx=10, pady=5)

            header = ctk.CTkFrame(container, fg_color="transparent")
            header.pack(fill="x", padx=10, pady=10)
            ctk.CTkLabel(header, text=title, font=self.font_body,
                        text_color="#F59E0B" if missing_coord else "#e5e2e3").pack(side="left")
            btn = ctk.CTkButton(header, text="View More ▼", width=80, height=24, fg_color="#3d3d3d", hover_color="#404850", text_color="#e5e2e3", font=self.font_mini,
                                command=lambda n=name: self.toggle_details(n))
            btn.pack(side="right")

            detail = ctk.CTkFrame(container, fg_color="#1f1f20", corner_radius=5)
            detail_text = "\n".join(f"• {cargo}" for cargo in items_here) or "(비어 있음)"
            if missing_coord:
                detail_text += (
                    "\n\n⚠️ 이 위치는 캘리브레이션 좌표가 없어 레이더뷰 지도에 표시되지 않습니다.\n"
                    "'위치 마킹 / 캘리브레이션' 탭에서 매칭점을 등록하고 호모그래피를 계산한 뒤,\n"
                    "이 위치를 다시 저장해주세요."
                )
            ctk.CTkLabel(detail, text=detail_text, justify="left", font=self.font_body, text_color="gray80").pack(
                anchor="w", padx=15, pady=10)

            self.cards[name] = {"container": container, "detail": detail, "btn": btn, "hidden": True}

    # ------------------------------------------------------------------
    # 탭 2: 차량 현황 (위치, 배터리, 이상 유무, 최근 작업)
    # ------------------------------------------------------------------
    def _rebuild_vehicle_cards(self) -> None:
        self._clear_cards_container()
        ctk.CTkLabel(self.cards_container, text="🚗 차량 현황", font=self.font_title, text_color="#e5e2e3").pack(anchor="w", padx=10, pady=(0, 10))

        for i in range(len(self.vehicle_positions)):
            key = f"차량 {i + 1}"
            location = self.vehicle_positions[i]
            status = dict(self.vehicle_status.get(key, {}))
            subtitle = f"현재 위치: {location}"
            if i == 0:
                snapshot = self.ros_bridge.snapshot()
                if snapshot.battery_percent is not None:
                    status["battery_pct"] = round(snapshot.battery_percent, 1)
                if snapshot.odom_xy is not None:
                    x, y = snapshot.odom_xy
                    yaw = snapshot.odom_yaw_deg or 0.0
                    subtitle += (
                        f"\nROS odom: x={x:.3f}, y={y:.3f}, yaw={yaw:.1f}°"
                    )
            self._build_status_card(
                key=key, subtitle=subtitle, status=status,
                on_status_changed=self._save_vehicle_status_change,
            )

    def _refresh_ros_status(self) -> None:
        if not self.winfo_exists():
            return
        snapshot = self.ros_bridge.snapshot()
        signature = (
            snapshot.ready,
            snapshot.battery_percent,
            snapshot.odom_xy,
            snapshot.odom_yaw_deg,
        )
        if (
            signature != self._last_ros_status_signature
            and self.status_tab_var.get() == "차량"
        ):
            self._last_ros_status_signature = signature
            self._rebuild_vehicle_cards()
        self.after(1000, self._refresh_ros_status)

    # ------------------------------------------------------------------
    # 탭 3: 로봇팔 현황 (배터리/전원, 이상 유무, 최근 작업)
    # ------------------------------------------------------------------
    def _rebuild_robot_arm_cards(self) -> None:
        self._clear_cards_container()
        ctk.CTkLabel(self.cards_container, text="🦾 로봇팔 현황", font=self.font_title, text_color="#e5e2e3").pack(anchor="w", padx=10, pady=(0, 10))

        robot_arm_names = [name for name in self.locations if _is_robot_arm(name)]
        if not robot_arm_names:
            ctk.CTkLabel(self.cards_container, text="등록된 로봇팔이 없습니다.\n'위치 마킹' 탭에서 이름에 \"로봇팔\"을 포함해 등록해주세요.",
                        font=self.font_body, text_color="gray60", justify="left").pack(anchor="w", pady=10)
            return

        robot_arm_status = load_robot_arm_status(robot_arm_names)
        for name in robot_arm_names:
            status = robot_arm_status.get(name, {})
            self._build_status_card(
                key=name, subtitle=None, status=status,
                on_status_changed=lambda k, s: self._save_robot_arm_status_change(robot_arm_names, k, s),
            )

    def _build_status_card(self, key: str, subtitle, status: Dict, on_status_changed) -> None:
        """차량/로봇팔 공용 상태 카드: 배터리, 이상 유무(토글 버튼), 최근 작업 이력을 보여줍니다."""
        fault = status.get("fault")
        battery = status.get("battery_pct", 100)
        last_job = status.get("last_job")
        last_job_time = status.get("last_job_time")

        container = ctk.CTkFrame(self.cards_container, fg_color="#2a2a2b", border_width=1, border_color="#404850", corner_radius=8)
        container.pack(fill="x", padx=10, pady=5)

        header = ctk.CTkFrame(container, fg_color="transparent")
        header.pack(fill="x", padx=12, pady=(10, 5))
        ctk.CTkLabel(header, text=key, font=self.font_body, text_color="#e5e2e3").pack(side="left")
        status_text = "🔴 이상 있음" if fault else "🟢 정상"
        ctk.CTkLabel(header, text=status_text, font=self.font_body,
                    text_color="#ffb4ab" if fault else "#61de8a").pack(side="right")

        if subtitle:
            ctk.CTkLabel(container, text=subtitle, font=self.font_body, text_color="gray70",
                        anchor="w").pack(fill="x", padx=12)

        battery_row = ctk.CTkFrame(container, fg_color="transparent")
        battery_row.pack(fill="x", padx=12, pady=(5, 0))
        ctk.CTkLabel(battery_row, text=f"🔋 배터리/전원: {battery}%", font=self.font_body).pack(side="left")

        job_text = f"마지막 작업: {last_job} ({last_job_time})" if last_job else "마지막 작업: 기록 없음"
        ctk.CTkLabel(container, text=job_text, font=self.font_body, text_color="gray70",
                    anchor="w", wraplength=280, justify="left").pack(fill="x", padx=12, pady=(5, 0))

        if fault:
            ctk.CTkLabel(container, text=f"⚠️ {fault}", font=self.font_body, text_color="#EA5455",
                        anchor="w", wraplength=280, justify="left").pack(fill="x", padx=12, pady=(5, 0))

        action_row = ctk.CTkFrame(container, fg_color="transparent")
        action_row.pack(fill="x", padx=12, pady=(8, 10))
        if fault:
            ctk.CTkButton(action_row, text="정상으로 표시", fg_color="#28C76F", hover_color="#20A360", height=26, text_color="white", font=self.font_mini,
                         command=lambda: on_status_changed(key, {**status, "fault": None})).pack(side="left")
        else:
            ctk.CTkButton(action_row, text="고장 신고", fg_color="#ffb4ab", hover_color="#ff897d", height=26, text_color="#690005", font=self.font_mini,
                         command=lambda: on_status_changed(key, {**status, "fault": "사용자가 이상을 신고했습니다."})).pack(side="left")

    def _save_vehicle_status_change(self, key: str, new_status: Dict) -> None:
        self.vehicle_status[key] = new_status
        save_vehicle_status(self.vehicle_status)
        self._rebuild_vehicle_cards()

    def _save_robot_arm_status_change(self, robot_arm_names: List[str], key: str, new_status: Dict) -> None:
        all_status = load_robot_arm_status(robot_arm_names)
        all_status[key] = new_status
        save_robot_arm_status(all_status)
        self._rebuild_robot_arm_cards()

    def toggle_details(self, name: str) -> None:
        card = self.cards.get(name)
        if not card:
            return
        if card["hidden"]:
            card["detail"].pack(fill="x", padx=10, pady=(0, 10))
            card["btn"].configure(text="Hide ▲")
            card["hidden"] = False
        else:
            card["detail"].pack_forget()
            card["btn"].configure(text="View More ▼")
            card["hidden"] = True

    # ------------------------------------------------------------------
    def refresh_from_disk(self) -> None:
        """위치/화물/차량 데이터를 디스크에서 다시 읽어와 지도와 현재 보고 있는 탭을 갱신합니다."""
        self.locations = load_named_locations()
        self.cargo_registry = load_cargo_registry()
        self.vehicle_positions, _ = load_vehicle_state()
        self.vehicle_status = load_vehicle_status()
        self._on_status_tab_change(self.status_tab_var.get())

    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        open_command_popup(self, on_cargo_updated=self.refresh_from_disk)

    # ------------------------------------------------------------------
    # 캔버스 클릭 - 차량/로봇팔 아이콘을 누르면 작업 현황 표시
    # ------------------------------------------------------------------
    def _on_canvas_click(self, event) -> None:
        # 나중에 그려진(위에 보이는) 아이콘이 우선 선택되도록 역순으로 검사
        for x1, y1, x2, y2, info_text in reversed(self._hit_regions):
            if x1 <= event.x <= x2 and y1 <= event.y <= y2:
                self.selection_label.configure(text=info_text, text_color="white")
                return
        self.selection_label.configure(
            text="차량이나 로봇팔 아이콘을 클릭하면 작업 현황이 여기에 표시됩니다.", text_color="gray60")

    # ------------------------------------------------------------------
    # 지도 렌더링
    # ------------------------------------------------------------------
    def _render_loop(self) -> None:
        try:
            self.canvas.delete("all")
            w = self.canvas.winfo_width()
            h = self.canvas.winfo_height()
            if w < 10 or h < 10:
                w, h = 800, 600
            self._draw_map(w, h)
        except Exception as exc:
            print(f"[레이더뷰 렌더링 오류] {exc}")
        self.after(300, self._render_loop)  # 화물 이동 등 변화를 어느 정도 주기적으로 반영

    def _draw_map(self, w: int, h: int) -> None:
        self._hit_regions = []  # 이번 렌더링에서 새로 그릴 클릭 영역들로 다시 채움

        self.canvas.create_rectangle(50, 50, w - 50, h - 50, outline="#334155", width=2)
        self.canvas.create_text(w / 2, 30, text="항만 관제 레이더 뷰", fill="gray80", font=self.font_title)

        coords = [entry["map_image_pixel"] for entry in self.locations.values() if "map_image_pixel" in entry]
        if not coords:
            self.canvas.create_text(
                w / 2, h / 2, text="캘리브레이션된 위치가 없습니다.\n'위치 마킹 / 캘리브레이션' 탭에서 먼저 등록해주세요.",
                fill="gray50", font=self.font_body,
            )
            return

        margin = 70

        # 핵심 수정: "등록된 점들끼리의 범위"가 아니라 "SLAM 지도 이미지 자체의 크기"를
        # 기준으로 비율을 계산합니다. 점들만의 좁은 범위를 캔버스 전체에 늘려버리면
        # 점들 간 거리가 실제보다 훨씬 과장되게 벌어져서 위치 마킹 탭에서 보던 지도
        # 모습과 다르게 보였습니다 (예: 창고가 유독 아래쪽에 있는 것처럼 보이는 문제).
        if self.map_image_size:
            min_x, min_y = 0, 0
            max_x, max_y = self.map_image_size
        else:
            # 지도 이미지 파일을 못 찾으면 예전처럼 점들의 범위로라도 대체
            xs = [c[0] for c in coords]
            ys = [c[1] for c in coords]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)

        # 혹시 어떤 점이 이미지 범위 밖(호모그래피 추정 오차 등으로 살짝 벗어난 경우)에
        # 있어도 캔버스 밖으로 잘려서 안 보이는 일이 없도록 범위를 넉넉히 확장합니다.
        xs_all = [c[0] for c in coords]
        ys_all = [c[1] for c in coords]
        min_x, max_x = min(min_x, min(xs_all)), max(max_x, max(xs_all))
        min_y, max_y = min(min_y, min(ys_all)), max(max_y, max(ys_all))

        span_x = max(max_x - min_x, 0.1)
        span_y = max(max_y - min_y, 0.1)

        def to_canvas(px: float, py: float):
            # 이미지 픽셀은 이미 "아래로 갈수록 y가 커지는" 화면 좌표계와 방향이 같으므로
            # 별도로 y축을 뒤집지 않고 그대로 비례 배치합니다.
            cx = margin + (px - min_x) / span_x * (w - 2 * margin)
            cy = margin + (py - min_y) / span_y * (h - 2 * margin)
            return cx, cy

        if self.show_locations.get():
            for name, entry in self.locations.items():
                if "map_image_pixel" not in entry or _is_robot_arm(name):
                    continue
                cx, cy = to_canvas(*entry["map_image_pixel"])
                cargo_count = sum(1 for loc in self.cargo_registry.values() if loc == name)
                self.canvas.create_rectangle(cx - 14, cy - 14, cx + 14, cy + 14, fill="#1f3b5c", outline="#92ccff")
                self.canvas.create_text(cx, cy + 24, text=f"{name} ({cargo_count})", fill="#e5e2e3", font=self.font_mini)

        if self.show_robot_arms.get():
            for name, entry in self.locations.items():
                if "map_image_pixel" not in entry or not _is_robot_arm(name):
                    continue
                cx, cy = to_canvas(*entry["map_image_pixel"])
                
                self.canvas.create_polygon(cx, cy - 16, cx - 14, cy + 10, cx + 14, cy + 10,
                                          fill="#ee671c", outline="#ee671c")
                self.canvas.create_text(cx, cy + 24, text=f"🦾 {name}", fill="#ee671c", font=self.font_mini)

                cargo_count = sum(1 for loc in self.cargo_registry.values() if loc == name)
                info_text = f"🦾 {name} — 등록된 로봇팔 위치 (근처 화물 {cargo_count}건)"
                self._hit_regions.append((cx - 16, cy - 18, cx + 16, cy + 12, info_text))

        if self.show_vehicles.get():
            n = max(len(self.vehicle_positions), 1)
            for i, veh_loc in enumerate(self.vehicle_positions):
                entry = self.locations.get(veh_loc)
                if not entry or "map_image_pixel" not in entry:
                    continue
                cx, cy = to_canvas(*entry["map_image_pixel"])
                offset = (i - (n - 1) / 2) * 20  
                self.canvas.create_oval(cx - 9 + offset, cy - 9, cx + 9 + offset, cy + 9, fill="#1f422e", outline="#61de8a")
                self.canvas.create_text(cx + offset, cy - 20, text=f"차량{i + 1}", fill="#61de8a", font=self.font_mini)

                info_text = f"🚗 차량 {i + 1} — 현재 위치: '{veh_loc}'"
                self._hit_regions.append((cx - 9 + offset, cy - 9, cx + 9 + offset, cy + 9, info_text))

        # 등록된 위치인데 좌표가 없어서 지도에 못 그린 것들을 안내 (특히 방금 등록한
        # 로봇팔 위치가 여기 걸리면, 캘리브레이션 계산 전에 저장된 것일 가능성이 큽니다)
        missing = [name for name, entry in self.locations.items() if "map_image_pixel" not in entry]
        if missing:
            self.canvas.create_text(
                w / 2, h - 20,
                text=f"⚠️ 좌표 미등록 위치 {len(missing)}건 (지도에 표시 안 됨): {', '.join(missing)}",
                fill="#F59E0B", font=self.font_mini,
            )

        # ------------------------------------------------------------------
        # SLAM 연결 상태 표시 갱신
        # ------------------------------------------------------------------
        slam_processor = SlamStreamProcessor.get_instance()
        is_slam_running = slam_processor.is_running

        try:
            if is_slam_running:
                self.slam_status_label.configure(
                    text="📡 SLAM: 연결됨",
                    text_color="#28C76F")
            else:
                self.slam_status_label.configure(
                    text="📡 SLAM: 연결 안 됨",
                    text_color="gray50")
        except Exception:
            pass
