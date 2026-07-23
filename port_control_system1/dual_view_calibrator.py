"""
dual_view_calibrator.py

CCTV 관제 카메라 화면과 AGV의 SLAM 지도(current_map.png/yaml)를 나란히 띄워놓고,
- 두 화면에서 같은 지점을 서로 클릭해서 "매칭점"을 등록하면 -> 호모그래피를 계산해서
  이후로는 CCTV에서 아무 지점이나 클릭하면 SLAM 지도 위의 대응 위치가 자동으로 표시됩니다.
- 위치를 이름 붙여 저장하면, 그 지점의
    (1) CCTV 픽셀 좌표
    (2) SLAM 지도 이미지 픽셀 좌표
    (3) 실제 map 좌표(미터, AGV 로컬라이제이션과 동일한 좌표계)
  가 전부 화면에 표시되고 location_marks_verified.json에 저장됩니다.

자연어 명령으로 이동/배차를 실행하는 기능은 command_center.py(우측 하단 "🗣️ 명령" 버튼 팝업)로
옮겼습니다. 이 화면은 순수하게 위치 마킹/캘리브레이션만 전담합니다.

내부적으로 pixel_to_map.py(PixelToMapCalibrator, 범용 2D->2D 호모그래피)와
slam_map_pixel_to_world.py(SlamMap, yaml 기반 정확한 픽셀->미터 변환)를 그대로 재사용합니다.
"""

import json
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog
from typing import Dict, List, Optional, Tuple

import customtkinter as ctk
from PIL import Image, ImageTk

from pixel_to_map import PixelToMapCalibrator
from slam_map_pixel_to_world import SlamMap

PointXY = Tuple[float, float]

# 실행 위치(cwd)와 무관하게 항상 이 스크립트가 있는 폴더의 파일을 쓰도록 고정합니다.
_APP_DIR = Path(__file__).resolve().parent
CALIBRATION_FILE = str(_APP_DIR / "cctv_to_map_image_homography.json")
LOCATIONS_FILE = str(_APP_DIR / "location_marks_verified.json")


# -----------------------------------------------------------------------------
# 클릭 가능한 이미지 패널 (CCTV / SLAM 지도 공용)
# -----------------------------------------------------------------------------

class ImagePanel(tk.Canvas):
    def __init__(self, master, on_click=None, **kwargs):
        super().__init__(master, bg="#111111", highlightthickness=0, **kwargs)
        self.on_click = on_click  # 캔버스를 클릭했을 때 호출할 외부 콜백 함수 (픽셀 좌표를 인자로 받음)
        self.original_image: Optional[Image.Image] = None       # 원본 이미지 (리사이즈 전, 항상 이 원본 기준으로 좌표 계산)
        self._tk_image: Optional[ImageTk.PhotoImage] = None      # tkinter가 실제로 그리는 이미지 객체 (참조 유지 안 하면 사라짐)
        self.markers: Dict[str, Tuple[PointXY, str]] = {}  # name -> (pixel_xy, color)  # 저장된 위치들의 확정 마커
        self.pending_marker: Optional[PointXY] = None  # 아직 이름 붙이기 전, 방금 클릭한 임시 위치
        self.highlight_marker: Optional[Tuple[PointXY, str]] = None  # (pixel_xy, color) 일시 강조
        self.route_points: List[Tuple[str, PointXY]] = []  # [(순번 라벨, pixel_xy), ...] 경유지 포함 경로 강조

        self.bind("<Button-1>", self._on_click)          # 마우스 좌클릭 이벤트 연결
        self.bind("<Configure>", lambda e: self.redraw())  # 창 크기 변경 시(=캔버스 크기 변경) 다시 그리기

    def load_image(self, path: str) -> None:
        self.original_image = Image.open(path).convert("RGB")
        self.redraw()

    def _image_rect(self):
        """캔버스 안에서 이미지가 실제로 그려지는 사각형 영역(좌상단 좌표, 너비, 높이)과
        원본 이미지 대비 축소/확대 비율(scale)을 계산합니다.
        캔버스 크기와 이미지 비율이 다를 수 있어서 가운데 정렬 + 여백 계산이 필요합니다."""
        cw = max(self.winfo_width(), 1)   # 캔버스(위젯) 현재 너비
        ch = max(self.winfo_height(), 1)  # 캔버스(위젯) 현재 높이
        if self.original_image is None:
            return 0, 0, cw, ch, 1.0
        iw, ih = self.original_image.size  # 원본 이미지 너비/높이
        scale = min(cw / iw, ch / ih)       # 가로/세로 중 더 많이 축소되는 쪽 비율을 채택 (비율 유지)
        sw, sh = int(iw * scale), int(ih * scale)  # 실제로 그려질 이미지 크기
        left = int((cw - sw) / 2)  # 가운데 정렬을 위한 좌측 여백
        top = int((ch - sh) / 2)   # 가운데 정렬을 위한 상단 여백
        return left, top, sw, sh, scale

    def widget_to_image_pixel(self, wx: int, wy: int) -> Optional[PointXY]:
        """캔버스 위 클릭 좌표(wx, wy)를 원본 이미지 기준 픽셀 좌표로 역변환합니다."""
        left, top, w, h, scale = self._image_rect()
        if w <= 0 or h <= 0 or scale <= 0:
            return None
        if not (left <= wx <= left + w and top <= wy <= top + h):
            return None  # 이미지 바깥(여백)을 클릭한 경우 무시
        return (wx - left) / scale, (wy - top) / scale

    def _on_click(self, event) -> None:
        pixel = self.widget_to_image_pixel(event.x, event.y)
        if pixel is None:
            return
        if self.on_click:
            self.on_click(pixel)  # 원본 이미지 기준 좌표로 변환해서 외부 콜백에 전달

    def set_pending(self, pixel: Optional[PointXY]) -> None:
        self.pending_marker = pixel
        self.redraw()

    def set_highlight(self, pixel: Optional[PointXY], color: str = "#FF9F43") -> None:
        self.highlight_marker = (pixel, color) if pixel is not None else None
        self.redraw()

    def set_route_points(self, points: List[Tuple[str, PointXY]]) -> None:
        """[(순번 라벨, pixel_xy), ...] - 경유지를 포함한 전체 이동 경로를 순서대로 강조 표시."""
        self.route_points = points
        self.redraw()

    def upsert_marker(self, name: str, pixel: PointXY, color: str = "#28C76F") -> None:
        self.markers[name] = (pixel, color)
        self.redraw()

    def remove_marker(self, name: str) -> None:
        self.markers.pop(name, None)
        self.redraw()

    def redraw(self) -> None:
        """캔버스를 전부 지우고, 이미지 + 저장된 마커 + 대기 중 마커 + 강조/경로 표시를
        현재 캔버스 크기에 맞춰 다시 그립니다. 크기가 바뀌거나 상태가 바뀔 때마다 호출됩니다."""
        left, top, w, h, scale = self._image_rect()
        self.delete("all")  # 캔버스에 그려진 모든 것을 지우고 처음부터 다시 그림 (단순하지만 확실한 방식)

        if self.original_image is None or w <= 0 or h <= 0:
            cw, ch = max(self.winfo_width(), 1), max(self.winfo_height(), 1)
            self.create_text(cw / 2, ch / 2, text="이미지 없음", fill="#8a8a8a", font=("Malgun Gothic", 13))
            return

        # 캔버스 크기에 맞춰 원본 이미지를 리사이즈해서 붙여넣기
        resized = self.original_image.resize((w, h), Image.BILINEAR)
        self._tk_image = ImageTk.PhotoImage(resized)  # tkinter용 이미지 객체로 변환 (self에 저장해둬야 가비지 컬렉션 안 됨)
        self.create_image(left, top, anchor="nw", image=self._tk_image)

        def to_widget(px, py):
            # 원본 이미지 픽셀 좌표 -> 지금 캔버스에 그려진 위치(화면 좌표)로 변환하는 헬퍼
            return left + px * scale, top + py * scale

        # 저장된 위치들을 초록색 마커로 표시
        for name, (pixel, color) in self.markers.items():
            cx, cy = to_widget(*pixel)
            self._draw_marker(cx, cy, name, color)

        # 이름 붙이기 전, 방금 클릭한 임시 위치를 노란색으로 표시
        if self.pending_marker is not None:
            cx, cy = to_widget(*self.pending_marker)
            self._draw_marker(cx, cy, "(대기)", "#FFD200")

        # 자연어 명령 실행 시 목적지 하나를 원으로 강조 표시
        if self.highlight_marker is not None:
            pixel, color = self.highlight_marker
            cx, cy = to_widget(*pixel)
            r = 12
            self.create_oval(cx - r, cy - r, cx + r, cy + r, outline=color, width=3)

        # 경유지를 포함한 전체 이동 경로를 번호가 붙은 점 + 점선으로 순서대로 표시
        if self.route_points:
            prev_widget_xy = None
            for label, pixel in self.route_points:
                cx, cy = to_widget(*pixel)
                if prev_widget_xy is not None:
                    self.create_line(*prev_widget_xy, cx, cy, fill="#FF9F43", width=2, dash=(5, 3))
                r = 10
                self.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#FF9F43", width=3)
                self.create_text(cx, cy, text=label, fill="#FF9F43", font=("Malgun Gothic", 10, "bold"))
                prev_widget_xy = (cx, cy)

    def _draw_marker(self, cx, cy, label, color) -> None:
        """작은 원 + 십자선 + 이름표 형태의 마커 하나를 그립니다."""
        r = 5
        self.create_oval(cx - r, cy - r, cx + r, cy + r, outline=color, width=2)
        self.create_line(cx - r - 4, cy, cx + r + 4, cy, fill=color)
        self.create_line(cx, cy - r - 4, cx, cy + r + 4, fill=color)
        self.create_text(cx + r + 6, cy, anchor="w", text=label, fill=color, font=("Malgun Gothic", 10, "bold"))


# -----------------------------------------------------------------------------
# 메인 앱
# -----------------------------------------------------------------------------

class DualViewCalibrator(ctk.CTkFrame):
    def __init__(self, master, cctv_image_path: str, slam_map_yaml_path: str, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=20, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=14, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=12)

        self.slam_map = SlamMap(slam_map_yaml_path)  # yaml의 resolution/origin으로 픽셀<->미터 변환 담당
        self.calibrator = PixelToMapCalibrator()
        if Path(CALIBRATION_FILE).exists():
            # 이전에 계산해둔 CCTV픽셀<->SLAM이미지픽셀 호모그래피가 있으면 그대로 불러옴
            self.calibrator = PixelToMapCalibrator.load(CALIBRATION_FILE)

        self.locations: Dict[str, Dict] = {}
        if Path(LOCATIONS_FILE).exists():
            self.locations = json.loads(Path(LOCATIONS_FILE).read_text(encoding="utf-8"))

        self.mode = ctk.StringVar(value="매칭점 등록")  # "매칭점 등록" 또는 "위치 등록" 중 하나
        self._pending_cctv_pixel: Optional[PointXY] = None  # 지금 막 CCTV에서 클릭한 좌표(확정 전)
        self._pending_map_pixel: Optional[PointXY] = None   # 지금 막 SLAM 지도에서 클릭한 좌표(확정 전)

        self._build_ui()
        self.cctv_panel.load_image(cctv_image_path)
        self.map_panel.load_image(self._resolve_map_image_path(slam_map_yaml_path))
        self._restore_markers()          # 기존에 저장된 위치들을 화면에 다시 표시
        self._update_calibration_status()  # 대응점 개수 등 캘리브레이션 상태 표시 갱신

    # ------------------------------------------------------------------
    def _resolve_map_image_path(self, yaml_path: str) -> str:
        """yaml이 가리키는 pgm 대신, 화면 표시용 png가 이미 있으면 그걸 우선 사용."""
        yaml_dir = Path(yaml_path).parent
        png_candidate = yaml_dir / "current_map.png"
        if png_candidate.exists():
            return str(png_candidate)

        # png가 없으면 yaml에 적힌 원본 이미지(pgm 등)를 그대로 사용
        import yaml as _yaml
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = _yaml.safe_load(f)
        return str(yaml_dir / meta["image"])

    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(2, weight=1)

        ctk.CTkLabel(self, text="🧭 CCTV ↔ SLAM 지도 좌표 검증 도구", font=self.font_title).grid(
            row=0, column=0, sticky="w", pady=(0, 10))

        # ---- 모드 선택 + 상태 ----
        control_row = ctk.CTkFrame(self, fg_color="transparent")
        control_row.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        control_row.grid_columnconfigure(2, weight=1)

        ctk.CTkLabel(control_row, text="모드:", font=self.font_body).grid(row=0, column=0, padx=(0, 8))
        ctk.CTkOptionMenu(control_row, variable=self.mode, values=["매칭점 등록", "위치 등록"],
                         command=lambda _=None: self._on_mode_change()).grid(row=0, column=1, padx=(0, 15))

        self.status_label = ctk.CTkLabel(control_row, text="", font=self.font_body, text_color="gray70")
        self.status_label.grid(row=0, column=2, sticky="w")

        # ---- 두 화면 ----
        panels_frame = ctk.CTkFrame(self, fg_color="transparent")
        panels_frame.grid(row=2, column=0, sticky="nsew", pady=(0, 10))
        panels_frame.grid_columnconfigure(0, weight=1)
        panels_frame.grid_columnconfigure(1, weight=1)
        panels_frame.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(panels_frame, text="📷 CCTV 관제 카메라", font=self.font_subtitle).grid(
            row=0, column=0, sticky="w", padx=(0, 5))
        ctk.CTkLabel(panels_frame, text="🗺️ AGV SLAM 지도", font=self.font_subtitle).grid(
            row=0, column=1, sticky="w", padx=(5, 0))

        cctv_container = ctk.CTkFrame(panels_frame, corner_radius=10)
        cctv_container.grid(row=1, column=0, sticky="nsew", padx=(0, 5))
        cctv_container.grid_columnconfigure(0, weight=1)
        cctv_container.grid_rowconfigure(0, weight=1)
        self.cctv_panel = ImagePanel(cctv_container, on_click=self._on_cctv_click)
        self.cctv_panel.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        map_container = ctk.CTkFrame(panels_frame, corner_radius=10)
        map_container.grid(row=1, column=1, sticky="nsew", padx=(5, 0))
        map_container.grid_columnconfigure(0, weight=1)
        map_container.grid_rowconfigure(0, weight=1)
        self.map_panel = ImagePanel(map_container, on_click=self._on_map_click)
        self.map_panel.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        # ---- 액션 영역 (모드별 컨트롤) ----
        action_frame = ctk.CTkFrame(self, corner_radius=10)
        action_frame.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        action_frame.grid_columnconfigure(1, weight=1)

        # 매칭점 등록용
        self.pair_count_label = ctk.CTkLabel(action_frame, text="등록된 대응점: 0개", font=self.font_body)
        self.pair_count_label.grid(row=0, column=0, sticky="w", padx=15, pady=(12, 5))
        ctk.CTkButton(action_frame, text="대응점 추가 (양쪽 클릭 후)", width=180,
                     command=self.add_pair).grid(row=0, column=1, sticky="w", padx=(0, 8), pady=(12, 5))
        ctk.CTkButton(action_frame, text="호모그래피 계산 및 저장", width=180,
                     command=self.compute_and_save_calibration).grid(row=0, column=2, sticky="w", pady=(12, 5))
        ctk.CTkButton(action_frame, text="캘리브레이션 초기화", width=150, fg_color="#EA5455", hover_color="#DC2626",
                     command=self.reset_calibration).grid(row=0, column=3, sticky="w", padx=(8, 0), pady=(12, 5))

        # 위치 등록용
        self.name_var = ctk.StringVar()
        self.name_entry = ctk.CTkEntry(action_frame, textvariable=self.name_var, placeholder_text="위치 이름 (예: 항구)")
        self.name_entry.grid(row=1, column=0, sticky="ew", padx=15, pady=(0, 12))
        ctk.CTkButton(action_frame, text="위치 저장하기", width=150,
                     command=self.save_location).grid(row=1, column=1, sticky="w", pady=(0, 12))
        ctk.CTkButton(action_frame, text="위치 삭제", width=110, fg_color="#EA5455", hover_color="#DC2626",
                     command=self.delete_location).grid(row=1, column=2, sticky="w", padx=(8, 0), pady=(0, 12))

        self.conversion_label = ctk.CTkLabel(action_frame, text="", font=self.font_body, text_color="#28C76F",
                                             wraplength=900, justify="left")
        self.conversion_label.grid(row=2, column=0, columnspan=3, sticky="w", padx=15, pady=(0, 12))

        # 우측 하단 "명령" 버튼 - 누르면 자연어 명령 팝업이 뜸
        bottom_row = ctk.CTkFrame(self, fg_color="transparent")
        bottom_row.grid(row=4, column=0, sticky="e", pady=(5, 0))
        ctk.CTkButton(bottom_row, text="🗣️ 명령", font=self.font_subtitle, fg_color="#92ccff", text_color="#003351",
                      hover_color="#cce5ff", height=40, width=100, command=self.open_command_popup).pack(side="right")

        self._on_mode_change()

    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        open_command_popup(self)

    # ------------------------------------------------------------------
    def _on_mode_change(self) -> None:
        is_pair_mode = self.mode.get() == "매칭점 등록"
        self.status_label.configure(
            text="CCTV와 SLAM 지도에서 같은 지점을 번갈아 클릭한 뒤 '대응점 추가'를 누르세요."
            if is_pair_mode else
            "CCTV 화면만 클릭하세요. SLAM 지도 위 대응 위치가 자동으로 표시됩니다."
        )
        self._pending_cctv_pixel = None
        self._pending_map_pixel = None
        self.cctv_panel.set_pending(None)
        self.map_panel.set_pending(None)

    def _update_calibration_status(self) -> None:
        self.pair_count_label.configure(text=f"등록된 대응점: {self.calibrator.num_points}개")

    # ------------------------------------------------------------------
    # 클릭 핸들러
    # ------------------------------------------------------------------
    def _on_cctv_click(self, pixel: PointXY) -> None:
        if self.mode.get() == "매칭점 등록":
            # 캘리브레이션용 대응점의 "CCTV 쪽" 좌표로 임시 저장 (SLAM 쪽 클릭과 짝지어질 때까지 대기)
            self._pending_cctv_pixel = pixel
            self.cctv_panel.set_pending(pixel)
        else:
            # 위치 등록 모드에서는 클릭 즉시 SLAM 지도 위 대응 위치를 미리보기로 계산해서 보여줌
            self._pending_cctv_pixel = pixel
            self.cctv_panel.set_pending(pixel)
            self._show_conversion_preview(pixel)

    def _on_map_click(self, pixel: PointXY) -> None:
        if self.mode.get() == "매칭점 등록":
            # 캘리브레이션용 대응점의 "SLAM 지도 쪽" 좌표로 임시 저장
            self._pending_map_pixel = pixel
            self.map_panel.set_pending(pixel)

    def _show_conversion_preview(self, cctv_pixel: PointXY) -> None:
        """CCTV에서 클릭한 좌표를 실시간으로 SLAM 이미지 픽셀 -> 실제 map 미터 좌표까지
        전부 변환해서 화면에 보여줍니다 (아직 저장 전 미리보기)."""
        if self.calibrator.num_points < 4:
            self.conversion_label.configure(
                text="⚠️ 아직 캘리브레이션(매칭점 4개 이상)이 완료되지 않아 SLAM 지도 위치를 계산할 수 없습니다.",
                text_color="#EA5455",
            )
            return

        # 1단계: CCTV 픽셀 -> SLAM 지도 이미지 픽셀 (호모그래피, 캘리브레이션 필요)
        map_image_pixel = self.calibrator.pixel_to_map(cctv_pixel)
        # 2단계: SLAM 지도 이미지 픽셀 -> 실제 map 좌표(미터) (yaml 수식, 캘리브레이션 불필요)
        map_meters = self.slam_map.pixel_to_map(*map_image_pixel)
        self.map_panel.set_pending(map_image_pixel)  # SLAM 지도 화면에도 임시 마커로 표시

        self.conversion_label.configure(
            text=(
                f"CCTV 픽셀 ({cctv_pixel[0]:.0f}, {cctv_pixel[1]:.0f})"
                f" → SLAM 지도 이미지 픽셀 ({map_image_pixel[0]:.1f}, {map_image_pixel[1]:.1f})"
                f" → 실제 map 좌표 ({map_meters[0]:.3f} m, {map_meters[1]:.3f} m)"
            ),
            text_color="#28C76F",
        )

    # ------------------------------------------------------------------
    # 매칭점 등록
    # ------------------------------------------------------------------
    def add_pair(self) -> None:
        """양쪽 화면에서 각각 클릭해둔 임시 좌표를 하나의 '대응점 쌍'으로 확정해서 등록합니다."""
        if self._pending_cctv_pixel is None or self._pending_map_pixel is None:
            messagebox.showinfo("두 지점 모두 필요", "CCTV 화면과 SLAM 지도 화면에서 각각 같은 지점을 클릭한 뒤 추가해주세요.")
            return

        self.calibrator.add_correspondence(self._pending_cctv_pixel, self._pending_map_pixel)
        self._update_calibration_status()

        # 다음 대응점을 받기 위해 임시 좌표 초기화
        self._pending_cctv_pixel = None
        self._pending_map_pixel = None
        self.cctv_panel.set_pending(None)
        self.map_panel.set_pending(None)

    def compute_and_save_calibration(self) -> None:
        """지금까지 등록한 대응점들로 호모그래피를 계산하고 파일에 저장합니다."""
        try:
            self.calibrator.compute()
            self.calibrator.save(CALIBRATION_FILE)
        except Exception as exc:
            messagebox.showwarning("캘리브레이션 오류", str(exc))
            return

        # 등록한 점들을 다시 자기 자신으로 변환해봐서 오차(정확도)를 확인 - 사용자에게 품질 피드백
        errors = self.calibrator.reprojection_error()
        max_err = max(errors) if errors else 0.0
        messagebox.showinfo(
            "캘리브레이션 완료",
            f"대응점 {self.calibrator.num_points}개로 계산 완료.\n"
            f"최대 재투영 오차: {max_err:.2f} px\n"
            f"(지도 이미지 픽셀 기준이라 큰 오차는 지도상 위치가 크게 벗어난다는 뜻이니, "
            f"오차가 크면 매칭점을 더 정확히 다시 찍어주세요.)",
        )

    # ------------------------------------------------------------------
    # 위치 저장
    # ------------------------------------------------------------------
    def save_location(self) -> None:
        if self.mode.get() != "위치 등록":
            messagebox.showinfo("모드 확인", "'위치 등록' 모드로 전환한 뒤 저장해주세요.")
            return
        if self._pending_cctv_pixel is None:
            messagebox.showinfo("위치 선택 필요", "먼저 CCTV 화면을 클릭해 위치를 지정해주세요.")
            return
        name = self.name_var.get().strip()
        if not name:
            messagebox.showinfo("이름 필요", "저장할 위치의 이름을 입력해주세요.")
            return

        cctv_pixel = self._pending_cctv_pixel
        entry = {"cctv_pixel": list(cctv_pixel)}  # CCTV 픽셀 좌표는 캘리브레이션 여부와 상관없이 항상 저장

        if self.calibrator.num_points >= 4:
            # 캘리브레이션이 되어 있으면 SLAM 이미지 픽셀 + 실제 map 미터 좌표까지 함께 계산해서 저장
            map_image_pixel = self.calibrator.pixel_to_map(cctv_pixel)
            map_meters = self.slam_map.pixel_to_map(*map_image_pixel)
            entry["map_image_pixel"] = list(map_image_pixel)
            entry["map_meters"] = list(map_meters)
            self.map_panel.upsert_marker(name, map_image_pixel, color="#28C76F")

        self.locations[name] = entry
        # 저장할 때마다 location_marks_verified.json 전체를 다시 씀 (파일이 커도 상관없을 정도로 작은 데이터)
        Path(LOCATIONS_FILE).write_text(json.dumps(self.locations, ensure_ascii=False, indent=2), encoding="utf-8")

        self.cctv_panel.upsert_marker(name, cctv_pixel, color="#28C76F")
        self._pending_cctv_pixel = None
        self.cctv_panel.set_pending(None)
        self.name_var.set("")

    def _restore_markers(self) -> None:
        """프로그램을 다시 켰을 때, 이전에 저장해둔 위치들을 두 화면에 그대로 복원합니다."""
        for name, entry in self.locations.items():
            self.cctv_panel.upsert_marker(name, tuple(entry["cctv_pixel"]), color="#28C76F")
            if "map_image_pixel" in entry:
                self.map_panel.upsert_marker(name, tuple(entry["map_image_pixel"]), color="#28C76F")

    def delete_location(self) -> None:
        """이름을 잘못 입력했거나 잘못 찍은 위치를 등록부에서 완전히 제거합니다."""
        name = simpledialog.askstring("위치 삭제", "삭제할 위치의 이름을 정확히 입력하세요:", parent=self)
        if not name:
            return
        name = name.strip()
        if name not in self.locations:
            messagebox.showwarning("찾을 수 없음", f"'{name}' 위치를 찾을 수 없습니다.")
            return

        del self.locations[name]
        Path(LOCATIONS_FILE).write_text(json.dumps(self.locations, ensure_ascii=False, indent=2), encoding="utf-8")

        self.cctv_panel.remove_marker(name)
        self.map_panel.remove_marker(name)
        messagebox.showinfo("삭제 완료", f"'{name}' 위치를 삭제했습니다.")

    def reset_calibration(self) -> None:
        """잘못 찍은 매칭점이 섞여 있어 처음부터 다시 캘리브레이션하고 싶을 때 사용합니다.
        지금까지 등록된 매칭점을 전부 지우고, 저장된 호모그래피 파일도 삭제합니다."""
        if not messagebox.askyesno(
            "캘리브레이션 초기화",
            "지금까지 등록한 모든 매칭점과 저장된 호모그래피가 삭제됩니다. 계속할까요?",
        ):
            return

        self.calibrator = PixelToMapCalibrator()  # 빈 캘리브레이터로 교체 (매칭점 0개부터 다시 시작)
        calibration_path = Path(CALIBRATION_FILE)
        if calibration_path.exists():
            calibration_path.unlink()  # 저장된 파일도 삭제해서 다음 실행 시 자동으로 다시 불러오지 않게 함

        self._update_calibration_status()
        messagebox.showinfo("초기화 완료", "캘리브레이션이 초기화되었습니다. 매칭점부터 다시 등록해주세요.")


# -----------------------------------------------------------------------------
# 단독 실행
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")

    cctv_path = sys.argv[1] if len(sys.argv) > 1 else "frame_00014.jpg"
    slam_yaml_path = sys.argv[2] if len(sys.argv) > 2 else "current_map.yaml"

    root = ctk.CTk()
    root.title("CCTV ↔ SLAM 지도 좌표 검증 도구")
    root.geometry("1400x900")

    app = DualViewCalibrator(root, cctv_image_path=cctv_path, slam_map_yaml_path=slam_yaml_path)
    app.pack(fill="both", expand=True, padx=15, pady=15)

    root.mainloop()
