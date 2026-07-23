"""
waypoint_rules_editor.py

"어떤 목적지로 갈 때 반드시 어떤 경유지를 거쳐야 하는지"를 설정하는 작은 도구입니다.
경유지 자체는 위치이므로 dual_view_calibrator.py로 미리 마킹해서 등록해두면,
여기 드롭다운에 자동으로 나타납니다.

실행:
    python waypoint_rules_editor.py
"""

import json
from pathlib import Path
from tkinter import messagebox
from typing import Dict, List

import customtkinter as ctk

from waypoint_rules import load_waypoint_rules, save_waypoint_rules

# dual_view_calibrator.py가 저장하는 최신 위치 파일 (구버전 location_marker_tool.py의
# location_marks.json은 더 이상 사용하지 않지만, 남아있으면 참고용으로 같이 읽어옵니다)
VERIFIED_LOCATIONS_FILE = "location_marks_verified.json"
LEGACY_LOCATIONS_FILE = "location_marks.json"


def load_known_location_names() -> List[str]:
    if Path(VERIFIED_LOCATIONS_FILE).exists():
        data = json.loads(Path(VERIFIED_LOCATIONS_FILE).read_text(encoding="utf-8"))
        return list(data.keys())
    if Path(LEGACY_LOCATIONS_FILE).exists():
        data = json.loads(Path(LEGACY_LOCATIONS_FILE).read_text(encoding="utf-8"))
        return list(data.keys())
    return []


class WaypointRulesEditor(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)

        self.font_title = ctk.CTkFont(family="Malgun Gothic", size=20, weight="bold")
        self.font_subtitle = ctk.CTkFont(family="Malgun Gothic", size=14, weight="bold")
        self.font_body = ctk.CTkFont(family="Malgun Gothic", size=12)

        self.known_locations = load_known_location_names()
        self.rules: Dict[str, str] = load_waypoint_rules()

        self._build_ui()
        self._refresh_rule_list()

    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(3, weight=1)

        # Header Section
        header_frame = ctk.CTkFrame(self, fg_color="transparent")
        header_frame.grid(row=0, column=0, sticky="ew", pady=(0, 20))
        
        title_frame = ctk.CTkFrame(header_frame, fg_color="transparent")
        title_frame.pack(anchor="w")
        ctk.CTkLabel(title_frame, text="🔀 필수 경유지(회차지점) 규칙 설정", font=self.font_title, text_color="#dce4ee").pack(side="left")

        ctk.CTkLabel(
            header_frame,
            text="'목적지'로 이동할 때는 항상 '필수 경유지'를 먼저 거치도록 설정합니다.\n"
                 "경유지 이름은 위치 마킹 도구로 미리 등록해두어야 아래 목록에 나타납니다.",
            font=self.font_body, text_color="#909090", justify="left"
        ).pack(anchor="w", pady=(5, 0), padx=30)

        # Form Section (bg-ctk-bg, bordered)
        form = ctk.CTkFrame(self, corner_radius=6, fg_color="#242424", border_width=1, border_color="#343638")
        form.grid(row=1, column=0, sticky="ew", pady=(0, 20))
        form.grid_columnconfigure((0, 1), weight=1)

        # Labels
        ctk.CTkLabel(form, text="목적지 (Destination)", font=self.font_body, text_color="#dce4ee").grid(row=0, column=0, sticky="w", padx=20, pady=(20, 5))
        ctk.CTkLabel(form, text="필수 경유지 (Mandatory Waypoint)", font=self.font_body, text_color="#dce4ee").grid(row=0, column=1, sticky="w", padx=20, pady=(20, 5))

        location_values = self.known_locations or ["(등록된 위치 없음)"]
        self.destination_var = ctk.StringVar(value=location_values[0])
        self.waypoint_var = ctk.StringVar(value=location_values[0])

        # Dropdowns
        ctk.CTkOptionMenu(form, variable=self.destination_var, values=location_values, 
                          fg_color="#343638", button_color="#343638", button_hover_color="#2e86c1", 
                          text_color="#dce4ee", height=36).grid(row=1, column=0, sticky="ew", padx=20, pady=(0, 20))
        ctk.CTkOptionMenu(form, variable=self.waypoint_var, values=location_values, 
                          fg_color="#343638", button_color="#343638", button_hover_color="#2e86c1", 
                          text_color="#dce4ee", height=36).grid(row=1, column=1, sticky="ew", padx=20, pady=(0, 20))

        # Add Rule Button
        ctk.CTkButton(form, text="규칙 추가 (Add Rule)", fg_color="transparent", border_width=2, 
                      border_color="#2e86c1", text_color="#2e86c1", hover_color="#2e86c1", 
                      font=self.font_subtitle, height=40, command=self.save_rule).grid(row=2, column=0, columnspan=2, sticky="ew", padx=20, pady=(0, 20))

        # Active Rules Section
        list_frame = ctk.CTkFrame(self, fg_color="transparent")
        list_frame.grid(row=2, column=0, sticky="nsew")
        list_frame.grid_columnconfigure(0, weight=1)
        list_frame.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)

        ctk.CTkLabel(list_frame, text="등록된 규칙 (Active Rules)", font=self.font_subtitle, text_color="#dce4ee").grid(row=0, column=0, sticky="w", pady=(0, 10))

        # Rule List Box
        self.rule_list_box = ctk.CTkTextbox(list_frame, font=("Consolas", 13), fg_color="#343638", text_color="#dce4ee", border_width=0, corner_radius=6, height=150)
        self.rule_list_box.grid(row=1, column=0, sticky="nsew", pady=(0, 15))
        self.rule_list_box.configure(state="disabled")

        # Delete Rule Button
        ctk.CTkButton(list_frame, text="🗑️ 선택 규칙 삭제 (Delete Selected Rule)", 
                      fg_color="#c85a5a", hover_color="#a34949", text_color="white",
                      font=self.font_subtitle, height=40, command=self.delete_rule).grid(row=2, column=0, sticky="ew")

    def open_command_popup(self) -> None:
        from command_center import open_command_popup
        open_command_popup(self)

    def save_rule(self) -> None:
        destination = self.destination_var.get().strip()  # 드롭다운에서 고른 "목적지"
        waypoint = self.waypoint_var.get().strip()         # 드롭다운에서 고른 "필수 경유지"

        # 둘 다 실제로 등록된 위치여야만 규칙으로 저장 (오타/미등록 위치 방지)
        if not destination or not waypoint or destination not in self.known_locations or waypoint not in self.known_locations:
            messagebox.showinfo("위치 필요", "먼저 위치 마킹 도구로 목적지와 경유지를 등록해주세요.")
            return
        if destination == waypoint:
            messagebox.showwarning("설정 오류", "목적지와 경유지가 같을 수 없습니다.")
            return

        self.rules[destination] = waypoint  # {목적지: 경유지} 형태로 규칙 딕셔너리에 추가/덮어쓰기
        save_waypoint_rules(self.rules)     # waypoint_rules.json에 즉시 저장
        self._refresh_rule_list()           # 화면의 규칙 목록도 갱신

    def delete_rule(self) -> None:
        from tkinter import simpledialog
        destination = simpledialog.askstring("규칙 삭제", "삭제할 규칙의 '목적지' 이름을 입력하세요:", parent=self)
        if not destination:
            return
        destination = destination.strip()
        if destination not in self.rules:
            messagebox.showwarning("찾을 수 없음", f"'{destination}'에 대한 규칙이 없습니다.")
            return
        del self.rules[destination]     # 딕셔너리에서 해당 목적지 규칙 제거
        save_waypoint_rules(self.rules)
        self._refresh_rule_list()

    def _refresh_rule_list(self) -> None:
        # CTkTextbox는 읽기 전용으로 두고 싶어서, 수정할 때만 잠깐 normal로 풀었다가 다시 잠급니다.
        self.rule_list_box.configure(state="normal")
        self.rule_list_box.delete("1.0", "end")  # "1.0"=1번째 줄 0번째 글자부터 끝까지 = 전체 삭제
        if not self.rules:
            self.rule_list_box.insert("end", "등록된 규칙이 없습니다.\n")
        for destination, waypoint in self.rules.items():
            self.rule_list_box.insert("end", f"{destination} 로 갈 때 -> 반드시 {waypoint} 경유\n")
        self.rule_list_box.configure(state="disabled")


if __name__ == "__main__":
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")

    root = ctk.CTk()
    root.title("필수 경유지 규칙 설정")
    root.geometry("700x600")

    app = WaypointRulesEditor(root)
    app.pack(fill="both", expand=True, padx=15, pady=15)

    root.mainloop()
