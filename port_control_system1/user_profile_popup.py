import customtkinter as ctk
from tkinter import messagebox

class UserProfilePopup(ctk.CTkToplevel):
    def __init__(self, master, current_user="Operator OP-084", role="최고 관리자 (Admin)", permissions="전체 시스템 제어, 긴급 정지, 설정 변경"):
        super().__init__(master)
        
        self.title("사용자 프로필")
        self.geometry("320x450")
        self.configure(fg_color="#1c1c1e")
        self.attributes("-topmost", True)
        self.resizable(False, False)
        
        # 메인 윈도우 우측 상단 근처에 띄우기
        self.update_idletasks()
        x = master.winfo_x() + master.winfo_width() - 340
        y = master.winfo_y() + 80
        self.geometry(f"+{x}+{y}")
        
        self.current_user = current_user
        
        # 상단 아바타 및 이름 영역
        header_frame = ctk.CTkFrame(self, fg_color="transparent")
        header_frame.pack(fill="x", pady=(30, 20))
        
        # 임시 아바타 (텍스트로 대체)
        avatar = ctk.CTkLabel(header_frame, text="👤", font=ctk.CTkFont(size=60))
        avatar.pack()
        
        self.name_label = ctk.CTkLabel(header_frame, text=self.current_user, 
                     font=ctk.CTkFont(family="Inter", size=20, weight="bold"), 
                     text_color="white")
        self.name_label.pack(pady=(10, 5))
                     
        badge_frame = ctk.CTkFrame(header_frame, fg_color="#28C76F", corner_radius=10, height=24)
        badge_frame.pack()
        badge_frame.pack_propagate(False)
        ctk.CTkLabel(badge_frame, text=role, 
                     font=ctk.CTkFont(family="Malgun Gothic", size=11, weight="bold"), 
                     text_color="white").pack(padx=10, pady=2)
                     
        # 권한 정보 영역
        info_frame = ctk.CTkFrame(self, fg_color="#2c2c2e", corner_radius=8)
        info_frame.pack(fill="x", padx=20, pady=10)
        
        ctk.CTkLabel(info_frame, text="보유 권한", 
                     font=ctk.CTkFont(family="Malgun Gothic", size=12, weight="bold"), 
                     text_color="#8e8e93").pack(anchor="w", padx=15, pady=(10, 2))
                     
        ctk.CTkLabel(info_frame, text=permissions, 
                     font=ctk.CTkFont(family="Malgun Gothic", size=13), 
                     text_color="#e5e5ea", justify="left", wraplength=240).pack(anchor="w", padx=15, pady=(0, 10))
                     
        # 구분선
        ctk.CTkFrame(self, height=1, fg_color="#3a3a3c").pack(fill="x", padx=20, pady=15)
        
        # 버튼 영역
        btn_font = ctk.CTkFont(family="Malgun Gothic", size=14)
        
        ctk.CTkButton(self, text="⚙️ 사용자 설정", font=btn_font, 
                      fg_color="transparent", hover_color="#2c2c2e", text_color="white", 
                      anchor="w", height=40, command=self._open_settings).pack(fill="x", padx=20, pady=2)
                      
        ctk.CTkButton(self, text="🔄 사용자 변경 (Switch User)", font=btn_font, 
                      fg_color="transparent", hover_color="#2c2c2e", text_color="white", 
                      anchor="w", height=40, command=self._switch_user).pack(fill="x", padx=20, pady=2)
                      
        ctk.CTkButton(self, text="🚪 로그아웃 (Logout)", font=btn_font, 
                      fg_color="transparent", hover_color="#3a1c1c", text_color="#ff453a", 
                      anchor="w", height=40, command=self._logout).pack(fill="x", padx=20, pady=2)
                      
    def _open_settings(self):
        uid = getattr(self.master, 'current_user_id', None)
        if not uid or not hasattr(self.master, 'USERS'):
            return
            
        pwd_dialog = ctk.CTkToplevel(self)
        pwd_dialog.title("비밀번호 확인")
        pwd_dialog.geometry("300x180")
        pwd_dialog.attributes("-topmost", True)
        pwd_dialog.resizable(False, False)
        
        self.update_idletasks()
        pwd_dialog.geometry(f"+{self.winfo_x()}+{self.winfo_y() + 50}")
        pwd_dialog.grab_set()
        
        ctk.CTkLabel(pwd_dialog, text="보안을 위해 현재 비밀번호를 입력하세요:", font=ctk.CTkFont(family="Malgun Gothic", size=13)).pack(pady=(25, 10))
        pw_entry = ctk.CTkEntry(pwd_dialog, show="*", width=200, height=35)
        pw_entry.pack(pady=5)
        
        def check_password():
            if pw_entry.get() == self.master.USERS[uid]['password']:
                pwd_dialog.destroy()
                self._show_edit_dialog()
            else:
                messagebox.showerror("오류", "비밀번호가 일치하지 않습니다.", parent=pwd_dialog)
                
        ctk.CTkButton(pwd_dialog, text="확인", command=check_password, width=200, height=35, fg_color="#2e86c1", hover_color="#3498db").pack(pady=(10, 20))
        
    def _show_edit_dialog(self):
        dialog = ctk.CTkToplevel(self)
        dialog.title("사용자 정보 수정")
        dialog.geometry("350x280")
        dialog.attributes("-topmost", True)
        dialog.resizable(False, False)
        
        # 화면 중앙
        self.update_idletasks()
        x = self.winfo_x() - 15
        y = self.winfo_y() + 50
        dialog.geometry(f"+{x}+{y}")
        dialog.grab_set()  # Modal
        
        ctk.CTkLabel(dialog, text="새로운 표시 이름:", font=ctk.CTkFont(family="Malgun Gothic", size=13, weight="bold")).pack(pady=(20, 5), padx=20, anchor="w")
        name_entry = ctk.CTkEntry(dialog, placeholder_text="이름 입력", height=35)
        name_entry.insert(0, self.name_label.cget("text"))
        name_entry.pack(fill="x", padx=20)
        
        ctk.CTkLabel(dialog, text="새로운 비밀번호 (변경 시 입력):", font=ctk.CTkFont(family="Malgun Gothic", size=13, weight="bold")).pack(pady=(15, 5), padx=20, anchor="w")
        pw_entry = ctk.CTkEntry(dialog, show="*", placeholder_text="비밀번호 입력", height=35)
        pw_entry.pack(fill="x", padx=20)
        
        def save_changes():
            new_name = name_entry.get().strip()
            new_pw = pw_entry.get().strip()
            
            uid = getattr(self.master, 'current_user_id', None)
            if not uid or not hasattr(self.master, 'USERS'):
                dialog.destroy()
                return
                
            updated = False
            if new_name and new_name != self.name_label.cget("text"):
                self.name_label.configure(text=new_name)
                if hasattr(self.master, 'user_name_label'):
                    self.master.user_name_label.configure(text=new_name)
                self.master.USERS[uid]['name'] = new_name
                updated = True
                
            if new_pw:
                self.master.USERS[uid]['password'] = new_pw
                updated = True
                
            if updated:
                messagebox.showinfo("성공", "사용자 정보가 성공적으로 변경되었습니다.", parent=dialog)
            dialog.destroy()
            
        btn_frame = ctk.CTkFrame(dialog, fg_color="transparent")
        btn_frame.pack(fill="x", pady=20, padx=20)
        
        ctk.CTkButton(btn_frame, text="저장", font=ctk.CTkFont(family="Malgun Gothic", size=14, weight="bold"),
                      command=save_changes, height=40, fg_color="#28C76F", hover_color="#20A058").pack(side="left", expand=True, padx=(0, 5))
        ctk.CTkButton(btn_frame, text="취소", font=ctk.CTkFont(family="Malgun Gothic", size=14),
                      command=dialog.destroy, height=40, fg_color="gray", hover_color="#4B5563").pack(side="right", expand=True, padx=(5, 0))
        
    def _switch_user(self):
        answer = messagebox.askyesno("사용자 변경", "현재 세션을 일시정지하고 사용자 변경 화면으로 이동하시겠습니까?", parent=self)
        if answer:
            self.destroy()
            if hasattr(self.master, 'show_login_screen'):
                self.master.show_login_screen()
            
    def _logout(self):
        answer = messagebox.askyesno("로그아웃", "정말로 로그아웃 하시겠습니까?\n진행 중인 자율주행 세션이 종료될 수 있습니다.", parent=self)
        if answer:
            self.destroy()
            if hasattr(self.master, 'show_login_screen'):
                self.master.show_login_screen()
