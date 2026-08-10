import json
import socket
import threading
import time
from typing import Callable, Optional


class RobotStatusListener:
    """UDP 포트로 수신되는 로봇팔(JetCobot)의 상태 메시지를 백그라운드에서 수신하고 파싱하는 리스너입니다."""
    
    def __init__(self, port: int = 15002, callback: Optional[Callable[[dict], None]] = None):
        self.port = port
        self.callback = callback
        self.sock = None
        self.running = False
        self.thread = None

    def start(self):
        if self.running:
            return
            
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.port))
        
        self.running = True
        self.thread = threading.Thread(target=self._listen, daemon=True, name=f"UDPListener-{self.port}")
        self.thread.start()

    def stop(self):
        self.running = False
        if self.sock:
            try:
                # 소켓 종료 시 recvfrom에서 발생하는 블로킹을 해제하기 위해 임시 소켓 전송
                temp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                temp_sock.sendto(b"STOP", ('127.0.0.1', self.port))
                temp_sock.close()
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        if self.thread:
            self.thread.join(timeout=1.0)
            self.thread = None

    def _listen(self):
        while self.running and self.sock:
            try:
                data, addr = self.sock.recvfrom(4096)
                if not data or data == b"STOP":
                    continue
                
                # 수신된 데이터 문자열 디코딩
                text = data.decode('utf-8', errors='ignore').strip()
                
                # JSON 객체가 연속으로 올 경우(예: '}{')를 대비해 줄바꿈 삽입
                text = text.replace('}{', '}\n{')
                
                for line in text.split('\n'):
                    line = line.strip()
                    if not line:
                        continue
                        
                    try:
                        msg = json.loads(line)
                        if self.callback:
                            self.callback(msg)
                    except json.JSONDecodeError:
                        print(f"[UDP_LISTENER] JSON 파싱 에러: {line}")
                        
            except OSError:
                if not self.running:
                    break
            except Exception as e:
                print(f"[UDP_LISTENER] 수신 중 에러 발생: {e}")
                time.sleep(0.1)
