#!/usr/bin/env python3

# OpenCV는 카메라 영상 처리와 직사각형 검출에 사용합니다.
import cv2

# numpy는 직사각형 좌표 계산에 사용합니다.
import numpy as np

# json은 설정 파일과 서버 통신에 사용합니다.
import json

# time은 대기와 키 유지 시간 계산에 사용합니다.
import time

# socket은 robot_arm_server.py와 TCP 통신하기 위해 사용합니다.
import socket

# threading은 로봇 명령과 카메라 화면 갱신을 분리하기 위해 사용합니다.
import threading

# queue는 관절 키 명령을 영상 루프와 분리하는 데 사용합니다.
import queue

# subprocess는 v4l2-ctl로 USB 카메라 이름을 찾는 데 사용합니다.
import subprocess

# Path는 설정 파일 경로 처리에 사용합니다.
from pathlib import Path

# ROS 2 Python 기능을 사용합니다.
import rclpy

# ROS 2 노드를 생성합니다.
from rclpy.node import Node

# 카메라 영상용 QoS 설정입니다.
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

# ROS 카메라 영상 메시지입니다.
from sensor_msgs.msg import Image

# ROS 영상을 OpenCV 영상으로 변환합니다.
from cv_bridge import CvBridge


# 설정 파일 경로입니다.
CONFIG_PATH = Path("/home/rsj/YOLO/src/jetcobot/jetcobot_bringup/config/arm_rect_auto_pick_config.json")


# 로봇팔 서버 주소입니다.
ARM_SERVER_HOST = "127.0.0.1"


# 로봇팔 서버 포트입니다.
ARM_SERVER_PORT = 15000


# 기본 설정값입니다.
DEFAULT_CONFIG = {
    "video_device": "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0",

    "object_width_mm": 33.0,
    "object_length_mm": 78.0,
    "object_height_mm": 35.0,
    "camera_fx_rect_px": 973.32886,
    "camera_fy_rect_px": 989.13171,
    "use_camera_depth": True,
    "depth_reference_mm": None,
    "depth_gain": 1.0,
    "depth_filter_alpha": 0.35,
    "depth_min_mm": 80.0,
    "depth_max_mm": 700.0,
    "depth_max_correction_mm": 50.0,
    "depth_max_age_sec": 3.0,

    "startup_move_enabled": True,

    "camera_ready_angles": [0.0, 45.0, -85.0, -25.0, 0.0, -45.0],

    "speed": 60,
    "angle_speed": 35,
    "jog_mm": 0.8,

    "center_tolerance_px": 20,
    "max_step_mm": 4.0,

    "x_from_image_y": -0.12,
    "y_from_image_x": -0.12,

    "approach_z_offset_mm": 75.0,
    "pick_down_mm": 38.0,
    "lift_up_mm": 80.0,

    "gripper_open": 100,
    "gripper_close": 20,
    "gripper_speed": 30,

    "min_area_px": 700,
    "max_area_px": 120000,
    "aspect_tolerance": 0.8,

    "target_u": None,
    "target_v": None,
    "direct_pick_target_u": None,
    "direct_pick_target_v": None,

    "scan_coords": None,
    "place_coords": None,

    "pick_manual_calibration_release_servos": True,
    "pick_manual_calibration_return_ready_after_save": True,
    "pick_manual_calibration_open_gripper": True,
    "direct_pick_require_fresh_detection": True,
    "direct_pick_use_relative_manual_calibration": True,
    "direct_pick_open_gripper_before_move": True,
    "manual_gripper_wait_sec": 0.6,
    "direct_pick_manual_calibration_model": "visual_delta",
    "zero_focus_center_max_iter": 4,
    "center_direction_sign": 1.0,
    "zero_focus_center_direction_sign": 1.0,
    "zero_focus_use_simple_centering": True,
    "zero_focus_simple_x_from_image_y": 0.025,
    "zero_focus_simple_y_from_image_x": 0.025,
    "zero_focus_auto_direction_tune": True,
    "zero_focus_probe_scale": 0.35,
    "zero_focus_probe_max_mm": 2.5,
    "zero_focus_use_live_jacobian": True,
    "zero_focus_live_probe_mm": 8.0,
    "zero_focus_live_gain": 0.95,
    "zero_focus_live_max_step_mm": 28.0,
    "zero_focus_probe_wait_sec": 0.35,
    "zero_focus_probe_settle_sec": 0.12,
    "zero_focus_accept_px": 35.0,
    "zero_focus_save_max_error_px": 35.0,
    "zero_focus_jacobian_cache_sec": 120.0,
    "zero_focus_allow_fallback_centering": False
}


# 설정 파일을 불러오는 함수입니다.
def load_config():

    # config 폴더를 생성합니다.
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)

    # 설정 파일이 없으면 기본값으로 생성합니다.
    if not CONFIG_PATH.exists():
        CONFIG_PATH.write_text(json.dumps(DEFAULT_CONFIG, indent=4, ensure_ascii=False))

    # 설정 파일을 읽습니다.
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)

    # 누락된 설정값은 기본값으로 채웁니다.
    for key, value in DEFAULT_CONFIG.items():
        cfg.setdefault(key, value)

    # 설정값을 반환합니다.
    return cfg


# 설정 파일을 저장하는 함수입니다.
def save_config(cfg):

    # 설정값을 JSON 파일로 저장합니다.
    CONFIG_PATH.write_text(json.dumps(cfg, indent=4, ensure_ascii=False))

    # 저장 로그를 출력합니다.
    print(f"[SAVE] {CONFIG_PATH}")



# 지정한 카메라 장치가 실제로 열리고 프레임이 읽히는지 확인하는 함수입니다.
def camera_can_read(dev):

    # OpenCV로 카메라를 엽니다.
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)

    # 열림 여부를 확인합니다.
    opened = cap.isOpened()

    # 기본 읽기 결과입니다.
    ok = False

    # 카메라가 열렸으면 프레임을 읽습니다.
    if opened:

        # 첫 프레임은 버립니다.
        cap.read()

        # 실제 확인용 프레임을 읽습니다.
        ret, frame = cap.read()

        # 프레임이 정상인지 확인합니다.
        ok = bool(ret and frame is not None)

    # 카메라를 해제합니다.
    cap.release()

    # 결과를 반환합니다.
    return ok


# /dev/video 번호가 바뀌어도 USB 로봇팔 카메라를 자동으로 찾는 함수입니다.
def find_camera_device(cfg):

    # 설정 파일의 video_device 값을 읽습니다.
    requested = str(cfg.get("video_device", "auto"))

    # 사용자가 /dev/video번호를 직접 넣었고 정상이라면 그대로 사용합니다.
    if requested != "auto" and Path(requested).exists() and camera_can_read(requested):
        return requested

    # v4l2-ctl로 USB 2.0 Camera를 우선 찾습니다.
    try:

        # 연결된 카메라 목록을 가져옵니다.
        out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True)

        # 카메라 이름과 장치 목록을 저장합니다.
        devices_by_name = []

        # 현재 카메라 이름입니다.
        current_name = None

        # 현재 카메라 장치 목록입니다.
        current_devs = []

        # 출력 줄을 하나씩 처리합니다.
        for raw_line in out.splitlines():

            # 오른쪽 공백을 제거합니다.
            line = raw_line.rstrip()

            # 빈 줄이면 현재 블록을 저장합니다.
            if not line:
                if current_name is not None:
                    devices_by_name.append((current_name, current_devs))
                current_name = None
                current_devs = []
                continue

            # /dev/video 줄이면 장치 목록에 넣습니다.
            if line.strip().startswith("/dev/video"):
                current_devs.append(line.strip())

            # 그 외에는 카메라 이름으로 처리합니다.
            elif not raw_line.startswith(" ") and not raw_line.startswith("\t"):
                if current_name is not None:
                    devices_by_name.append((current_name, current_devs))
                current_name = line
                current_devs = []

        # 마지막 블록 저장입니다.
        if current_name is not None:
            devices_by_name.append((current_name, current_devs))

        # 로봇팔 USB 카메라 이름을 우선합니다.
        preferred_names = ["USB 2.0 Camera", "USB Camera"]

        # 선호 이름 순서대로 확인합니다.
        for preferred in preferred_names:

            # 카메라 블록을 확인합니다.
            for name, devs in devices_by_name:

                # 이름이 맞는 경우만 사용합니다.
                if preferred in name:

                    # 해당 카메라의 /dev/video 후보를 확인합니다.
                    for dev in devs:

                        # 실제 프레임이 읽히면 반환합니다.
                        if camera_can_read(dev):
                            return dev

    except Exception as e:

        # v4l2-ctl 실패 시 fallback으로 넘어갑니다.
        print("[WARN] v4l2-ctl camera search failed:", e)

    # fallback: /dev/video0~9 중 실제로 읽히는 장치를 찾습니다.
    readable = []

    # 후보 장치를 확인합니다.
    for i in range(10):

        # 후보 장치 경로입니다.
        dev = f"/dev/video{i}"

        # 존재하고 읽히면 후보에 넣습니다.
        if Path(dev).exists() and camera_can_read(dev):
            readable.append(dev)

    # 내장 웹캠 /dev/video0보다 외장 카메라를 우선합니다.
    for dev in readable:
        if dev != "/dev/video0":
            return dev

    # 그래도 없으면 첫 번째 읽히는 카메라를 사용합니다.
    if readable:
        return readable[0]

    # 읽을 수 있는 카메라가 없으면 오류입니다.
    raise RuntimeError("읽을 수 있는 카메라를 찾지 못했습니다.")


# robot_arm_server.py와 통신하는 클라이언트 클래스입니다.
class ArmClient:

    # 서버에 JSON 명령을 보내는 함수입니다.
    def request(self, payload):

        try:

            # TCP 서버에 연결합니다.
            with socket.create_connection(
                (ARM_SERVER_HOST, ARM_SERVER_PORT),
                timeout=3.0
            ) as sock:

                # 로봇팔 이동과 그리퍼 동작 응답은 최대 15초 기다립니다.
                sock.settimeout(15.0)

                # JSON 한 줄을 서버로 보냅니다.
                sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))

                # 응답 버퍼입니다.
                data = b""

                # 한 줄 응답을 받을 때까지 반복합니다.
                while not data.endswith(b"\n"):

                    # 응답 일부를 받습니다.
                    chunk = sock.recv(4096)

                    # 응답이 없으면 중단합니다.
                    if not chunk:
                        break

                    # 버퍼에 추가합니다.
                    data += chunk

            # 응답이 비어 있으면 실패 처리합니다.
            if not data:
                return {"ok": False, "error": "empty response from robot_arm_server"}

            # JSON 응답을 파싱합니다.
            response = json.loads(data.decode("utf-8").strip())

        except Exception as e:

            # 서버 연결 실패 시 프로그램이 죽지 않게 실패 응답만 반환합니다.
            response = {"ok": False, "error": f"robot_arm_server connection failed: {e}"}

        # 실패 응답이면 출력합니다.
        if not response.get("ok", False):
            print("[ARM SERVER ERROR]", response)

        # 응답을 반환합니다.
        return response

    # 현재 TCP 좌표를 가져오는 함수입니다.
    def get_coords(self):

        # 서버에 현재 좌표 요청을 보냅니다.
        response = self.request({"cmd": "get_coords"})

        # 좌표를 반환합니다.
        return response.get("coords", None)

    # 관절 각도 이동 명령입니다.
    def send_angles(self, angles, speed, wait=0.7):

        # 서버에 관절 이동 요청을 보냅니다.
        return self.request({
            "cmd": "send_angles",
            "angles": angles,
            "speed": int(speed),
            "wait": float(wait)
        })

    # TCP 좌표 이동 명령입니다.
    def send_coords(self, coords, speed, mode=1, wait=0.0):

        # 서버에 좌표 이동 요청을 보냅니다.
        return self.request({
            "cmd": "send_coords",
            "coords": coords,
            "speed": int(speed),
            "mode": int(mode),
            "wait": float(wait)
        })

    # 빠른 수동 jog 명령입니다.
    def jog(self, axis, delta, speed):

        # 서버에 jog 요청을 보냅니다.
        return self.request({
            "cmd": "jog",
            "axis": axis,
            "delta": float(delta),
            "speed": int(speed)
        })

    # 그리퍼 TCP 기준 한 단계 이동 명령입니다.
    def tcp_step(self, axis, delta, speed):

        # x/y/z 좌표만 바꾸고 rx/ry/rz는 유지합니다.
        return self.request({
            "cmd": "tcp_step",
            "axis": axis,
            "delta": float(delta),
            "speed": int(speed)
        })

    # 그리퍼 TCP 좌표 캐시 초기화 명령입니다.
    def reset_tcp_cache(self):

        # 다음 이동 때 현재 TCP 좌표를 다시 읽게 합니다.
        return self.request({
            "cmd": "reset_tcp_cache"
        })

    # 연속 수동 이동 시작 명령입니다.
    def jog_start(self, axis, direction, speed):

        # 서버에 연속 jog 시작 요청을 보냅니다.
        return self.request({
            "cmd": "jog_start",
            "axis": axis,
            "direction": int(direction),
            "speed": int(speed)
        })

    # 연속 수동 이동 정지 명령입니다.
    def jog_stop(self):

        # 서버에 연속 jog 정지 요청을 보냅니다.
        return self.request({
            "cmd": "jog_stop"
        })

    # 손으로 위치를 맞출 수 있게 서보 토크를 풉니다.
    def release_all_servos(self):

        return self.request({
            "cmd": "release_all_servos"
        })

    # 손으로 맞춘 위치를 유지하도록 서보 토크를 다시 잡습니다.
    def focus_all_servos(self):

        return self.request({
            "cmd": "focus_all_servos"
        })

    # 그리퍼 제어 명령입니다.
    def set_gripper_value(self, value, speed, wait=0.5):

        # 서버에 그리퍼 요청을 보냅니다.
        return self.request({
            "cmd": "set_gripper",
            "value": int(value),
            "speed": int(speed),
            "wait": float(wait)
        })


# 직사각형 자동 집기 클라이언트 클래스입니다.
class ArmCameraRectAutoPickClient:

    # 초기화 함수입니다.
    def __init__(self):

        # 설정을 불러옵니다.
        self.cfg = load_config()

        # 로봇팔 서버 클라이언트를 생성합니다.
        self.arm = ArmClient()

        # 카메라 장치를 직접 열지 않고 보정된 ROS 영상만 사용합니다.
        self.video_device = "/camera/image_rect"

        # ROS 2가 아직 초기화되지 않았다면 초기화합니다.
        if not rclpy.ok():
            rclpy.init(args=None)

        # 보정 영상 수신 전용 ROS 노드를 생성합니다.
        self.ros_node = Node(
            "arm_rect_auto_pick_image_receiver"
        )

        # ROS Image를 OpenCV 영상으로 바꾸는 객체입니다.
        self.bridge = CvBridge()

        # 가장 최근 영상 하나만 저장합니다.
        self.latest_frame = None

        # 영상 콜백과 화면 스레드 사이의 프레임 접근을 보호합니다.
        self.camera_lock = threading.Lock()

        # 카메라에서는 과거 프레임보다 최신 프레임이 중요합니다.
        # 큐를 1장으로 제한해 오래된 프레임이 쌓이지 않게 합니다.
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # 보정된 로봇팔 카메라 영상을 구독합니다.
        self.image_sub = self.ros_node.create_subscription(
            Image,
            self.video_device,
            self.image_callback,
            camera_qos
        )

        # ROS 영상 수신을 화면 처리와 별도 스레드에서 실행합니다.
        self.ros_spin_running = True

        self.ros_spin_thread = threading.Thread(
            target=self.ros_spin_loop,
            daemon=True
        )

        self.ros_spin_thread.start()

        # 영상 검출 부하 조절 변수입니다.
        self.vision_frame_count = 0
        self.cached_detection = None
        self.cached_edges = np.zeros(
            (480, 640),
            dtype=np.uint8
        )

        # 기본값 2이면 검출은 약 15FPS, 화면은 최신 프레임으로 갱신합니다.
        self.vision_process_every_n_frames = max(
            1,
            int(
                self.cfg.get(
                    "vision_process_every_n_frames",
                    2
                )
            )
        )

        # 디버그 에지 창은 기본적으로 끕니다.
        self.show_edges = bool(
            self.cfg.get(
                "show_edges",
                False
            )
        )

        # 창 이름입니다.
        self.window_name = "arm_camera_rect_auto_pick_client"

        # 작업 중복 방지 변수입니다.
        self.task_busy = False

        # 작업 lock입니다.
        self.task_lock = threading.Lock()

        # JOINT_ASYNC_QUEUE_INIT
        # 관절 키 명령을 저장하는 큐입니다.
        self.joint_jog_queue = queue.Queue(maxsize=6)

        # 관절 키 명령을 순서대로 처리하는 전용 스레드입니다.
        self.joint_jog_worker = threading.Thread(
            target=self.joint_jog_worker_loop,
            daemon=True
        )

        # 관절 제어 스레드를 시작합니다.
        self.joint_jog_worker.start()

        # camera_lock은 영상 구독 생성 전에 이미 초기화했습니다.

        # 수동 이동 명령입니다.
        self.manual_cmd = None

        # 마지막 수동 키 입력 시간입니다.
        self.last_manual_key_time = 0.0

        # 수동 명령 lock입니다.
        self.manual_lock = threading.Lock()

        # 현재 연속 jog 명령입니다.
        self.active_jog_cmd = None

        # jog_coord 사용 가능 여부입니다.
        self.continuous_jog_supported = True

        # 수동 이동 스레드입니다.
        self.teleop_thread = threading.Thread(target=self.teleop_loop, daemon=True)

        # 수동 이동 스레드를 시작합니다.
        self.teleop_thread.start()

        # 직사각형이 연속해서 안정적으로 보인 횟수입니다.
        self.auto_stable_count = 0

        # 직사각형이 보이지 않은 연속 프레임 수입니다.
        self.auto_missing_count = 0

        # 이전 프레임에서 검출된 물체 중심입니다.
        self.last_detect_center = None

        # 한 물체를 중복해서 계속 집지 않도록 막는 변수입니다.
        self.auto_pick_latched = False

        # 마지막 자동 집기 시작 시간입니다.
        self.last_auto_pick_time = 0.0

        # OBJECT_READY_MEMORY_INIT
        # 카메라에서 집을 물체가 인식된 상태인지 저장합니다.
        self.object_ready = False

        # 마지막으로 물체를 정상 인식한 시간입니다.
        self.last_recognized_time = 0.0

        # 마지막으로 인식한 물체 정보를 저장합니다.
        self.last_recognized_detection = None

        # 같은 물체에 대해 READY 로그가 반복되지 않게 합니다.
        self.object_ready_announced = False

        # n 키로 시작한 직접 집기 수동 보정 상태입니다.
        self.pick_manual_calibration_active = isinstance(
            self.cfg.get(
                "pending_direct_pick_manual_calibration",
                None
            ),
            dict
        )

        # 시작 로그입니다.
        print("[OK] arm_camera_rect_auto_pick_client started")
        print(f"[CAMERA] {self.video_device}")
        print(f"[CONFIG] {CONFIG_PATH}")
        print("[OBJECT] rectangle 33 x 78 x 35 mm")
        print("[KEY] i/k=z, j/l=y, u/m=x, q=center, w=pick+return home, e=place")
        print("[FLOW] 시작 자세 이동 -> 물체 인식 -> w 한 번 -> 집기 -> 시작 자세 복귀")
        print("[KEY] a=startup pose, t=save current as camera_ready, s=save scan, p=save place")
        print("[KEY] n=start manual pick calibration, y=save manual pick pose")

        # 시작할 때 카메라 기본 자세로 이동합니다.
        if bool(self.cfg.get("startup_move_enabled", True)):

            # 로봇팔 서버와 실제 로봇 통신이 준비될 때까지 기다립니다.
            server_ready = self.wait_for_arm_server(
                timeout_sec=30.0
            )

            # 서버 준비가 완료된 경우에만 초기 자세로 이동합니다.
            if server_ready:
                self.go_camera_ready_pose()

            # 서버가 준비되지 않으면 오류를 출력합니다.
            else:
                print(
                    "[STARTUP ERROR] "
                    "robot_arm_server가 준비되지 않아 "
                    "초기 자세 이동을 실행하지 못했습니다."
                )

    # 카메라에서 현재 보이는 물체를 집기 준비 상태로 기억합니다.
    def remember_visible_object(self, detection):
        # 최초로 인식한 물체 좌표는 집기가 끝날 때까지 덮어쓰지 않습니다.
        if not isinstance(detection, dict):
            return False

        if (
            bool(getattr(self, "object_ready", False))
            and isinstance(
                getattr(self, "last_recognized_detection", None),
                dict
            )
        ):
            return True

        center = detection.get("center", None)

        if (
            not isinstance(center, (list, tuple))
            or len(center) < 2
        ):
            return False

        recognition_coords = self.get_coords()

        if (
            not isinstance(recognition_coords, list)
            or len(recognition_coords) < 6
        ):
            print("[FIRST TARGET LOCK ERROR] TCP 좌표 읽기 실패")
            return False

        frame_width = 640
        frame_height = 480

        with self.camera_lock:
            if self.latest_frame is not None:
                frame_height, frame_width = self.latest_frame.shape[:2]

        locked_time = time.time()

        self.last_recognized_time = locked_time
        self.last_recognized_detection = {
            "center": (
                float(center[0]),
                float(center[1])
            ),
            "confidence": float(
                detection.get("confidence", 0.0)
            ),
            "depth_mm": detection.get("depth_mm", None),
            "px_per_mm": detection.get("px_per_mm", None),
            "long_px": detection.get("long_px", None),
            "short_px": detection.get("short_px", None),
            "frame_width": int(frame_width),
            "frame_height": int(frame_height),
            "tcp_coords_at_recognition": [
                float(value)
                for value in recognition_coords[:6]
            ],
            "recognized_at": locked_time,
        }

        self.object_ready = True
        self.object_ready_announced = True

        confidence_percent = (
            float(detection.get("confidence", 0.0)) * 100.0
        )

        print(
            "[FIRST TARGET LOCK] "
            f"pixel=({float(center[0]):.1f}, "
            f"{float(center[1]):.1f}) | "
            f"depth={detection.get('depth_mm', None)} | "
            f"confidence={confidence_percent:.1f}%"
        )

        print(
            "[FIRST TARGET LOCK] "
            "이 좌표는 집기가 끝날 때까지 갱신하지 않습니다. "
            "w 키를 누르면 자동 집기를 시작합니다."
        )

        return True



    # 그리퍼 상태를 바꾸지 않고 카메라 초기 자세로 복귀합니다.
    def return_to_camera_ready_pose(self):

        # 설정된 초기 관절 각도를 가져옵니다.
        angles = self.cfg.get("camera_ready_angles", None)

        # 초기 관절값 형식을 확인합니다.
        if not isinstance(angles, list) or len(angles) < 6:
            print(
                "[RETURN HOME ERROR] "
                "camera_ready_angles 값이 올바르지 않습니다:",
                angles
            )
            return False

        # 초기 자세 복귀 로그입니다.
        print("[RETURN HOME] 초기 카메라 자세로 복귀:", angles[:6])

        # 그리퍼는 닫힌 상태를 유지하고 관절만 초기 자세로 이동합니다.
        response = self.arm.send_angles(
            angles[:6],
            int(self.cfg.get("return_home_speed", self.cfg.get("angle_speed", 35))),
            wait=float(self.cfg.get("return_home_wait_sec", 0.8))
        )

        # 서버 응답이 올바른지 확인합니다.
        if not isinstance(response, dict) or not response.get("ok", False):
            print("[RETURN HOME ERROR] 초기 자세 복귀 실패:", response)
            return False

        # TCP 좌표 캐시를 초기화합니다.
        self.arm.reset_tcp_cache()

        # 복귀 완료 로그입니다.
        print("[RETURN HOME] 초기 자세 복귀 완료")
        return True


    # 그리퍼를 엽니다.
    def open_gripper(self):

        value = int(
            self.cfg.get(
                "gripper_open",
                100
            )
        )
        speed = int(
            self.cfg.get(
                "gripper_speed",
                50
            )
        )
        wait_sec = float(
            self.cfg.get(
                "manual_gripper_wait_sec",
                0.6
            )
        )

        print(f"[GRIPPER] open value={value} speed={speed}")
        response = self.arm.set_gripper_value(
            value,
            speed,
            wait=wait_sec
        )

        if (
            not isinstance(response, dict)
            or not response.get("ok", False)
        ):
            print("[GRIPPER ERROR] open failed:", response)
            return False

        self.cfg["last_gripper_command"] = {
            "action": "open",
            "value": value,
            "speed": speed,
            "timestamp": round(time.time(), 3)
        }
        save_config(self.cfg)

        print("[GRIPPER] open complete")
        return True


    # 그리퍼를 닫습니다.
    def close_gripper(self):

        value = int(
            self.cfg.get(
                "gripper_close",
                20
            )
        )
        speed = int(
            self.cfg.get(
                "gripper_speed",
                50
            )
        )
        wait_sec = float(
            self.cfg.get(
                "manual_gripper_wait_sec",
                0.6
            )
        )

        print(f"[GRIPPER] close value={value} speed={speed}")
        response = self.arm.set_gripper_value(
            value,
            speed,
            wait=wait_sec
        )

        if (
            not isinstance(response, dict)
            or not response.get("ok", False)
        ):
            print("[GRIPPER ERROR] close failed:", response)
            return False

        self.cfg["last_gripper_command"] = {
            "action": "close",
            "value": value,
            "speed": speed,
            "timestamp": round(time.time(), 3)
        }
        save_config(self.cfg)

        print("[GRIPPER] close complete")
        return True


    # 기억된 물체를 자동으로 집고 초기 자세로 복귀하는 전체 동작입니다.
    # 카메라에서 인식한 물체의 근처까지 X/Y로 이동합니다.
    def coarse_approach_to_object(self):

        # 보조 접근 반복 횟수입니다.
        maximum_iterations = int(
            self.cfg.get(
                "coarse_approach_max_iter",
                10
            )
        )

        # 물체가 화면 중앙에서 이 픽셀 안에 들어오면 근처 도착으로 봅니다.
        tolerance_px = float(
            self.cfg.get(
                "coarse_approach_tolerance_px",
                18.0
            )
        )

        # 한 번에 이동할 최대 거리입니다.
        maximum_step_mm = abs(
            float(
                self.cfg.get(
                    "coarse_approach_max_step_mm",
                    24.0
                )
            )
        )

        # 보정 행렬이 있으면 픽셀 오차를 로봇 X/Y 이동량으로 직접 변환합니다.
        use_pixel_jacobian = bool(
            self.cfg.get(
                "coarse_use_pixel_jacobian",
                True
            )
        )

        pixel_jacobian = None

        if use_pixel_jacobian:
            matrix_data = self.cfg.get(
                "pixel_jacobian",
                None
            )

            try:
                pixel_jacobian = np.array(
                    matrix_data,
                    dtype=float
                )

                if pixel_jacobian.shape != (2, 2):
                    pixel_jacobian = None

            except Exception:
                pixel_jacobian = None

        # 영상 세로 오차를 로봇 X축 이동으로 바꾸는 값입니다.
        x_from_image_y = float(
            self.cfg.get(
                "coarse_x_from_image_y",
                self.cfg.get(
                    "x_from_image_y",
                    -0.12
                )
            )
        )

        # 영상 가로 오차를 로봇 Y축 이동으로 바꾸는 값입니다.
        y_from_image_x = float(
            self.cfg.get(
                "coarse_y_from_image_x",
                self.cfg.get(
                    "y_from_image_x",
                    -0.12
                )
            )
        )

        # 가장 최근 중심 오차입니다.
        last_error_distance = None
        previous_safe_coords = None
        used_jacobian_fallback = False

        # 물체가 화면 중앙 근처로 올 때까지 반복합니다.
        for iteration in range(1, maximum_iterations + 1):

            # 가장 최근의 보정 영상을 읽습니다.
            frame = self.read_frame()

            # 영상이 없으면 중단합니다.
            if frame is None:
                print("[COARSE APPROACH ERROR] 카메라 영상 없음")
                return False

            # 현재 영상에서 물체를 다시 인식합니다.
            detection, _ = self.detect_rectangle_object(frame)

            # 현재 프레임에서 물체를 못 찾은 경우 마지막 인식값을 사용합니다.
            if detection is None:

                detection = self.last_recognized_detection

                # 마지막 인식값도 없으면 이동할 수 없습니다.
                if detection is None:
                    print(
                        "[COARSE APPROACH ERROR] "
                        "현재 및 기억된 물체 인식값 없음"
                    )
                    return False

                print(
                    "[COARSE APPROACH] "
                    "현재 프레임 미검출, 마지막 인식 위치 사용"
                )

            # 영상 크기입니다.
            frame_height, frame_width = frame.shape[:2]

            # 보조 접근에서는 그리퍼 보정점이 아니라 화면 중앙을 사용합니다.
            # 정확한 집기점이 없어도 로봇팔 전체를 물체 근처까지 보내기 위함입니다.
            target_u = frame_width / 2.0
            target_v = frame_height / 2.0

            # 인식된 물체 중심입니다.
            center_u, center_v = detection.get(
                "center",
                (target_u, target_v)
            )

            # 영상 중심과 물체 사이의 픽셀 오차입니다.
            error_u = float(center_u) - target_u
            error_v = float(center_v) - target_v

            # 전체 중심 오차 거리입니다.
            error_distance = (
                error_u ** 2
                + error_v ** 2
            ) ** 0.5

            # 현재 오차를 출력합니다.
            print(
                "[COARSE APPROACH] "
                f"{iteration}/{maximum_iterations} | "
                f"err_px=({error_u:.1f}, {error_v:.1f}) | "
                f"distance={error_distance:.1f}"
            )

            # 물체가 화면 중앙 근처에 들어오면 성공입니다.
            if error_distance <= tolerance_px:

                print(
                    "[COARSE APPROACH OK] "
                    "로봇팔이 물체 근처에 도착했습니다."
                )

                return True

            # 이전 이동보다 오차가 크게 증가하면 방향이 잘못된 것입니다.
            if (
                last_error_distance is not None
                and error_distance > last_error_distance * 1.35
            ):
                if (
                    isinstance(previous_safe_coords, list)
                    and len(previous_safe_coords) >= 6
                ):
                    print(
                        "[COARSE APPROACH ROLLBACK] "
                        "오차 증가 전 좌표로 복귀"
                    )
                    self.send_coords(
                        previous_safe_coords,
                        "coarse_approach_rollback",
                        wait=float(
                            self.cfg.get(
                                "coarse_approach_wait_sec",
                                0.32
                            )
                        )
                    )

                if (
                    pixel_jacobian is not None
                    and not used_jacobian_fallback
                ):
                    print(
                        "[COARSE APPROACH FALLBACK] "
                        "보정 행렬 이동 방향이 맞지 않아 "
                        "부호 기반 보정으로 전환합니다."
                    )
                    pixel_jacobian = None
                    used_jacobian_fallback = True
                    last_error_distance = None
                    time.sleep(
                        float(
                            self.cfg.get(
                                "coarse_approach_settle_sec",
                                0.12
                            )
                        )
                    )
                    continue

                print(
                    "[COARSE APPROACH STOP] "
                    "오차가 증가하여 추가 이동을 중단합니다. "
                    "coarse_x_from_image_y 또는 "
                    "coarse_y_from_image_x 부호를 반대로 설정하세요."
                )

                return False

            # 현재 오차를 다음 반복 비교용으로 저장합니다.
            last_error_distance = error_distance

            if pixel_jacobian is not None:
                try:
                    pixel_error = np.array(
                        [error_u, error_v],
                        dtype=float
                    )
                    robot_delta = (
                        -np.linalg.inv(pixel_jacobian)
                        @ pixel_error
                        * float(
                            self.cfg.get(
                                "coarse_approach_gain",
                                0.85
                            )
                        )
                    )
                    delta_x = float(robot_delta[0])
                    delta_y = float(robot_delta[1])

                except np.linalg.LinAlgError:
                    pixel_jacobian = None
                    delta_x = error_v * x_from_image_y
                    delta_y = error_u * y_from_image_x

            else:
                # 영상 세로 오차를 로봇 X축 이동량으로 변환합니다.
                delta_x = error_v * x_from_image_y

                # 영상 가로 오차를 로봇 Y축 이동량으로 변환합니다.
                delta_y = error_u * y_from_image_x

            # 한 번에 이동할 최대 거리로 제한합니다.
            delta_x = max(
                -maximum_step_mm,
                min(maximum_step_mm, delta_x)
            )

            delta_y = max(
                -maximum_step_mm,
                min(maximum_step_mm, delta_y)
            )

            # 너무 작은 이동은 실제 로봇에서 무시될 수 있어 최소 이동량을 적용합니다.
            minimum_step_mm = abs(
                float(
                    self.cfg.get(
                        "coarse_approach_min_step_mm",
                        1.0
                    )
                )
            )

            # X축 오차가 남아 있는데 이동량이 너무 작으면 최소 이동량을 적용합니다.
            if (
                abs(error_v) > tolerance_px / 2.0
                and 0.0 < abs(delta_x) < minimum_step_mm
            ):
                delta_x = (
                    minimum_step_mm
                    if delta_x > 0.0
                    else -minimum_step_mm
                )

            # Y축 오차가 남아 있는데 이동량이 너무 작으면 최소 이동량을 적용합니다.
            if (
                abs(error_u) > tolerance_px / 2.0
                and 0.0 < abs(delta_y) < minimum_step_mm
            ):
                delta_y = (
                    minimum_step_mm
                    if delta_y > 0.0
                    else -minimum_step_mm
                )

            # 현재 로봇 TCP 좌표를 읽습니다.
            coords = self.get_coords()

            # 좌표를 읽지 못하면 중단합니다.
            if coords is None:
                print(
                    "[COARSE APPROACH ERROR] "
                    "현재 로봇 좌표 읽기 실패"
                )
                return False

            previous_safe_coords = coords[:6]

            # 현재 좌표를 복사합니다.
            target_coords = coords[:6]

            # X/Y 좌표만 보조 이동량만큼 변경합니다.
            target_coords[0] += delta_x
            target_coords[1] += delta_y

            # 이동량을 출력합니다.
            print(
                "[COARSE APPROACH MOVE] "
                f"dx={delta_x:+.2f}mm, "
                f"dy={delta_y:+.2f}mm"
            )

            # 물체 근처로 이동합니다.
            moved = self.send_coords(
                target_coords,
                "coarse_object_approach",
                wait=float(
                    self.cfg.get(
                        "coarse_approach_wait_sec",
                        0.32
                    )
                )
            )

            # 이동 실패 시 중단합니다.
            if not moved:
                print(
                    "[COARSE APPROACH ERROR] "
                    "로봇 이동 명령 실패"
                )
                return False

            # 이동 후 카메라와 로봇이 안정될 시간을 기다립니다.
            time.sleep(
                float(
                    self.cfg.get(
                        "coarse_approach_settle_sec",
                        0.12
                    )
                )
            )

        # 최대 반복 후에도 중앙 근처에 오지 못했습니다.
        print(
            "[COARSE APPROACH STOP] "
            "최대 반복에 도달했습니다. "
            "현재 위치에서 정지합니다."
        )

        return False


    # 초기 카메라 접근 후 현재 보이는 컨테이너 좌표로 다시 잠급니다.
    def relock_visible_object_after_approach(self):

        frame = self.read_frame()

        if frame is None:
            print("[TARGET RELOCK ERROR] 카메라 영상 없음")
            return False

        detection, _ = self.detect_rectangle_object(frame)

        if not isinstance(detection, dict):
            print("[TARGET RELOCK ERROR] 현재 프레임에서 컨테이너 미검출")
            return False

        center = detection.get("center", None)

        if (
            not isinstance(center, (list, tuple))
            or len(center) < 2
        ):
            print("[TARGET RELOCK ERROR] 중심 좌표 없음")
            return False

        recognition_coords = self.get_coords()

        if (
            not isinstance(recognition_coords, list)
            or len(recognition_coords) < 6
        ):
            print("[TARGET RELOCK ERROR] TCP 좌표 읽기 실패")
            return False

        frame_height, frame_width = frame.shape[:2]
        locked_time = time.time()

        self.last_recognized_time = locked_time
        self.last_recognized_detection = {
            "center": (
                float(center[0]),
                float(center[1])
            ),
            "confidence": float(
                detection.get("confidence", 0.0)
            ),
            "depth_mm": detection.get("depth_mm", None),
            "px_per_mm": detection.get("px_per_mm", None),
            "long_px": detection.get("long_px", None),
            "short_px": detection.get("short_px", None),
            "frame_width": int(frame_width),
            "frame_height": int(frame_height),
            "tcp_coords_at_recognition": [
                float(value)
                for value in recognition_coords[:6]
            ],
            "recognized_at": locked_time,
        }
        self.object_ready = True
        self.object_ready_announced = True

        print(
            "[TARGET RELOCK OK] "
            f"pixel=({float(center[0]):.1f}, "
            f"{float(center[1]):.1f}) | "
            f"depth={detection.get('depth_mm', None)} | "
            f"tcp={[round(float(value), 2) for value in recognition_coords[:3]]}"
        )

        return True


    # 첫 프레임에서 저장한 물체 위치로 이동하고 자동 집기합니다.
    # 2번 관절을 이용해 로봇팔 자세를 먼저 낮춥니다.
    def joint2_assisted_descent(self, final_target_z):

        # 기능 사용 여부입니다.
        if not bool(
            self.cfg.get(
                "joint2_assist_enabled",
                True
            )
        ):
            return self.get_coords()

        # 현재 TCP 좌표를 읽습니다.
        start_coords = self.get_coords()

        # 좌표를 읽지 못하면 기존 Cartesian 하강만 사용합니다.
        if (
            not isinstance(start_coords, list)
            or len(start_coords) < 6
        ):

            print(
                "[JOINT2 ASSIST WARNING] "
                "현재 좌표 읽기 실패, TCP Z 하강만 사용"
            )

            return None

        # 현재 좌표를 실수형으로 복사합니다.
        start_coords = [
            float(value)
            for value in start_coords[:6]
        ]

        # 마지막 TCP 하강을 위해 남겨둘 높이입니다.
        reserve_z_mm = abs(
            float(
                self.cfg.get(
                    "joint2_assist_reserve_z_mm",
                    20.0
                )
            )
        )

        # 2번 관절 보조 하강의 목표 Z입니다.
        assist_target_z = (
            float(final_target_z)
            + reserve_z_mm
        )

        # 이미 충분히 낮으면 관절 보조를 생략합니다.
        if float(start_coords[2]) <= assist_target_z:

            print(
                "[JOINT2 ASSIST] "
                "현재 높이가 충분히 낮아 관절 보조를 생략합니다."
            )

            return start_coords

        # 관절 이동 속도입니다.
        joint_speed = int(
            self.cfg.get(
                "joint2_assist_speed",
                18
            )
        )

        # 관절 이동 후 기다릴 시간입니다.
        joint_wait_sec = float(
            self.cfg.get(
                "joint2_assist_wait_sec",
                0.45
            )
        )

        candidate_joints = self.cfg.get(
            "joint_assist_candidate_joints",
            [2, 3, 4]
        )

        if not isinstance(candidate_joints, list):
            candidate_joints = [2, 3, 4]

        candidate_joints = [
            int(joint_id)
            for joint_id in candidate_joints
            if 1 <= int(joint_id) <= 6
        ]

        if not candidate_joints:
            candidate_joints = [2, 3, 4]

        joint_min_values = self.cfg.get(
            "self_collision_joint_min_deg",
            [-165.0, -125.0, -145.0, -135.0, -155.0, -175.0]
        )
        joint_max_values = self.cfg.get(
            "self_collision_joint_max_deg",
            [165.0, 125.0, 145.0, 135.0, 155.0, 175.0]
        )

        # 지정 관절을 한 번 이동시키는 내부 함수입니다.
        def move_joint(joint_id, delta_deg):
            joint_index = int(joint_id) - 1
            joint_min_deg = float(joint_min_values[joint_index])
            joint_max_deg = float(joint_max_values[joint_index])

            # 서버에 관절 상대 이동을 요청합니다.
            response = self.arm.request(
                {
                    "cmd": "joint_step",
                    "joint": int(joint_id),
                    "delta": float(delta_deg),
                    "speed": joint_speed,
                    "wait": joint_wait_sec,
                    "min_angle": joint_min_deg,
                    "max_angle": joint_max_deg
                }
            )

            # 명령 성공 여부를 반환합니다.
            return (
                isinstance(response, dict)
                and response.get("ok", False)
            )

        # 시험 이동 각도입니다.
        probe_deg = abs(
            float(
                self.cfg.get(
                    "joint2_probe_deg",
                    2.0
                )
            )
        )

        # 시험 이동에서 최소한 줄어야 할 Z입니다.
        minimum_probe_drop_mm = abs(
            float(
                self.cfg.get(
                    "joint2_probe_min_drop_mm",
                    0.7
                )
            )
        )

        # 후보 관절과 양방향을 모두 시험합니다.
        probe_results = []

        for candidate_joint in candidate_joints:
            for candidate_direction in (1, -1):

                # 시험 직전 좌표를 읽습니다.
                before_probe = self.get_coords()

                # 좌표가 없으면 시험을 중단합니다.
                if (
                    not isinstance(before_probe, list)
                    or len(before_probe) < 6
                ):
                    continue

                # 시험 방향으로 관절을 이동합니다.
                moved_probe = move_joint(
                    candidate_joint,
                    candidate_direction
                    * probe_deg
                )

                # 이동 명령 실패면 다음 방향을 시험합니다.
                if not moved_probe:
                    continue

                # 실제 좌표가 갱신될 시간을 기다립니다.
                time.sleep(0.12)

                # 시험 후 좌표를 읽습니다.
                after_probe = self.get_coords()

                # 원래 각도로 되돌립니다.
                move_joint(
                    candidate_joint,
                    -candidate_direction
                    * probe_deg
                )

                # 복귀 후 잠시 기다립니다.
                time.sleep(0.12)

                # 좌표가 비정상이면 결과를 버립니다.
                if (
                    not isinstance(after_probe, list)
                    or len(after_probe) < 6
                ):
                    continue

                # 양수이면 아래로 내려간 거리입니다.
                drop_mm = (
                    float(before_probe[2])
                    - float(after_probe[2])
                )

                # 시험 결과를 저장합니다.
                probe_results.append(
                    (
                        drop_mm,
                        candidate_joint,
                        candidate_direction
                    )
                )

                print(
                    "[JOINT ASSIST PROBE] "
                    f"joint={candidate_joint} | "
                    f"direction={candidate_direction:+d} | "
                    f"z={float(before_probe[2]):.2f}"
                    f"->{float(after_probe[2]):.2f} | "
                    f"drop={drop_mm:+.2f}mm"
                )

        best_joint = 0
        joint_direction = 0

        # 가장 많이 내려간 관절/방향을 선택합니다.
        if probe_results:
            best_drop_mm, best_joint, best_direction = max(
                probe_results,
                key=lambda item: item[0]
            )

            # 실제로 Z가 내려간 경우에만 사용합니다.
            if best_drop_mm >= minimum_probe_drop_mm:
                best_joint = int(best_joint)
                joint_direction = int(best_direction)

                print(
                    "[JOINT ASSIST PROBE OK] "
                    f"joint={best_joint} | "
                    f"direction={joint_direction:+d} | "
                    f"시험 하강={best_drop_mm:.2f}mm"
                )

        # 내려가는 관절을 찾지 못하면 기존 Z 하강으로 넘어갑니다.
        if best_joint == 0 or joint_direction == 0:

            print(
                "[JOINT ASSIST WARNING] "
                "TCP Z가 내려가는 관절 방향을 찾지 못했습니다. "
                "TCP Z 하강만 사용합니다."
            )

            return self.get_coords()

        # 한 단계 관절 이동 각도입니다.
        step_deg = abs(
            float(
                self.cfg.get(
                    "joint2_assist_step_deg",
                    2.5
                )
            )
        )

        # 전체 관절 사용 최대 각도입니다.
        maximum_total_deg = abs(
            float(
                self.cfg.get(
                    "joint2_assist_max_total_deg",
                    14.0
                )
            )
        )

        # 한 단계 이동에서 최소한 줄어야 할 Z입니다.
        minimum_step_drop_mm = abs(
            float(
                self.cfg.get(
                    "joint2_assist_min_step_drop_mm",
                    0.5
                )
            )
        )

        # 관절 사용 중 허용할 최대 X/Y 이동 거리입니다.
        maximum_xy_drift_mm = abs(
            float(
                self.cfg.get(
                    "joint2_assist_max_xy_drift_mm",
                    35.0
                )
            )
        )

        # 현재 좌표입니다.
        current_coords = self.get_coords()

        # 좌표 형식을 확인합니다.
        if (
            not isinstance(current_coords, list)
            or len(current_coords) < 6
        ):
            current_coords = start_coords[:6]

        # 사용한 전체 관절 각도입니다.
        used_total_deg = 0.0

        # 최대 각도 또는 목표 높이까지 반복합니다.
        while (
            float(current_coords[2]) > assist_target_z
            and used_total_deg < maximum_total_deg
        ):

            # 남은 허용 각도입니다.
            remaining_deg = (
                maximum_total_deg
                - used_total_deg
            )

            # 이번 이동 각도입니다.
            this_step_deg = min(
                step_deg,
                remaining_deg
            )

            # 이동 전 좌표입니다.
            before_step = current_coords[:6]

            # 선택된 관절을 내려가는 방향으로 이동합니다.
            moved = move_joint(
                best_joint,
                joint_direction
                * this_step_deg
            )

            # 이동 명령 실패 시 보조 하강을 종료합니다.
            if not moved:

                print(
                    "[JOINT ASSIST STOP] "
                    f"관절 {best_joint} 이동 명령 실패"
                )

                break

            # 좌표 갱신을 기다립니다.
            time.sleep(0.12)

            # 이동 후 좌표를 읽습니다.
            after_step = self.get_coords()

            # 좌표를 읽지 못하면 마지막 이동을 되돌립니다.
            if (
                not isinstance(after_step, list)
                or len(after_step) < 6
            ):

                move_joint(
                    best_joint,
                    -joint_direction
                    * this_step_deg
                )

                print(
                    "[JOINT ASSIST STOP] "
                    "이동 후 좌표 읽기 실패"
                )

                break

            # 실제로 내려간 Z 거리입니다.
            actual_drop_mm = (
                float(before_step[2])
                - float(after_step[2])
            )

            # 시작 위치 기준 X/Y 이동 거리입니다.
            xy_drift_mm = (
                (
                    float(after_step[0])
                    - float(start_coords[0])
                ) ** 2
                + (
                    float(after_step[1])
                    - float(start_coords[1])
                ) ** 2
            ) ** 0.5

            # 내려가지 않았거나 반대로 움직였으면 마지막 이동을 취소합니다.
            if actual_drop_mm < minimum_step_drop_mm:

                move_joint(
                    best_joint,
                    -joint_direction
                    * this_step_deg
                )

                print(
                    "[JOINT ASSIST STOP] "
                    f"Z 하강 부족: {actual_drop_mm:.2f}mm"
                )

                break

            # X/Y가 너무 많이 벗어나면 마지막 이동을 취소합니다.
            if xy_drift_mm > maximum_xy_drift_mm:

                move_joint(
                    best_joint,
                    -joint_direction
                    * this_step_deg
                )

                print(
                    "[JOINT ASSIST STOP] "
                    f"X/Y 이탈 과다: {xy_drift_mm:.2f}mm"
                )

                break

            # 정상 이동 결과를 반영합니다.
            current_coords = [
                float(value)
                for value in after_step[:6]
            ]

            # 사용한 관절 각도를 누적합니다.
            used_total_deg += this_step_deg

            print(
                "[JOINT ASSIST MOVE] "
                f"joint={best_joint} | "
                f"delta={joint_direction * this_step_deg:+.2f}deg | "
                f"z={float(before_step[2]):.2f}"
                f"->{float(current_coords[2]):.2f} | "
                f"drop={actual_drop_mm:.2f}mm | "
                f"xy_drift={xy_drift_mm:.2f}mm | "
                f"total_angle={used_total_deg:.2f}deg"
            )

        # 관절 이동 후 TCP 좌표 캐시를 초기화합니다.
        reset_response = self.arm.reset_tcp_cache()

        # 완료 상태를 출력합니다.
        print(
            "[JOINT ASSIST COMPLETE] "
            f"start_z={float(start_coords[2]):.2f} | "
            f"current_z={float(current_coords[2]):.2f} | "
            f"target_with_reserve={assist_target_z:.2f} | "
            f"joint={best_joint} | "
            f"used={used_total_deg:.2f}deg"
        )

        # 2번 관절 이동 후의 실제 좌표를 반환합니다.
        return current_coords


    # 컨테이너가 화면에서 충분히 크게 보일 때까지 천천히 접근합니다.
    # 컨테이너가 화면에 크게 찰 때까지 접근하고 손실 시 한 단계 복귀합니다.
    def approach_until_container_fills_frame(self):

        # 긴 변이 화면 가로에서 차지해야 할 목표 비율입니다.
        target_long_ratio = float(
            self.cfg.get(
                "fill_target_long_ratio",
                0.72
            )
        )

        # 짧은 변이 화면 세로에서 차지해야 할 목표 비율입니다.
        target_short_ratio = float(
            self.cfg.get(
                "fill_target_short_ratio",
                0.42
            )
        )

        # 한 번에 내려갈 거리입니다.
        approach_step_mm = abs(
            float(
                self.cfg.get(
                    "fill_approach_step_mm",
                    4.0
                )
            )
        )

        # 접근 중 허용할 최대 전체 하강 거리입니다.
        maximum_total_down_mm = abs(
            float(
                self.cfg.get(
                    "fill_approach_max_down_mm",
                    85.0
                )
            )
        )

        # 접근 최대 반복 횟수입니다.
        maximum_iterations = int(
            self.cfg.get(
                "fill_approach_max_iter",
                28
            )
        )

        # 안전 최소 Z입니다.
        minimum_safe_z = float(
            self.cfg.get(
                "minimum_safe_z_mm",
                25.0
            )
        )

        # 현재까지 내려간 전체 거리입니다.
        total_down_mm = 0.0

        # 마지막 정상 검출 결과입니다.
        last_safe_detection = None

        # 마지막 정상 프레임 크기입니다.
        last_safe_frame_shape = None

        # 마지막 정상 비율입니다.
        last_safe_long_ratio = 0.0
        last_safe_short_ratio = 0.0

        # 마지막 이동 직전의 안전 좌표입니다.
        last_safe_coords = None

        # 마지막 검출 이후 실제 이동을 했는지 나타냅니다.
        moved_after_last_detection = False

        # 한 단계 복귀 여부입니다.
        rolled_back = False

        print(
            "[FILL APPROACH] 시작 | "
            f"long_target={target_long_ratio:.2f} | "
            f"short_target={target_short_ratio:.2f} | "
            f"step={approach_step_mm:.2f}mm"
        )

        # 컨테이너가 목표 크기에 도달할 때까지 반복합니다.
        for iteration in range(1, maximum_iterations + 1):

            # 최신 카메라 프레임을 읽습니다.
            frame = self.read_frame()

            # 현재 프레임의 검출 결과입니다.
            detection = None

            # 영상이 있으면 컨테이너를 검출합니다.
            if frame is not None:
                detection, _ = self.detect_rectangle_object(
                    frame
                )

            # 접근 중 컨테이너가 사라진 경우입니다.
            if not isinstance(detection, dict):

                print(
                    "[FILL APPROACH LOST] "
                    "컨테이너가 화면에서 사라졌습니다."
                )

                # 직전 이동 전 안전 좌표가 있으면 한 단계 복귀합니다.
                if (
                    moved_after_last_detection
                    and isinstance(last_safe_coords, list)
                    and len(last_safe_coords) >= 6
                ):

                    print(
                        "[FILL APPROACH ROLLBACK] "
                        "마지막 이동 전 좌표로 복귀:",
                        [
                            round(float(value), 2)
                            for value in last_safe_coords[:6]
                        ]
                    )

                    returned = self.send_coords(
                        last_safe_coords[:6],
                        "fill_rollback_one_step",
                        wait=float(
                            self.cfg.get(
                                "fill_rollback_wait_sec",
                                0.55
                            )
                        )
                    )

                    if not returned:
                        print(
                            "[FILL APPROACH ERROR] "
                            "한 단계 복귀 실패"
                        )
                        return None

                    time.sleep(
                        float(
                            self.cfg.get(
                                "fill_rollback_settle_sec",
                                0.25
                            )
                        )
                    )

                    rolled_back = True

                    print(
                        "[FILL APPROACH ROLLBACK OK] "
                        "마지막 정상 검출 좌표를 잠급니다."
                    )

                    break

                print(
                    "[FILL APPROACH ERROR] "
                    "복귀할 정상 좌표가 없습니다."
                )

                return None

            # 프레임 크기입니다.
            frame_height, frame_width = frame.shape[:2]

            # 검출된 긴 변과 짧은 변 픽셀 길이입니다.
            long_px = float(
                detection.get(
                    "long_px",
                    0.0
                )
            )

            short_px = float(
                detection.get(
                    "short_px",
                    0.0
                )
            )

            # 화면에서 차지하는 비율입니다.
            long_ratio = (
                long_px
                / max(
                    1.0,
                    float(frame_width)
                )
            )

            short_ratio = (
                short_px
                / max(
                    1.0,
                    float(frame_height)
                )
            )

            # 현재 TCP 좌표를 읽습니다.
            current_coords = self.get_coords()

            if (
                not isinstance(current_coords, list)
                or len(current_coords) < 6
            ):

                print(
                    "[FILL APPROACH ERROR] "
                    "현재 TCP 좌표 읽기 실패"
                )

                return None

            current_coords = [
                float(value)
                for value in current_coords[:6]
            ]

            # 이번 정상 검출값을 안전값으로 저장합니다.
            last_safe_detection = dict(
                detection
            )

            last_safe_frame_shape = (
                frame_height,
                frame_width
            )

            last_safe_long_ratio = long_ratio
            last_safe_short_ratio = short_ratio

            # 다음 이동 후 인식이 사라지면 이 좌표로 돌아옵니다.
            last_safe_coords = current_coords[:6]

            moved_after_last_detection = False

            print(
                "[FILL APPROACH] "
                f"{iteration}/{maximum_iterations} | "
                f"long={long_px:.1f}px "
                f"({long_ratio:.3f}) | "
                f"short={short_px:.1f}px "
                f"({short_ratio:.3f}) | "
                f"down={total_down_mm:.1f}mm"
            )

            # 목표 화면 크기에 도달하면 좌표를 잠급니다.
            if (
                long_ratio >= target_long_ratio
                and short_ratio >= target_short_ratio
            ):

                print(
                    "[FILL APPROACH OK] "
                    "컨테이너가 큰 화면 목표 크기에 도달했습니다."
                )

                break

            # 최대 접근 거리 도달 시 현재 좌표를 잠급니다.
            if total_down_mm >= maximum_total_down_mm:

                print(
                    "[FILL APPROACH LIMIT] "
                    "최대 접근 거리에 도달했습니다."
                )

                break

            # 남은 허용 접근 거리입니다.
            remaining_down_mm = (
                maximum_total_down_mm
                - total_down_mm
            )

            # 이번 단계 이동 거리입니다.
            this_step_mm = min(
                approach_step_mm,
                remaining_down_mm
            )

            # 다음 접근 좌표입니다.
            next_coords = current_coords[:6]

            # X/Y와 자세는 유지하고 Z만 내립니다.
            next_coords[2] = max(
                minimum_safe_z,
                float(current_coords[2])
                - this_step_mm
            )

            # 실제 이동 거리입니다.
            actual_step_mm = (
                float(current_coords[2])
                - float(next_coords[2])
            )

            if actual_step_mm <= 0.5:

                print(
                    "[FILL APPROACH LIMIT] "
                    "minimum_safe_z에 도달했습니다."
                )

                break

            print(
                "[FILL APPROACH MOVE] "
                f"z={float(current_coords[2]):.2f}"
                f"->{float(next_coords[2]):.2f} | "
                f"step={actual_step_mm:.2f}mm"
            )

            moved = self.send_coords(
                next_coords,
                "fill_slow_approach",
                wait=float(
                    self.cfg.get(
                        "fill_approach_move_wait_sec",
                        0.36
                    )
                )
            )

            if not moved:

                print(
                    "[FILL APPROACH ERROR] "
                    "접근 이동 실패"
                )

                return None

            moved_after_last_detection = True
            total_down_mm += actual_step_mm

            time.sleep(
                float(
                    self.cfg.get(
                        "fill_approach_settle_sec",
                        0.20
                    )
                )
            )

        # 정상 검출값이 한 번도 없으면 실패입니다.
        if last_safe_detection is None:

            print(
                "[FILL APPROACH ERROR] "
                "잠글 컨테이너 좌표가 없습니다."
            )

            return None

        # 최종 잠금 좌표를 읽습니다.
        lock_coords = self.get_coords()

        # 읽기 실패 시 마지막 안전 좌표를 사용합니다.
        if (
            not isinstance(lock_coords, list)
            or len(lock_coords) < 6
        ):
            lock_coords = last_safe_coords

        if (
            not isinstance(lock_coords, list)
            or len(lock_coords) < 6
        ):

            print(
                "[FILL APPROACH ERROR] "
                "최종 잠금 좌표가 없습니다."
            )

            return None

        lock_coords = [
            float(value)
            for value in lock_coords[:6]
        ]

        # 컨테이너 중심 픽셀입니다.
        center_u, center_v = last_safe_detection.get(
            "center",
            (0.0, 0.0)
        )

        # 잠금 정보를 설정에 저장합니다.
        self.cfg["last_container_lock"] = {
            "center_u_px": round(
                float(center_u),
                3
            ),
            "center_v_px": round(
                float(center_v),
                3
            ),
            "long_px": round(
                float(
                    last_safe_detection.get(
                        "long_px",
                        0.0
                    )
                ),
                3
            ),
            "short_px": round(
                float(
                    last_safe_detection.get(
                        "short_px",
                        0.0
                    )
                ),
                3
            ),
            "long_ratio": round(
                last_safe_long_ratio,
                5
            ),
            "short_ratio": round(
                last_safe_short_ratio,
                5
            ),
            "depth_mm": last_safe_detection.get(
                "depth_mm",
                None
            ),
            "tcp_coords": [
                round(value, 3)
                for value in lock_coords
            ],
            "rolled_back_one_step": bool(
                rolled_back
            ),
            "timestamp": round(
                time.time(),
                3
            )
        }

        save_config(
            self.cfg
        )

        print(
            "[CONTAINER COORD LOCK] "
            f"pixel=({float(center_u):.1f}, "
            f"{float(center_v):.1f}) | "
            f"ratio=({last_safe_long_ratio:.3f}, "
            f"{last_safe_short_ratio:.3f}) | "
            f"tcp=({lock_coords[0]:.2f}, "
            f"{lock_coords[1]:.2f}, "
            f"{lock_coords[2]:.2f}) | "
            f"rollback={rolled_back}"
        )

        return {
            "lock_coords": lock_coords,
            "detection": last_safe_detection,
            "frame_shape": last_safe_frame_shape,
            "total_down_mm": total_down_mm,
            "rolled_back": rolled_back
        }




    # 현재 TCP 회전값을 카메라 바닥 보기 자세로 저장합니다.
    def save_current_floor_camera_rpy(self):
        # 바닥 보기 TCP 회전값과 6개 관절값을 함께 저장합니다.
        coords = self.get_coords()
        angles_response = self.arm.request({"cmd": "get_angles"})
        angles = angles_response.get("angles", None)

        if (
            not isinstance(coords, list)
            or len(coords) < 6
        ):
            print("[FLOOR CAMERA SAVE ERROR] TCP 좌표 읽기 실패")
            return False

        if (
            not isinstance(angles, list)
            or len(angles) < 6
        ):
            print(
                "[FLOOR CAMERA SAVE ERROR] 관절 각도 읽기 실패:",
                angles_response
            )
            return False

        floor_rpy = [
            float(coords[3]),
            float(coords[4]),
            float(coords[5])
        ]

        floor_angles = [
            float(value)
            for value in angles[:6]
        ]

        self.cfg["floor_camera_rpy"] = floor_rpy
        self.cfg["floor_camera_angles"] = floor_angles
        save_config(self.cfg)

        print(
            "[FLOOR CAMERA SAVE OK] "
            f"rpy={[round(value, 2) for value in floor_rpy]}"
        )
        print(
            "[FLOOR CAMERA SAVE OK] "
            f"angles={[round(value, 2) for value in floor_angles]}"
        )
        print(
            "[FLOOR CAMERA SAVE] "
            "자동 동작에서는 6개 관절을 한 번에 이동합니다."
        )

        return True



        # 잠근 X/Y/Z를 유지하고 카메라가 바닥을 보도록 회전합니다.
    def orient_camera_to_floor(self, lock_coords):

        # 저장된 바닥 보기 자세입니다.
        floor_rpy = self.cfg.get(
            "floor_camera_rpy",
            None
        )

        # 장착 각도마다 값이 달라서 저장값이 없으면 중단합니다.
        if (
            not isinstance(floor_rpy, list)
            or len(floor_rpy) < 3
        ):

            print(
                "[FLOOR CAMERA ERROR] "
                "floor_camera_rpy가 저장되지 않았습니다."
            )

            print(
                "[FLOOR CAMERA GUIDE] "
                "카메라가 바닥을 보도록 수동 조정한 뒤 "
                "카메라 창에서 g를 한 번 누르세요."
            )

            return None

        # 잠금 좌표를 복사합니다.
        floor_coords = [
            float(value)
            for value in lock_coords[:6]
        ]

        # X/Y/Z는 유지하고 회전값만 변경합니다.
        floor_coords[3] = float(
            floor_rpy[0]
        )

        floor_coords[4] = float(
            floor_rpy[1]
        )

        floor_coords[5] = float(
            floor_rpy[2]
        )

        print(
            "[FLOOR CAMERA MOVE] "
            f"xyz=({floor_coords[0]:.2f}, "
            f"{floor_coords[1]:.2f}, "
            f"{floor_coords[2]:.2f}) | "
            f"rpy=({floor_coords[3]:.2f}, "
            f"{floor_coords[4]:.2f}, "
            f"{floor_coords[5]:.2f})"
        )

        moved = self.send_coords(
            floor_coords,
            "camera_face_floor",
            wait=float(
                self.cfg.get(
                    "floor_camera_move_wait_sec",
                    0.85
                )
            )
        )

        if not moved:

            print(
                "[FLOOR CAMERA ERROR] "
                "바닥 보기 자세 이동 실패"
            )

            return None

        time.sleep(
            float(
                self.cfg.get(
                    "floor_camera_settle_sec",
                    0.35
                )
            )
        )

        # 실제 이동 후 좌표를 읽습니다.
        measured_coords = self.get_coords()

        if (
            isinstance(measured_coords, list)
            and len(measured_coords) >= 6
        ):

            floor_coords = [
                float(value)
                for value in measured_coords[:6]
            ]

        self.cfg["last_floor_camera_coords"] = [
            round(value, 3)
            for value in floor_coords
        ]

        save_config(
            self.cfg
        )

        print(
            "[FLOOR CAMERA OK] "
            "카메라 바닥 보기 자세 완료"
        )

        return floor_coords


    # 카메라 중심 좌표를 그리퍼 중심 좌표로 이동합니다.
    def move_camera_center_to_gripper_center(
        self,
        lock_coords
    ):

        # 카메라 중심에서 그리퍼 중심까지의 X 오프셋입니다.
        offset_x_mm = float(
            self.cfg.get(
                "camera_to_gripper_x_mm",
                20.0
            )
        )

        # 카메라 중심에서 그리퍼 중심까지의 Y 오프셋입니다.
        offset_y_mm = float(
            self.cfg.get(
                "camera_to_gripper_y_mm",
                10.0
            )
        )

        # 잠금 좌표를 복사합니다.
        gripper_target_coords = [
            float(value)
            for value in lock_coords[:6]
        ]

        # X/Y 오프셋만 적용합니다.
        gripper_target_coords[0] += offset_x_mm
        gripper_target_coords[1] += offset_y_mm

        print(
            "[CAMERA -> GRIPPER] "
            f"offset_x={offset_x_mm:+.2f}mm | "
            f"offset_y={offset_y_mm:+.2f}mm"
        )

        print(
            "[GRIPPER CENTER TARGET] "
            f"({gripper_target_coords[0]:.2f}, "
            f"{gripper_target_coords[1]:.2f}, "
            f"{gripper_target_coords[2]:.2f})"
        )

        # 그리퍼 중심이 컨테이너 위로 오도록 이동합니다.
        moved = self.send_coords(
            gripper_target_coords,
            "camera_to_gripper_center",
            wait=float(
                self.cfg.get(
                    "camera_to_gripper_wait_sec",
                    0.65
                )
            )
        )

        # 이동 실패 시 None을 반환합니다.
        if not moved:

            print(
                "[CAMERA -> GRIPPER ERROR] "
                "그리퍼 중심 이동 실패"
            )

            return None

        # 이동 후 안정화 시간을 기다립니다.
        time.sleep(
            float(
                self.cfg.get(
                    "camera_to_gripper_settle_sec",
                    0.25
                )
            )
        )

        # 실제 이동 후 좌표를 읽습니다.
        measured_coords = self.get_coords()

        # 실제 좌표를 읽었으면 그것을 사용합니다.
        if (
            isinstance(measured_coords, list)
            and len(measured_coords) >= 6
        ):

            gripper_target_coords = [
                float(value)
                for value in measured_coords[:6]
            ]

        # 그리퍼 중심 좌표를 설정에 저장합니다.
        self.cfg["last_gripper_center_coords"] = [
            round(value, 3)
            for value in gripper_target_coords
        ]

        save_config(
            self.cfg
        )

        return gripper_target_coords


    # 잠근 X/Y 좌표를 유지하면서 수직으로 하강해 집습니다.
    # 카메라-그리퍼 높이 오프셋을 반영해 수직 하강합니다.
    # 카메라-그리퍼 높이 오프셋을 반영해 제한 없이 수직 하강합니다.
    # 현재 6개 관절 각도를 안전 검사 용도로 읽습니다.
    def get_joint_angles_for_safety(self):

        # 서버에 현재 관절 각도를 요청합니다.
        response = self.arm.request(
            {
                "cmd": "get_angles"
            }
        )

        # 응답에서 관절 각도를 가져옵니다.
        angles = response.get(
            "angles",
            None
        )

        # 정상적인 6개 관절값이 아니면 None을 반환합니다.
        if (
            not isinstance(angles, list)
            or len(angles) < 6
        ):

            print(
                "[SELF COLLISION GUARD ERROR] "
                "관절 각도 읽기 실패:",
                response
            )

            return None

        # 앞 6개 값을 실수형으로 반환합니다.
        try:

            return [
                float(value)
                for value in angles[:6]
            ]

        except (TypeError, ValueError):

            print(
                "[SELF COLLISION GUARD ERROR] "
                "관절 각도 형식 오류:",
                angles
            )

            return None


    # 현재 관절 자세가 링크끼리 부딪힐 위험이 있는지 검사합니다.
    def check_self_collision_risk(
        self,
        current_angles,
        previous_angles=None,
        descent_start_angles=None
    ):

        # 관절값 자체가 없으면 안전을 확인할 수 없으므로 위험으로 처리합니다.
        if (
            not isinstance(current_angles, list)
            or len(current_angles) < 6
        ):

            return (
                True,
                "현재 관절 각도 없음"
            )

        # 관절별 절대 최소 각도입니다.
        minimum_angles = self.cfg.get(
            "self_collision_joint_min_deg",
            [
                -165.0,
                -125.0,
                -145.0,
                -135.0,
                -155.0,
                -175.0
            ]
        )

        # 관절별 절대 최대 각도입니다.
        maximum_angles = self.cfg.get(
            "self_collision_joint_max_deg",
            [
                165.0,
                125.0,
                145.0,
                135.0,
                155.0,
                175.0
            ]
        )

        # 각 관절이 절대 안전 범위를 벗어났는지 검사합니다.
        for index in range(6):

            minimum_value = float(
                minimum_angles[index]
            )

            maximum_value = float(
                maximum_angles[index]
            )

            current_value = float(
                current_angles[index]
            )

            if not (
                minimum_value
                <= current_value
                <= maximum_value
            ):

                return (
                    True,
                    (
                        f"J{index + 1} 절대 각도 초과: "
                        f"{current_value:.2f}deg "
                        f"범위=[{minimum_value:.2f}, "
                        f"{maximum_value:.2f}]"
                    )
                )

        # 한 번의 Cartesian 이동에서 허용할 최대 관절 변화량입니다.
        maximum_single_step_delta = abs(
            float(
                self.cfg.get(
                    "self_collision_max_single_step_joint_delta_deg",
                    14.0
                )
            )
        )

        # 직전 관절값과 비교해 갑작스러운 IK 자세 변경을 검사합니다.
        if (
            isinstance(previous_angles, list)
            and len(previous_angles) >= 6
        ):

            for index in range(6):

                step_delta = abs(
                    float(current_angles[index])
                    - float(previous_angles[index])
                )

                if step_delta > maximum_single_step_delta:

                    return (
                        True,
                        (
                            f"J{index + 1} 한 단계 급변: "
                            f"{step_delta:.2f}deg > "
                            f"{maximum_single_step_delta:.2f}deg"
                        )
                    )

        # 하강 시작 자세에서 허용할 관절별 최대 변화량입니다.
        maximum_excursion = self.cfg.get(
            "self_collision_max_descent_excursion_deg",
            [
                35.0,
                45.0,
                55.0,
                65.0,
                70.0,
                90.0
            ]
        )

        # 하강 중 역기구학이 전혀 다른 팔 자세로 접히는 것을 막습니다.
        if (
            isinstance(descent_start_angles, list)
            and len(descent_start_angles) >= 6
        ):

            for index in range(6):

                excursion = abs(
                    float(current_angles[index])
                    - float(descent_start_angles[index])
                )

                allowed_excursion = abs(
                    float(
                        maximum_excursion[index]
                    )
                )

                if excursion > allowed_excursion:

                    return (
                        True,
                        (
                            f"J{index + 1} 하강 시작 자세 이탈: "
                            f"{excursion:.2f}deg > "
                            f"{allowed_excursion:.2f}deg"
                        )
                    )

        # 아래 규칙은 2·3·4번 관절이 몸체 방향으로 과도하게 접히는 것을 막습니다.
        joint2 = float(
            current_angles[1]
        )

        joint3 = float(
            current_angles[2]
        )

        joint4 = float(
            current_angles[3]
        )

        # 2번과 3번 관절이 같은 방향으로 동시에 크게 접힌 상태입니다.
        same_direction_fold_limit = abs(
            float(
                self.cfg.get(
                    "self_collision_same_direction_j2_j3_sum_deg",
                    155.0
                )
            )
        )

        if (
            joint2 * joint3 > 0.0
            and abs(joint2) + abs(joint3)
            >= same_direction_fold_limit
        ):

            return (
                True,
                (
                    "J2/J3 같은 방향 과접힘: "
                    f"|J2|+|J3|="
                    f"{abs(joint2) + abs(joint3):.2f}deg"
                )
            )

        # 어깨·팔꿈치·손목이 모두 크게 접힌 상태를 점수로 검사합니다.
        fold_score = (
            abs(joint2)
            + abs(joint3)
            + 0.50 * abs(joint4)
        )

        maximum_fold_score = abs(
            float(
                self.cfg.get(
                    "self_collision_max_fold_score",
                    220.0
                )
            )
        )

        if fold_score >= maximum_fold_score:

            return (
                True,
                (
                    "팔 링크 과접힘 점수 초과: "
                    f"{fold_score:.2f} >= "
                    f"{maximum_fold_score:.2f}"
                )
            )

        # 모든 검사를 통과했습니다.
        return (
            False,
            "safe"
        )


    # 바닥 Z=0까지 허용하되 관절 자기충돌을 감시하며 수직 하강합니다.
    def vertical_pick_from_locked_coords(
        self,
        gripper_center_coords,
        detection=None
    ):

        # 카메라가 그리퍼보다 위에 있는 실제 거리입니다.
        camera_to_gripper_z_mm = abs(
            float(
                self.cfg.get(
                    "camera_to_gripper_z_mm",
                    60.0
                )
            )
        )

        # 바닥 접촉을 허용하므로 여유 거리는 기본 0mm입니다.
        target_clearance_mm = max(
            0.0,
            float(
                self.cfg.get(
                    "gripper_target_clearance_mm",
                    0.0
                )
            )
        )

        # 카메라가 측정한 최종 깊이입니다.
        camera_depth_mm = None

        if isinstance(detection, dict):

            camera_depth_mm = detection.get(
                "depth_mm",
                None
            )

        try:

            camera_depth_mm = float(
                camera_depth_mm
            )

            if camera_depth_mm <= 0.0:

                camera_depth_mm = None

        except (TypeError, ValueError):

            camera_depth_mm = None

        # 최종 깊이가 있으면 하강 거리를 계산합니다.
        if camera_depth_mm is not None:

            raw_vertical_down_mm = max(
                0.0,
                (
                    camera_depth_mm
                    - camera_to_gripper_z_mm
                    - target_clearance_mm
                )
            )

            vertical_down_mm = (
                raw_vertical_down_mm
                * float(
                    self.cfg.get(
                        "locked_vertical_depth_scale",
                        1.0
                    )
                )
                + float(
                    self.cfg.get(
                        "locked_vertical_depth_bias_mm",
                        0.0
                    )
                )
            )

            print(
                "[VERTICAL DEPTH FLOOR0] "
                f"camera_depth={camera_depth_mm:.2f}mm | "
                f"camera_to_gripper_z={camera_to_gripper_z_mm:.2f}mm | "
                f"clearance={target_clearance_mm:.2f}mm | "
                f"raw_down={raw_vertical_down_mm:.2f}mm | "
                f"down={vertical_down_mm:.2f}mm"
            )

        # 깊이가 없으면 고정 하강량을 사용합니다.
        else:

            vertical_down_mm = max(
                0.0,
                float(
                    self.cfg.get(
                        "locked_vertical_down_mm",
                        45.0
                    )
                )
            )

            print(
                "[VERTICAL DEPTH WARNING] "
                "깊이값이 없어 고정 하강량 사용:",
                vertical_down_mm
            )

        minimum_vertical_down_mm = max(
            0.0,
            float(
                self.cfg.get(
                    "locked_vertical_min_down_mm",
                    105.0
                )
            )
        )

        maximum_vertical_down_mm = max(
            minimum_vertical_down_mm,
            float(
                self.cfg.get(
                    "locked_vertical_max_down_mm",
                    165.0
                )
            )
        )

        if vertical_down_mm < minimum_vertical_down_mm:
            print(
                "[VERTICAL DEPTH ADJUST] "
                f"계산 하강량 {vertical_down_mm:.2f}mm가 작아 "
                f"최소 {minimum_vertical_down_mm:.2f}mm로 보정"
            )
            vertical_down_mm = minimum_vertical_down_mm

        if vertical_down_mm > maximum_vertical_down_mm:
            print(
                "[VERTICAL DEPTH LIMIT] "
                f"계산 하강량 {vertical_down_mm:.2f}mm를 "
                f"최대 {maximum_vertical_down_mm:.2f}mm로 제한"
            )
            vertical_down_mm = maximum_vertical_down_mm

        # 관절 충돌을 확인하기 위해 작은 단계로 이동합니다.
        vertical_step_mm = max(
            1.0,
            abs(
                float(
                    self.cfg.get(
                        "self_collision_guard_vertical_step_mm",
                        15.0
                    )
                )
            )
        )

        # 절대 바닥 좌표입니다.
        floor_contact_z_mm = float(
            self.cfg.get(
                "floor_contact_z_mm",
                0.0
            )
        )

        # 시작 TCP 좌표입니다.
        current_coords = [
            float(value)
            for value in gripper_center_coords[:6]
        ]

        target_z_override = self.cfg.get(
            "locked_vertical_target_z_mm",
            None
        )

        try:
            target_z_override = float(target_z_override)
        except (TypeError, ValueError):
            target_z_override = None

        if target_z_override is not None:
            target_z = max(
                floor_contact_z_mm,
                min(
                    float(current_coords[2]),
                    target_z_override
                )
            )
            vertical_down_mm = (
                float(current_coords[2])
                - target_z
            )
            print(
                "[VERTICAL TARGET Z] "
                f"locked_vertical_target_z_mm={target_z_override:.2f} | "
                f"down={vertical_down_mm:.2f}mm"
            )
        else:
            # 바닥 Z=0까지 하강을 허용합니다.
            target_z = max(
                floor_contact_z_mm,
                float(current_coords[2])
                - vertical_down_mm
            )

        actual_total_down_mm = (
            float(current_coords[2])
            - target_z
        )

        if bool(
            self.cfg.get(
                "joint2_assist_before_vertical_pick",
                True
            )
        ):
            assisted_coords = self.joint2_assisted_descent(
                target_z
            )

            if (
                isinstance(assisted_coords, list)
                and len(assisted_coords) >= 6
            ):
                current_coords = [
                    float(value)
                    for value in assisted_coords[:6]
                ]

                print(
                    "[LOCKED VERTICAL JOINT ASSIST] "
                    f"z_after_assist={float(current_coords[2]):.2f} | "
                    f"target_z={target_z:.2f}"
                )

                if float(current_coords[2]) <= target_z + 0.8:
                    print(
                        "[LOCKED VERTICAL JOINT ASSIST OK] "
                        "관절 보조 하강으로 목표 Z에 도달"
                    )
                    target_z = float(current_coords[2])

        actual_total_down_mm = (
            float(current_coords[2])
            - target_z
        )

        descent_start_z_for_free_pick = float(
            gripper_center_coords[2]
        )
        actual_joint_down_mm = (
            descent_start_z_for_free_pick
            - float(current_coords[2])
        )
        minimum_required_vertical_down_mm = float(
            self.cfg.get(
                "locked_vertical_min_required_down_mm",
                55.0
            )
        )

        if (
            bool(
                self.cfg.get(
                    "free_joint_descent_pick_enabled",
                    True
                )
            )
            and actual_joint_down_mm
            >= minimum_required_vertical_down_mm
        ):
            print(
                "[FREE JOINT PICK OK] "
                f"actual_down={actual_joint_down_mm:.2f}mm | "
                "X/Y 고정 Cartesian 하강 없이 현재 자세에서 집기"
            )

            self.cfg["last_vertical_pick_coords"] = [
                round(value, 3)
                for value in current_coords
            ]

            self.cfg["last_vertical_pick_calculation"] = {
                "camera_depth_mm": camera_depth_mm,
                "camera_to_gripper_z_mm": camera_to_gripper_z_mm,
                "target_clearance_mm": target_clearance_mm,
                "applied_down_mm": vertical_down_mm,
                "minimum_down_mm": minimum_vertical_down_mm,
                "maximum_down_mm": maximum_vertical_down_mm,
                "target_z_mm": target_z,
                "actual_down_mm": actual_joint_down_mm,
                "free_joint_descent_pick_enabled": True,
                "timestamp": round(time.time(), 3)
            }

            save_config(self.cfg)
            return current_coords

        # 하강 시작 관절 각도를 읽습니다.
        descent_start_angles = (
            self.get_joint_angles_for_safety()
        )

        # 관절값을 읽지 못하면 충돌 안전을 확인할 수 없으므로 시작하지 않습니다.
        if descent_start_angles is None:

            print(
                "[SELF COLLISION GUARD STOP] "
                "하강 시작 관절값을 읽지 못해 중단"
            )

            return None

        # 하강 시작 자세부터 안전한지 검사합니다.
        start_risk, start_reason = (
            self.check_self_collision_risk(
                descent_start_angles,
                previous_angles=None,
                descent_start_angles=descent_start_angles
            )
        )

        if start_risk:

            print(
                "[SELF COLLISION GUARD STOP] "
                "시작 자세 위험:",
                start_reason
            )

            return None

        previous_angles = descent_start_angles[:6]
        previous_safe_coords = current_coords[:6]
        descent_start_z = float(current_coords[2])
        vertical_iteration = 0
        maximum_vertical_iterations = int(
            self.cfg.get(
                "locked_vertical_max_iter",
                24
            )
        )

        print(
            "[LOCKED VERTICAL PICK FLOOR0] "
            f"fixed_xy=({current_coords[0]:.2f}, "
            f"{current_coords[1]:.2f}) | "
            f"z={current_coords[2]:.2f}"
            f"->{target_z:.2f} | "
            f"down={actual_total_down_mm:.2f}mm | "
            f"floor_z={floor_contact_z_mm:.2f}"
        )

        # 목표 Z까지 수직 하강합니다.
        while float(current_coords[2]) - target_z > 0.8:
            vertical_iteration += 1

            if vertical_iteration > maximum_vertical_iterations:
                print(
                    "[LOCKED VERTICAL STOP] "
                    "최대 수직 하강 반복 횟수 초과"
                )
                return None

            # 이동 전 관절 상태를 다시 확인합니다.
            before_angles = (
                self.get_joint_angles_for_safety()
            )

            if before_angles is None:

                print(
                    "[SELF COLLISION GUARD STOP] "
                    "이동 전 관절값 읽기 실패"
                )

                return None

            before_risk, before_reason = (
                self.check_self_collision_risk(
                    before_angles,
                    previous_angles=previous_angles,
                    descent_start_angles=descent_start_angles
                )
            )

            if before_risk:

                print(
                    "[SELF COLLISION GUARD STOP] "
                    "이동 전 위험:",
                    before_reason
                )

                return None

            # 다음 작은 하강 목표입니다.
            next_z = max(
                target_z,
                float(current_coords[2])
                - vertical_step_mm
            )

            next_coords = current_coords[:6]
            before_command_z = float(current_coords[2])

            # X/Y와 회전값은 고정하고 Z만 변경합니다.
            next_coords[2] = next_z

            print(
                "[LOCKED VERTICAL MOVE GUARDED] "
                f"{vertical_iteration}/{maximum_vertical_iterations} | "
                f"z={float(current_coords[2]):.2f}"
                f"->{next_z:.2f}"
            )

            moved = self.send_coords(
                next_coords,
                "locked_vertical_down_guarded",
                wait=float(
                    self.cfg.get(
                        "locked_vertical_wait_sec",
                        0.45
                    )
                )
            )

            if not moved:

                print(
                    "[LOCKED VERTICAL ERROR] "
                    "수직 하강 명령 실패"
                )

                return None

            time.sleep(
                float(
                    self.cfg.get(
                        "self_collision_guard_check_delay_sec",
                        0.08
                    )
                )
            )

            measured_after_coords = self.get_coords()

            if (
                isinstance(measured_after_coords, list)
                and len(measured_after_coords) >= 6
            ):
                measured_after_coords = [
                    float(value)
                    for value in measured_after_coords[:6]
                ]
                measured_step_down_mm = (
                    before_command_z
                    - float(measured_after_coords[2])
                )

                print(
                    "[LOCKED VERTICAL VERIFY] "
                    f"actual_step={measured_step_down_mm:.2f}mm | "
                    f"actual_z={float(measured_after_coords[2]):.2f}"
                )

                current_coords = measured_after_coords[:6]

                if measured_step_down_mm < float(
                    self.cfg.get(
                        "locked_vertical_min_actual_step_mm",
                        1.5
                    )
                ):
                    print(
                        "[LOCKED VERTICAL WARNING] "
                        "실제 Z 하강량이 작습니다. "
                        "다음 반복에서 다시 하강합니다."
                    )
            else:
                current_coords = next_coords[:6]

            # 이동 후 실제 관절 각도를 확인합니다.
            after_angles = (
                self.get_joint_angles_for_safety()
            )

            if after_angles is None:

                print(
                    "[SELF COLLISION GUARD] "
                    "이동 후 관절값 읽기 실패, 직전 좌표로 복귀"
                )

                self.send_coords(
                    previous_safe_coords,
                    "self_collision_guard_retreat",
                    wait=0.55
                )

                return None

            after_risk, after_reason = (
                self.check_self_collision_risk(
                    after_angles,
                    previous_angles=before_angles,
                    descent_start_angles=descent_start_angles
                )
            )

            # 충돌 위험 자세가 감지되면 즉시 직전 안전 좌표로 복귀합니다.
            if after_risk:

                print(
                    "[SELF COLLISION GUARD TRIGGERED] ",
                    after_reason
                )

                print(
                    "[SELF COLLISION GUARD RETREAT] "
                    "직전 안전 TCP 좌표로 복귀:",
                    [
                        round(value, 2)
                        for value in previous_safe_coords
                    ]
                )

                retreated = self.send_coords(
                    previous_safe_coords,
                    "self_collision_guard_retreat",
                    wait=float(
                        self.cfg.get(
                            "self_collision_guard_retreat_wait_sec",
                            0.65
                        )
                    )
                )

                print(
                    "[SELF COLLISION GUARD STOP] "
                    f"복귀 성공={retreated}, 집기 중단"
                )

                return None

            # 안전한 이동이 확인되면 현재 상태를 저장합니다.
            previous_safe_coords = current_coords[:6]
            previous_angles = after_angles[:6]

            time.sleep(
                float(
                    self.cfg.get(
                        "locked_vertical_settle_sec",
                        0.12
                    )
                )
            )

        # 최종 실제 좌표를 읽습니다.
        measured_final_coords = self.get_coords()

        if (
            isinstance(measured_final_coords, list)
            and len(measured_final_coords) >= 6
        ):

            current_coords = [
                float(value)
                for value in measured_final_coords[:6]
            ]

        actual_total_down_measured_mm = (
            descent_start_z
            - float(current_coords[2])
        )
        minimum_required_vertical_down_mm = float(
            self.cfg.get(
                "locked_vertical_min_required_down_mm",
                55.0
            )
        )

        if actual_total_down_measured_mm < minimum_required_vertical_down_mm:
            print(
                "[LOCKED VERTICAL ERROR] "
                f"실제 Z 하강 부족: {actual_total_down_measured_mm:.2f}mm "
                f"< {minimum_required_vertical_down_mm:.2f}mm"
            )
            return None

        # 최종 집기 좌표와 계산값을 저장합니다.
        self.cfg["last_vertical_pick_coords"] = [
            round(value, 3)
            for value in current_coords
        ]

        self.cfg["last_vertical_pick_calculation"] = {
            "camera_depth_mm": camera_depth_mm,
            "camera_to_gripper_z_mm": camera_to_gripper_z_mm,
            "target_clearance_mm": target_clearance_mm,
            "applied_down_mm": vertical_down_mm,
            "minimum_down_mm": minimum_vertical_down_mm,
            "maximum_down_mm": maximum_vertical_down_mm,
            "target_z_mm": target_z,
            "actual_down_mm": actual_total_down_measured_mm,
            "floor_contact_z_mm": floor_contact_z_mm,
            "self_collision_guard_enabled": True,
            "timestamp": round(
                time.time(),
                3
            )
        }

        save_config(
            self.cfg
        )

        print(
            "[SELF COLLISION GUARD OK] "
            "바닥 Z까지 수직 하강 완료"
        )

        return current_coords








    # 화면 확대 접근 -> 좌표 잠금 -> 그리퍼 중심 이동 -> 수직 집기를 실행합니다.
    # 첫 화면에서 컨테이너 중심·거리·현재 TCP 좌표를 저장합니다.
    def capture_initial_container_coordinate(self):
        # 최초 인식 시 잠긴 픽셀·깊이만 사용합니다.
        remembered = getattr(
            self,
            "last_recognized_detection",
            None
        )

        if not isinstance(remembered, dict):
            print("[INITIAL CAPTURE ERROR] 잠긴 최초 좌표가 없습니다.")
            return None

        center = remembered.get("center", None)

        if (
            not isinstance(center, (list, tuple))
            or len(center) < 2
        ):
            print("[INITIAL CAPTURE ERROR] 잠긴 픽셀 중심이 없습니다.")
            return None

        center_u = float(center[0])
        center_v = float(center[1])

        depth_mm = remembered.get(
            "depth_mm",
            self.cfg.get("initial_default_depth_mm", 210.0)
        )

        try:
            depth_mm = float(depth_mm)
        except (TypeError, ValueError):
            depth_mm = float(
                self.cfg.get("initial_default_depth_mm", 210.0)
            )

        frame_width = int(
            remembered.get(
                "frame_width",
                self.cfg.get("snapshot_frame_width", 640)
            )
        )
        frame_height = int(
            remembered.get(
                "frame_height",
                self.cfg.get("snapshot_frame_height", 480)
            )
        )

        target_u = float(
            self.cfg.get(
                "floor_camera_target_u",
                frame_width / 2.0
            )
        )
        target_v = float(
            self.cfg.get(
                "floor_camera_target_v",
                frame_height / 2.0
            )
        )

        error_u = center_u - target_u
        error_v = center_v - target_v

        focal_x_px = float(
            self.cfg.get("camera_fx_rect_px", 973.32886)
        )
        focal_y_px = float(
            self.cfg.get("camera_fy_rect_px", 989.13171)
        )

        camera_horizontal_mm = (
            error_u * depth_mm / max(1.0, focal_x_px)
        )
        camera_vertical_mm = (
            error_v * depth_mm / max(1.0, focal_y_px)
        )

        x_sign = float(
            self.cfg.get(
                "floor_image_y_to_robot_x_sign",
                1.0
            )
        )
        y_sign = float(
            self.cfg.get(
                "floor_image_x_to_robot_y_sign",
                -1.0
            )
        )
        initial_gain = float(
            self.cfg.get("initial_floor_xy_gain", 1.0)
        )

        delta_x = (
            x_sign
            * camera_vertical_mm
            * initial_gain
        )
        delta_y = (
            y_sign
            * camera_horizontal_mm
            * initial_gain
        )

        maximum_initial_xy_mm = abs(
            float(
                self.cfg.get(
                    "initial_floor_max_xy_mm",
                    80.0
                )
            )
        )

        delta_x = max(
            -maximum_initial_xy_mm,
            min(maximum_initial_xy_mm, delta_x)
        )
        delta_y = max(
            -maximum_initial_xy_mm,
            min(maximum_initial_xy_mm, delta_y)
        )

        current_coords = self.get_coords()

        if (
            not isinstance(current_coords, list)
            or len(current_coords) < 6
        ):
            print("[INITIAL CAPTURE ERROR] 현재 TCP 좌표 읽기 실패")
            return None

        current_coords = [
            float(value)
            for value in current_coords[:6]
        ]

        recognition_coords = remembered.get(
            "tcp_coords_at_recognition",
            None
        )

        recognition_base_coords = None

        if (
            isinstance(recognition_coords, list)
            and len(recognition_coords) >= 6
        ):
            recognition_base_coords = [
                float(value)
                for value in recognition_coords[:6]
            ]

            pose_shift_mm = (
                (
                    current_coords[0]
                    - recognition_base_coords[0]
                ) ** 2
                + (
                    current_coords[1]
                    - recognition_base_coords[1]
                ) ** 2
                + (
                    current_coords[2]
                    - recognition_base_coords[2]
                ) ** 2
            ) ** 0.5

            maximum_pose_shift_mm = abs(
                float(
                    self.cfg.get(
                        "first_target_pose_guard_mm",
                        10.0
                    )
                )
            )

            if pose_shift_mm > maximum_pose_shift_mm:
                print(
                    "[INITIAL CAPTURE ERROR] "
                    "첫 인식 후 로봇 위치가 변경되었습니다. "
                    f"shift={pose_shift_mm:.2f}mm"
                )
                print(
                    "[GUIDE] 물체를 화면에서 잠시 치웠다가 "
                    "다시 보여 새 좌표를 잠그세요."
                )
                return None

        absolute_base_coords = (
            recognition_base_coords
            if recognition_base_coords is not None
            else current_coords
        )

        target_camera_x_mm = (
            float(absolute_base_coords[0])
            + delta_x
        )
        target_camera_y_mm = (
            float(absolute_base_coords[1])
            + delta_y
        )
        estimated_container_surface_z_mm = (
            float(absolute_base_coords[2])
            - depth_mm
        )

        initial_target = {
            "center_u_px": center_u,
            "center_v_px": center_v,
            "depth_mm": depth_mm,
            "delta_x_mm": delta_x,
            "delta_y_mm": delta_y,
            "start_coords": current_coords,
            "recognition_coords": absolute_base_coords,
            "target_camera_x_mm": target_camera_x_mm,
            "target_camera_y_mm": target_camera_y_mm,
            "estimated_container_surface_z_mm": (
                estimated_container_surface_z_mm
            ),
            "frame_width": frame_width,
            "frame_height": frame_height,
            "recognized_at": remembered.get("recognized_at", None),
        }

        self.cfg["last_initial_container_target"] = {
            "center_u_px": round(center_u, 3),
            "center_v_px": round(center_v, 3),
            "depth_mm": round(depth_mm, 3),
            "delta_x_mm": round(delta_x, 3),
            "delta_y_mm": round(delta_y, 3),
            "start_coords": [
                round(value, 3)
                for value in current_coords
            ],
            "recognition_coords": [
                round(value, 3)
                for value in absolute_base_coords
            ],
            "target_camera_x_mm": round(
                target_camera_x_mm,
                3
            ),
            "target_camera_y_mm": round(
                target_camera_y_mm,
                3
            ),
            "estimated_container_surface_z_mm": round(
                estimated_container_surface_z_mm,
                3
            ),
            "timestamp": round(time.time(), 3),
        }

        save_config(self.cfg)

        print(
            "[FLOW 1/5] 최초 좌표 잠금 완료 | "
            f"pixel=({center_u:.1f}, {center_v:.1f}) | "
            f"depth={depth_mm:.1f}mm | "
            f"xy_delta=({delta_x:+.2f}, {delta_y:+.2f})mm | "
            f"target_xy=({target_camera_x_mm:.2f}, "
            f"{target_camera_y_mm:.2f})"
        )

        return initial_target



    # 첫 화면 좌표 근처로 이동하면서 카메라를 바닥 방향으로 전환합니다.
    def move_to_initial_floor_target(self, initial_target):
        # 1) 카메라 바닥 보기, 2) 잠긴 좌표로 X/Y 이동 순서로 실행합니다.
        delta_x = float(
            initial_target.get("delta_x_mm", 0.0)
        )
        delta_y = float(
            initial_target.get("delta_y_mm", 0.0)
        )

        floor_angles = self.cfg.get(
            "floor_camera_angles",
            None
        )
        floor_rpy = self.cfg.get(
            "floor_camera_rpy",
            None
        )

        if (
            isinstance(floor_angles, list)
            and len(floor_angles) >= 6
        ):
            print(
                "[FLOW 2/5] 카메라 바닥 보기 "
                "(6관절 동시 이동)"
            )

            response = self.arm.send_angles(
                [
                    float(value)
                    for value in floor_angles[:6]
                ],
                int(
                    self.cfg.get(
                        "floor_orientation_speed",
                        55
                    )
                ),
                wait=float(
                    self.cfg.get(
                        "floor_orientation_wait_sec",
                        1.0
                    )
                )
            )

            if (
                not isinstance(response, dict)
                or not response.get("ok", False)
            ):
                print("[FLOOR ORIENTATION ERROR]", response)
                return None

            self.arm.reset_tcp_cache()

        elif (
            isinstance(floor_rpy, list)
            and len(floor_rpy) >= 3
        ):
            print(
                "[FLOOR ORIENTATION WARNING] "
                "floor_camera_angles가 없어 RPY fallback 사용"
            )
            print(
                "[GUIDE] 바닥 보기 자세에서 g를 누르면 "
                "다음부터 6관절 동시 이동을 사용합니다."
            )

            current_coords = self.get_coords()

            if (
                not isinstance(current_coords, list)
                or len(current_coords) < 6
            ):
                return None

            orientation_coords = [
                float(value)
                for value in current_coords[:6]
            ]
            orientation_coords[3] = float(floor_rpy[0])
            orientation_coords[4] = float(floor_rpy[1])
            orientation_coords[5] = float(floor_rpy[2])

            if not self.send_coords(
                orientation_coords,
                "camera_face_floor_fallback",
                wait=float(
                    self.cfg.get(
                        "floor_orientation_wait_sec",
                        1.0
                    )
                )
            ):
                return None

        else:
            print(
                "[FLOOR ORIENTATION ERROR] "
                "바닥 보기 자세가 저장되지 않았습니다."
            )
            print(
                "[GUIDE] 카메라를 바닥으로 맞춘 뒤 g를 누르세요."
            )
            return None

        time.sleep(
            float(
                self.cfg.get(
                    "floor_camera_settle_sec",
                    0.15
                )
            )
        )

        oriented_coords = self.get_coords()

        if (
            not isinstance(oriented_coords, list)
            or len(oriented_coords) < 6
        ):
            print(
                "[FLOOR ORIENTATION ERROR] "
                "자세 전환 후 TCP 좌표 읽기 실패"
            )
            return None

        oriented_coords = [
            float(value)
            for value in oriented_coords[:6]
        ]

        xy_target_coords = oriented_coords[:6]

        target_camera_x_mm = initial_target.get(
            "target_camera_x_mm",
            None
        )
        target_camera_y_mm = initial_target.get(
            "target_camera_y_mm",
            None
        )

        try:
            target_camera_x_mm = float(target_camera_x_mm)
            target_camera_y_mm = float(target_camera_y_mm)
        except (TypeError, ValueError):
            target_camera_x_mm = None
            target_camera_y_mm = None

        if (
            target_camera_x_mm is not None
            and target_camera_y_mm is not None
            and bool(
                self.cfg.get(
                    "initial_floor_use_absolute_xy",
                    True
                )
            )
        ):
            xy_target_coords[0] = target_camera_x_mm
            xy_target_coords[1] = target_camera_y_mm
            move_mode_text = "absolute"

        else:
            xy_target_coords[0] += delta_x
            xy_target_coords[1] += delta_y
            move_mode_text = "relative"

        print(
            "[FLOW 3/5] 최초 잠금 좌표로 X/Y 동시 이동 | "
            f"mode={move_mode_text} | "
            f"dx={delta_x:+.2f}mm, "
            f"dy={delta_y:+.2f}mm | "
            f"target=({xy_target_coords[0]:.2f}, "
            f"{xy_target_coords[1]:.2f})"
        )

        moved = self.send_coords(
            xy_target_coords,
            "first_locked_xy_move",
            wait=float(
                self.cfg.get(
                    "floor_xy_move_wait_sec",
                    0.65
                )
            )
        )

        if not moved:
            print("[INITIAL XY MOVE ERROR] 이동 실패")
            return None

        time.sleep(
            float(
                self.cfg.get(
                    "initial_floor_settle_sec",
                    0.18
                )
            )
        )

        measured_coords = self.get_coords()

        if (
            isinstance(measured_coords, list)
            and len(measured_coords) >= 6
        ):
            xy_target_coords = [
                float(value)
                for value in measured_coords[:6]
            ]

        print(
            "[INITIAL XY MOVE OK] "
            f"{[round(value, 2) for value in xy_target_coords]}"
        )

        return xy_target_coords



    # 바닥 카메라 정렬용 검출값을 여러 프레임 중앙값으로 안정화합니다.
    def get_stable_floor_detection(self):
        sample_count = max(
            1,
            int(
                self.cfg.get(
                    "floor_detection_sample_count",
                    3
                )
            )
        )
        sample_interval_sec = max(
            0.0,
            float(
                self.cfg.get(
                    "floor_detection_sample_interval_sec",
                    0.035
                )
            )
        )

        detections = []
        frame_shape = None

        for _ in range(sample_count):
            frame = self.read_frame()

            if frame is None:
                time.sleep(sample_interval_sec)
                continue

            detection, _ = self.detect_rectangle_object(frame)

            if isinstance(detection, dict):
                detections.append(dict(detection))
                frame_shape = frame.shape[:2]

            if sample_interval_sec > 0.0:
                time.sleep(sample_interval_sec)

        if not detections:
            return None, None

        centers_u = []
        centers_v = []
        depths = []

        for detection in detections:
            center = detection.get("center", None)

            if (
                isinstance(center, (list, tuple))
                and len(center) >= 2
            ):
                centers_u.append(float(center[0]))
                centers_v.append(float(center[1]))

            depth_value = detection.get("depth_mm", None)

            try:
                depth_value = float(depth_value)
            except (TypeError, ValueError):
                depth_value = None

            if depth_value is not None and depth_value > 0.0:
                depths.append(depth_value)

        stable_detection = dict(detections[-1])

        if centers_u and centers_v:
            stable_detection["center"] = (
                float(np.median(centers_u)),
                float(np.median(centers_v))
            )

        if depths:
            stable_detection["depth_mm"] = float(
                np.median(depths)
            )

        return stable_detection, frame_shape



    # 바닥 보기 카메라로 확대하면서 컨테이너 중심을 X/Y로 계속 맞춥니다.
    def floor_visual_approach_and_center(
        self,
        initial_target,
        floor_start_coords
    ):
        # Z와 RPY를 고정하고 카메라 영상으로 X/Y만 정밀 보정합니다.
        maximum_iterations = int(
            self.cfg.get("floor_center_only_max_iter", 12)
        )
        center_tolerance_px = abs(
            float(
                self.cfg.get(
                    "floor_center_tolerance_px",
                    12.0
                )
            )
        )
        acceptance_px = abs(
            float(
            self.cfg.get("floor_center_accept_px", 18.0)
            )
        )
        maximum_xy_correction_mm = abs(
            float(
                self.cfg.get(
                    "floor_center_max_step_mm",
                    8.0
                )
            )
        )
        minimum_xy_step_mm = abs(
            float(
                self.cfg.get(
                    "floor_center_min_step_mm",
                    0.6
                )
            )
        )
        center_gain = float(
            self.cfg.get("floor_center_gain", 0.75)
        )
        maximum_missing_frames = int(
            self.cfg.get("floor_missing_frame_limit", 5)
        )

        current_coords = [
            float(value)
            for value in floor_start_coords[:6]
        ]
        locked_z = float(current_coords[2])
        locked_rpy = [
            float(current_coords[3]),
            float(current_coords[4]),
            float(current_coords[5]),
        ]

        missing_frames = 0
        last_detection = None
        last_frame_shape = None
        last_error_distance = None

        print(
            "[FLOW 4/5] 카메라-컨테이너 X/Y 정밀 정렬 | "
            f"z_fixed={locked_z:.2f}"
        )

        for iteration in range(
            1,
            maximum_iterations + 1
        ):
            detection, frame_shape = self.get_stable_floor_detection()

            if not isinstance(detection, dict):
                missing_frames += 1
                print(
                    "[FLOOR XY LOST] "
                    f"{missing_frames}/{maximum_missing_frames}"
                )

                if missing_frames >= maximum_missing_frames:
                    return None

                time.sleep(0.12)
                continue

            missing_frames = 0
            last_detection = dict(detection)
            last_frame_shape = frame_shape

            if (
                isinstance(frame_shape, (list, tuple))
                and len(frame_shape) >= 2
            ):
                frame_height = int(frame_shape[0])
                frame_width = int(frame_shape[1])
            else:
                frame_height = int(
                    initial_target.get(
                        "frame_height",
                        480
                    )
                )
                frame_width = int(
                    initial_target.get(
                        "frame_width",
                        640
                    )
                )

            center_u, center_v = detection.get(
                "center",
                (
                    frame_width / 2.0,
                    frame_height / 2.0
                )
            )

            target_u = float(
                self.cfg.get(
                    "floor_camera_target_u",
                    frame_width / 2.0
                )
            )
            target_v = float(
                self.cfg.get(
                    "floor_camera_target_v",
                    frame_height / 2.0
                )
            )

            error_u = float(center_u) - target_u
            error_v = float(center_v) - target_v
            error_distance = (
                error_u ** 2 + error_v ** 2
            ) ** 0.5

            print(
                "[FLOOR XY] "
                f"{iteration}/{maximum_iterations} | "
                f"error=({error_u:+.1f}, "
                f"{error_v:+.1f})px"
            )

            if (
                abs(error_u) <= center_tolerance_px
                and abs(error_v) <= center_tolerance_px
            ):
                print("[FLOOR XY LOCK] 중심 정렬 완료")
                break

            if (
                last_error_distance is not None
                and error_distance
                > last_error_distance * 1.45
            ):
                print(
                    "[FLOOR XY STOP] 오차가 증가했습니다. "
                    "floor_image_*_sign을 확인하세요."
                )
                return None

            last_error_distance = error_distance

            depth_mm = detection.get(
                "depth_mm",
                initial_target.get("depth_mm", 210.0)
            )

            try:
                depth_mm = float(depth_mm)
            except (TypeError, ValueError):
                depth_mm = float(
                    initial_target.get("depth_mm", 210.0)
                )

            focal_x_px = float(
                self.cfg.get("camera_fx_rect_px", 973.32886)
            )
            focal_y_px = float(
                self.cfg.get("camera_fy_rect_px", 989.13171)
            )

            correction_x = (
                float(
                    self.cfg.get(
                        "floor_image_y_to_robot_x_sign",
                        1.0
                    )
                )
                * error_v
                * depth_mm
                / max(1.0, focal_y_px)
                * center_gain
            )
            correction_y = (
                float(
                    self.cfg.get(
                        "floor_image_x_to_robot_y_sign",
                        -1.0
                    )
                )
                * error_u
                * depth_mm
                / max(1.0, focal_x_px)
                * center_gain
            )

            correction_x = max(
                -maximum_xy_correction_mm,
                min(maximum_xy_correction_mm, correction_x)
            )
            correction_y = max(
                -maximum_xy_correction_mm,
                min(maximum_xy_correction_mm, correction_y)
            )

            if (
                abs(error_v) > center_tolerance_px
                and 0.0 < abs(correction_x)
                < minimum_xy_step_mm
            ):
                correction_x = (
                    minimum_xy_step_mm
                    if correction_x > 0.0
                    else -minimum_xy_step_mm
                )

            if (
                abs(error_u) > center_tolerance_px
                and 0.0 < abs(correction_y)
                < minimum_xy_step_mm
            ):
                correction_y = (
                    minimum_xy_step_mm
                    if correction_y > 0.0
                    else -minimum_xy_step_mm
                )

            measured_coords = self.get_coords()

            if (
                isinstance(measured_coords, list)
                and len(measured_coords) >= 6
            ):
                current_coords = [
                    float(value)
                    for value in measured_coords[:6]
                ]

            next_coords = current_coords[:6]
            next_coords[0] += correction_x
            next_coords[1] += correction_y
            next_coords[2] = locked_z
            next_coords[3] = locked_rpy[0]
            next_coords[4] = locked_rpy[1]
            next_coords[5] = locked_rpy[2]

            print(
                "[FLOOR XY MOVE] "
                f"dx={correction_x:+.2f}mm, "
                f"dy={correction_y:+.2f}mm | "
                f"z={locked_z:.2f} 고정"
            )

            moved = self.send_coords(
                next_coords,
                "floor_xy_only",
                wait=float(
                    self.cfg.get(
                        "floor_visual_move_wait_sec",
                        0.22
                    )
                )
            )

            if not moved:
                return None

            current_coords = next_coords

            time.sleep(
                float(
                    self.cfg.get(
                        "floor_visual_settle_sec",
                        0.08
                    )
                )
            )

        if not isinstance(last_detection, dict):
            return None

        final_center = last_detection.get("center", None)

        if (
            isinstance(final_center, (list, tuple))
            and len(final_center) >= 2
            and last_frame_shape is not None
        ):
            frame_height, frame_width = last_frame_shape
            final_target_u = float(
                self.cfg.get(
                    "floor_camera_target_u",
                    frame_width / 2.0
                )
            )
            final_target_v = float(
                self.cfg.get(
                    "floor_camera_target_v",
                    frame_height / 2.0
                )
            )

            final_error_distance = (
                (
                    float(final_center[0])
                    - final_target_u
                ) ** 2
                + (
                    float(final_center[1])
                    - final_target_v
                ) ** 2
            ) ** 0.5

            if final_error_distance > acceptance_px:
                print(
                    "[FLOOR XY ERROR] 최종 중심 오차가 큽니다: "
                    f"{final_error_distance:.1f}px"
                )
                return None

        measured_coords = self.get_coords()

        if (
            isinstance(measured_coords, list)
            and len(measured_coords) >= 6
        ):
            current_coords = [
                float(value)
                for value in measured_coords[:6]
            ]

        current_coords[2] = locked_z
        current_coords[3] = locked_rpy[0]
        current_coords[4] = locked_rpy[1]
        current_coords[5] = locked_rpy[2]

        self.cfg["last_floor_visual_lock"] = {
            "center": last_detection.get("center", None),
            "depth_mm": last_detection.get("depth_mm", None),
            "tcp_coords": [
                round(value, 3)
                for value in current_coords
            ],
            "z_was_locked": True,
            "timestamp": round(time.time(), 3),
        }

        save_config(self.cfg)

        return {
            "lock_coords": current_coords,
            "detection": last_detection,
            "frame_shape": last_frame_shape,
        }



        # 확대 후 컨테이너 중심을 그리퍼 중심 X/Y에 맞춥니다.
    def align_gripper_to_floor_locked_container(
        self,
        floor_lock_result
    ):

        lock_coords = floor_lock_result.get(
            "lock_coords",
            None
        )

        detection = floor_lock_result.get(
            "detection",
            None
        )

        frame_shape = floor_lock_result.get(
            "frame_shape",
            None
        )

        if (
            not isinstance(lock_coords, list)
            or len(lock_coords) < 6
            or not isinstance(detection, dict)
        ):

            print(
                "[GRIPPER ALIGN ERROR] "
                "최종 잠금 좌표가 없습니다."
            )

            return None

        if (
            isinstance(frame_shape, (list, tuple))
            and len(frame_shape) >= 2
        ):

            frame_height = int(frame_shape[0])
            frame_width = int(frame_shape[1])

        else:

            frame_width = int(
                self.cfg.get(
                    "snapshot_frame_width",
                    640
                )
            )

            frame_height = int(
                self.cfg.get(
                    "snapshot_frame_height",
                    480
                )
            )

        center_u, center_v = detection.get(
            "center",
            (
                frame_width / 2.0,
                frame_height / 2.0
            )
        )

        center_u = float(center_u)
        center_v = float(center_v)

        target_u = float(
            self.cfg.get(
                "floor_camera_target_u",
                frame_width / 2.0
            )
        )

        target_v = float(
            self.cfg.get(
                "floor_camera_target_v",
                frame_height / 2.0
            )
        )

        error_u = center_u - target_u
        error_v = center_v - target_v

        depth_mm = detection.get(
            "depth_mm",
            self.cfg.get(
                "initial_default_depth_mm",
                210.0
            )
        )

        try:

            depth_mm = float(depth_mm)

        except (TypeError, ValueError):

            depth_mm = float(
                self.cfg.get(
                    "initial_default_depth_mm",
                    210.0
                )
            )

        focal_x_px = float(
            self.cfg.get(
                "camera_fx_rect_px",
                973.32886
            )
        )

        focal_y_px = float(
            self.cfg.get(
                "camera_fy_rect_px",
                989.13171
            )
        )

        # 남은 픽셀 오차를 마지막 X/Y 보정값으로 변환합니다.
        residual_x_mm = (
            float(
                self.cfg.get(
                    "floor_image_y_to_robot_x_sign",
                    1.0
                )
            )
            * error_v
            * depth_mm
            / max(1.0, focal_y_px)
        )

        residual_y_mm = (
            float(
                self.cfg.get(
                    "floor_image_x_to_robot_y_sign",
                    -1.0
                )
            )
            * error_u
            * depth_mm
            / max(1.0, focal_x_px)
        )

        # 카메라 중심과 그리퍼 중심의 고정 오프셋입니다.
        camera_to_gripper_x_mm = float(
            self.cfg.get(
                "camera_to_gripper_x_mm",
                20.0
            )
        )

        camera_to_gripper_y_mm = float(
            self.cfg.get(
                "camera_to_gripper_y_mm",
                10.0
            )
        )

        final_gripper_offset_x_mm = float(
            self.cfg.get(
                "final_gripper_offset_x_mm",
                0.0
            )
        )

        final_gripper_offset_y_mm = float(
            self.cfg.get(
                "final_gripper_offset_y_mm",
                120.0
            )
        )

        gripper_coords = [
            float(value)
            for value in lock_coords[:6]
        ]

        gripper_coords[0] += (
            residual_x_mm
            + camera_to_gripper_x_mm
            + final_gripper_offset_x_mm
        )

        gripper_coords[1] += (
            residual_y_mm
            + camera_to_gripper_y_mm
            + final_gripper_offset_y_mm
        )

        print(
            "[GRIPPER XY ALIGN] "
            f"residual=({residual_x_mm:+.2f}, "
            f"{residual_y_mm:+.2f})mm | "
            f"offset=({camera_to_gripper_x_mm:+.2f}, "
            f"{camera_to_gripper_y_mm:+.2f})mm | "
            f"final=({final_gripper_offset_x_mm:+.2f}, "
            f"{final_gripper_offset_y_mm:+.2f})mm"
        )

        moved = self.send_coords(
            gripper_coords,
            "gripper_xy_from_floor_lock",
            wait=float(
                self.cfg.get(
                    "gripper_xy_align_wait_sec",
                    0.35
                )
            )
        )

        if not moved:

            print(
                "[GRIPPER ALIGN ERROR] "
                "그리퍼 X/Y 정렬 이동 실패"
            )

            return None

        time.sleep(
            float(
                self.cfg.get(
                    "gripper_xy_align_settle_sec",
                    0.12
                )
            )
        )

        measured_coords = self.get_coords()

        if (
            isinstance(measured_coords, list)
            and len(measured_coords) >= 6
        ):

            gripper_coords = [
                float(value)
                for value in measured_coords[:6]
            ]

        self.cfg["last_gripper_xy_alignment"] = {
            "residual_x_mm": round(
                residual_x_mm,
                3
            ),
            "residual_y_mm": round(
                residual_y_mm,
                3
            ),
            "camera_to_gripper_x_mm": round(
                camera_to_gripper_x_mm,
                3
            ),
            "camera_to_gripper_y_mm": round(
                camera_to_gripper_y_mm,
                3
            ),
            "final_gripper_offset_x_mm": round(
                final_gripper_offset_x_mm,
                3
            ),
            "final_gripper_offset_y_mm": round(
                final_gripper_offset_y_mm,
                3
            ),
            "target_coords": [
                round(value, 3)
                for value in gripper_coords
            ],
            "timestamp": round(
                time.time(),
                3
            )
        }

        save_config(
            self.cfg
        )

        return gripper_coords


    # 직접 집기 목표 좌표를 계산합니다. 실제 이동 전 보정 저장에도 사용합니다.
    def calculate_direct_xyz_pick_target(
        self,
        current_coords,
        detection,
        frame_shape,
        apply_manual_calibration=True
    ):

        if (
            not isinstance(current_coords, list)
            or len(current_coords) < 6
        ):
            return None

        if not isinstance(detection, dict):
            return None

        current_coords = [
            float(value)
            for value in current_coords[:6]
        ]

        frame_height, frame_width = frame_shape[:2]
        center_u, center_v = detection.get(
            "center",
            (
                frame_width / 2.0,
                frame_height / 2.0
            )
        )

        target_u_value = self.cfg.get(
            "direct_pick_target_u",
            None
        )
        if target_u_value is None:
            target_u_value = self.cfg.get(
                "target_u",
                None
            )
        if target_u_value is None:
            target_u_value = frame_width / 2.0

        target_v_value = self.cfg.get(
            "direct_pick_target_v",
            None
        )
        if target_v_value is None:
            target_v_value = self.cfg.get(
                "target_v",
                None
            )
        if target_v_value is None:
            target_v_value = frame_height / 2.0

        target_u = float(target_u_value)
        target_v = float(target_v_value)

        depth_mm = detection.get(
            "depth_mm",
            self.cfg.get(
                "initial_default_depth_mm",
                210.0
            )
        )

        try:
            depth_mm = float(depth_mm)
        except (TypeError, ValueError):
            depth_mm = float(
                self.cfg.get(
                    "initial_default_depth_mm",
                    210.0
                )
            )

        error_u = float(center_u) - target_u
        error_v = float(center_v) - target_v

        focal_x_px = float(
            self.cfg.get("camera_fx_rect_px", 973.32886)
        )
        focal_y_px = float(
            self.cfg.get("camera_fy_rect_px", 989.13171)
        )

        xy_gain = float(
            self.cfg.get(
                "direct_pick_xy_gain",
                1.0
            )
        )

        delta_x = (
            float(
                self.cfg.get(
                    "direct_pick_x_sign",
                    -1.0
                )
            )
            * error_v
            * depth_mm
            / max(1.0, focal_y_px)
            * xy_gain
        )
        delta_y = (
            float(
                self.cfg.get(
                    "direct_pick_y_sign",
                    1.0
                )
            )
            * error_u
            * depth_mm
            / max(1.0, focal_x_px)
            * xy_gain
        )

        maximum_xy_mm = abs(
            float(
                self.cfg.get(
                    "direct_pick_max_xy_mm",
                    180.0
                )
            )
        )
        delta_x = max(
            -maximum_xy_mm,
            min(maximum_xy_mm, delta_x)
        )
        delta_y = max(
            -maximum_xy_mm,
            min(maximum_xy_mm, delta_y)
        )

        camera_to_gripper_x_mm = float(
            self.cfg.get("camera_to_gripper_x_mm", 20.0)
        )
        camera_to_gripper_y_mm = float(
            self.cfg.get("camera_to_gripper_y_mm", 35.0)
        )
        final_offset_x_mm = float(
            self.cfg.get("final_gripper_offset_x_mm", 0.0)
        )
        final_offset_y_mm = float(
            self.cfg.get("final_gripper_offset_y_mm", 0.0)
        )

        z_down_mm = max(
            0.0,
            float(
                self.cfg.get(
                    "direct_pick_down_mm",
                    self.cfg.get(
                        "locked_vertical_min_down_mm",
                        105.0
                    )
                )
            )
        )

        if bool(
            self.cfg.get(
                "direct_pick_use_depth_z",
                True
            )
        ):
            camera_to_gripper_z_mm = abs(
                float(
                    self.cfg.get(
                        "camera_to_gripper_z_mm",
                        60.0
                    )
                )
            )
            z_clearance_mm = float(
                self.cfg.get(
                    "direct_pick_z_clearance_mm",
                    20.0
                )
            )
            depth_based_down_mm = (
                depth_mm
                - camera_to_gripper_z_mm
                - z_clearance_mm
            )
            z_down_mm = max(
                0.0,
                depth_based_down_mm
            )

        z_down_mm += float(
            self.cfg.get(
                "direct_pick_extra_down_mm",
                0.0
            )
        )

        minimum_z_down_mm = abs(
            float(
                self.cfg.get(
                    "direct_pick_min_down_mm",
                    65.0
                )
            )
        )
        maximum_z_down_mm = abs(
            float(
                self.cfg.get(
                    "direct_pick_max_down_mm",
                    105.0
                )
            )
        )
        z_down_mm = max(
            minimum_z_down_mm,
            min(maximum_z_down_mm, z_down_mm)
        )

        move_x_mm = (
            delta_x
            + camera_to_gripper_x_mm
            + final_offset_x_mm
        )
        move_y_mm = (
            delta_y
            + camera_to_gripper_y_mm
            + final_offset_y_mm
        )

        if bool(
            self.cfg.get(
                "direct_pick_invert_total_xy",
                True
            )
        ):
            move_x_mm = -move_x_mm
            move_y_mm = -move_y_mm

        visual_move_x_mm = move_x_mm
        visual_move_y_mm = move_y_mm
        visual_move_z_mm = -z_down_mm

        maximum_total_xy_mm = abs(
            float(
                self.cfg.get(
                    "direct_pick_max_total_xy_mm",
                    90.0
                )
            )
        )
        move_x_mm = max(
            -maximum_total_xy_mm,
            min(maximum_total_xy_mm, move_x_mm)
        )
        move_y_mm = max(
            -maximum_total_xy_mm,
            min(maximum_total_xy_mm, move_y_mm)
        )

        target_coords = current_coords[:6]
        target_coords[0] += move_x_mm
        target_coords[1] += move_y_mm
        target_coords[2] = max(
            float(
                self.cfg.get(
                    "floor_contact_z_mm",
                    0.0
                )
            ),
            float(current_coords[2]) - z_down_mm
        )

        base_target_coords = target_coords[:6]

        manual_offset_x_mm = 0.0
        manual_offset_y_mm = 0.0
        manual_offset_z_mm = 0.0
        calibration_source = "none"

        if (
            apply_manual_calibration
            and bool(
                self.cfg.get(
                    "direct_pick_use_relative_manual_calibration",
                    True
                )
            )
        ):
            calibration = self.cfg.get(
                "last_direct_pick_manual_calibration",
                None
            )

            if isinstance(calibration, dict):
                manual_offset_x_mm = float(
                    calibration.get(
                        "correction_x_mm",
                        self.cfg.get(
                            "direct_pick_manual_offset_x_mm",
                            0.0
                        )
                    )
                )
                manual_offset_y_mm = float(
                    calibration.get(
                        "correction_y_mm",
                        self.cfg.get(
                            "direct_pick_manual_offset_y_mm",
                            0.0
                        )
                    )
                )
                manual_offset_z_mm = float(
                    calibration.get(
                        "correction_z_mm",
                        self.cfg.get(
                            "direct_pick_manual_offset_z_mm",
                            0.0
                        )
                    )
                )
                calibration_source = "manual_correction_offset"

        elif apply_manual_calibration:
            manual_offset_x_mm = float(
                self.cfg.get(
                    "direct_pick_final_robot_x_mm",
                    0.0
                )
            )
            manual_offset_y_mm = float(
                self.cfg.get(
                    "direct_pick_final_robot_y_mm",
                    0.0
                )
            )
            manual_offset_z_mm = -max(
                0.0,
                float(
                    self.cfg.get(
                        "direct_pick_final_extra_down_mm",
                        0.0
                    )
                )
            )
            calibration_source = "legacy_final_offset"

        if calibration_source in (
            "none",
            "legacy_final_offset",
            "last_manual_relative",
            "manual_correction_offset"
        ):
            target_coords[0] += manual_offset_x_mm
            target_coords[1] += manual_offset_y_mm
            target_coords[2] = max(
                float(
                    self.cfg.get(
                        "floor_contact_z_mm",
                        0.0
                    )
                ),
                target_coords[2] + manual_offset_z_mm
            )

        return {
            "target_coords": target_coords,
            "center_u": float(center_u),
            "center_v": float(center_v),
            "depth_mm": depth_mm,
            "move_x_mm": move_x_mm,
            "move_y_mm": move_y_mm,
            "move_z_mm": target_coords[2] - current_coords[2],
            "visual_move_x_mm": visual_move_x_mm,
            "visual_move_y_mm": visual_move_y_mm,
            "visual_move_z_mm": visual_move_z_mm,
            "base_target_coords": base_target_coords,
            "manual_offset_x_mm": manual_offset_x_mm,
            "manual_offset_y_mm": manual_offset_y_mm,
            "manual_offset_z_mm": manual_offset_z_mm,
            "calibration_source": calibration_source,
            "start_coords": current_coords,
            "frame_width": int(frame_width),
            "frame_height": int(frame_height),
        }


    # 현재 카메라가 보는 컨테이너를 기준으로 X/Y/Z를 한 번에 이동해 집습니다.
    def direct_xyz_pick_and_return_home(self):

        # w를 여러 번 실행할 때 이전 이동 좌표 캐시가 섞이지 않게 합니다.
        self.arm.reset_tcp_cache()

        frame = self.read_frame()

        if frame is None:
            print("[DIRECT XYZ PICK ERROR] 카메라 영상 없음")
            return False

        detection, _ = self.detect_rectangle_object(frame)

        if (
            not isinstance(detection, dict)
            and not bool(
                self.cfg.get(
                    "direct_pick_require_fresh_detection",
                    True
                )
            )
        ):
            detection = getattr(
                self,
                "last_recognized_detection",
                None
            )

        if not isinstance(detection, dict):
            print(
                "[DIRECT XYZ PICK ERROR] "
                "현재 화면에서 컨테이너를 새로 인식하지 못했습니다."
            )
            return False

        current_coords = self.get_coords()

        if (
            not isinstance(current_coords, list)
            or len(current_coords) < 6
        ):
            print("[DIRECT XYZ PICK ERROR] 현재 TCP 좌표 읽기 실패")
            return False

        current_coords = [
            float(value)
            for value in current_coords[:6]
        ]

        if bool(
            self.cfg.get(
                "direct_pick_open_gripper_before_move",
                True
            )
        ):
            if not self.open_gripper():
                print("[DIRECT XYZ PICK ERROR] 시작 전 그리퍼 열기 실패")
                return False

            self.arm.reset_tcp_cache()

            refreshed_coords = self.get_coords()

            if (
                isinstance(refreshed_coords, list)
                and len(refreshed_coords) >= 6
            ):
                current_coords = [
                    float(value)
                    for value in refreshed_coords[:6]
                ]

        pick_plan = self.calculate_direct_xyz_pick_target(
            current_coords,
            detection,
            frame.shape,
            apply_manual_calibration=True
        )

        if not isinstance(pick_plan, dict):
            print("[DIRECT XYZ PICK ERROR] 목표 좌표 계산 실패")
            return False

        target_coords = pick_plan["target_coords"]
        center_u = pick_plan["center_u"]
        center_v = pick_plan["center_v"]
        depth_mm = pick_plan["depth_mm"]
        move_x_mm = pick_plan["move_x_mm"]
        move_y_mm = pick_plan["move_y_mm"]
        base_target_coords = pick_plan["base_target_coords"]
        manual_offset_x_mm = pick_plan["manual_offset_x_mm"]
        manual_offset_y_mm = pick_plan["manual_offset_y_mm"]
        manual_offset_z_mm = pick_plan["manual_offset_z_mm"]
        calibration_source = pick_plan["calibration_source"]

        print(
            "[DIRECT XYZ PICK] "
            f"pixel=({float(center_u):.1f}, {float(center_v):.1f}) | "
            f"depth={depth_mm:.1f} | "
            f"move=({move_x_mm:+.2f}, "
            f"{move_y_mm:+.2f}, "
            f"{target_coords[2] - current_coords[2]:+.2f})"
        )
        print(
            "[DIRECT XYZ PICK CALIB] "
            f"source={calibration_source} | "
            f"base=({base_target_coords[0]:.2f}, "
            f"{base_target_coords[1]:.2f}, "
            f"{base_target_coords[2]:.2f}) | "
            f"delta=({manual_offset_x_mm:+.2f}, "
            f"{manual_offset_y_mm:+.2f}, "
            f"{manual_offset_z_mm:+.2f})"
        )

        xy_first_clearance_mm = abs(
            float(
                self.cfg.get(
                    "direct_pick_xy_first_clearance_mm",
                    15.0
                )
            )
        )
        xy_first_coords = target_coords[:6]
        xy_first_coords[2] = max(
            float(current_coords[2]),
            float(target_coords[2]) + xy_first_clearance_mm
        )

        print(
            "[DIRECT XYZ PICK] X/Y 먼저 이동 | "
            f"target=({xy_first_coords[0]:.2f}, "
            f"{xy_first_coords[1]:.2f}, "
            f"{xy_first_coords[2]:.2f})"
        )

        moved = self.send_coords(
            xy_first_coords,
            "direct_pick_xy_first",
            wait=float(
                self.cfg.get(
                    "direct_pick_move_wait_sec",
                    1.2
                )
            )
        )

        if not moved:
            print("[DIRECT XYZ PICK ERROR] X/Y 선이동 실패")
            return False

        z_target_coords = xy_first_coords[:6]
        z_target_coords[2] = float(target_coords[2])

        print(
            "[DIRECT XYZ PICK] Z 하강 이동 | "
            f"target_z={z_target_coords[2]:.2f}"
        )

        moved = self.send_coords(
            z_target_coords,
            "direct_pick_z_down",
            wait=float(
                self.cfg.get(
                    "direct_pick_z_move_wait_sec",
                    self.cfg.get(
                        "direct_pick_move_wait_sec",
                        1.2
                    )
                )
            )
        )

        if not moved:
            print("[DIRECT XYZ PICK ERROR] Z 하강 이동 실패")
            return False

        time.sleep(
            float(
                self.cfg.get(
                    "direct_pick_settle_sec",
                    0.20
                )
            )
        )

        measured_coords = self.get_coords()

        if (
            isinstance(measured_coords, list)
            and len(measured_coords) >= 6
        ):
            target_coords = [
                float(value)
                for value in measured_coords[:6]
            ]

        print("[DIRECT XYZ PICK] 그리퍼 닫기")

        if not self.close_gripper():
            print("[DIRECT XYZ PICK ERROR] 그리퍼 닫기 실패")
            return False

        time.sleep(
            float(
                self.cfg.get(
                    "lock_pick_after_grip_wait_sec",
                    0.15
                )
            )
        )

        lift_coords = target_coords[:6]
        lift_coords[2] = float(lift_coords[2]) + abs(
            float(
                self.cfg.get(
                    "locked_vertical_lift_mm",
                    55.0
                )
            )
        )

        lifted = self.send_coords(
            lift_coords,
            "direct_xyz_pick_lift",
            wait=float(
                self.cfg.get(
                    "locked_vertical_lift_wait_sec",
                    0.45
                )
            )
        )

        if not lifted:
            print("[DIRECT XYZ PICK ERROR] 들어올리기 실패")
            return False

        returned = self.return_to_camera_ready_pose()

        if not returned:
            print("[DIRECT XYZ PICK ERROR] 초기 위치 복귀 실패")
            return False

        self.object_ready = False
        self.object_ready_announced = False
        self.last_recognized_detection = None
        self.last_recognized_time = 0.0
        self.auto_pick_latched = False
        self.auto_stable_count = 0
        self.auto_missing_count = 0
        self.last_detect_center = None

        self.cfg["last_direct_xyz_pick"] = {
            "center_u_px": round(float(center_u), 3),
            "center_v_px": round(float(center_v), 3),
            "depth_mm": round(depth_mm, 3),
            "target_coords": [
                round(value, 3)
                for value in target_coords
            ],
            "timestamp": round(time.time(), 3)
        }
        save_config(self.cfg)

        print("[DIRECT XYZ PICK COMPLETE]")
        return True


    # 수동 보정 시작: 현재 카메라 인식값과 자동 계산 목표 좌표를 저장합니다.
    def start_direct_pick_manual_calibration(self):

        frame = self.read_frame()

        if frame is None:
            print("[PICK CALIB ERROR] 카메라 영상 없음")
            return False

        # 보정은 현재 화면의 컨테이너 위치를 기준으로 새로 시작합니다.
        detection, _ = self.detect_rectangle_object(frame)

        if not isinstance(detection, dict):
            detection = getattr(
                self,
                "last_recognized_detection",
                None
            )

        if not isinstance(detection, dict):
            print("[PICK CALIB ERROR] 컨테이너 인식값 없음")
            return False

        current_coords = self.get_coords()

        if (
            not isinstance(current_coords, list)
            or len(current_coords) < 6
        ):
            print("[PICK CALIB ERROR] 현재 TCP 좌표 읽기 실패")
            return False

        current_coords = [
            float(value)
            for value in current_coords[:6]
        ]

        pick_plan = self.calculate_direct_xyz_pick_target(
            current_coords,
            detection,
            frame.shape,
            apply_manual_calibration=False
        )

        if not isinstance(pick_plan, dict):
            print("[PICK CALIB ERROR] 자동 목표 좌표 계산 실패")
            return False

        recognition_coords = detection.get(
            "tcp_coords_at_recognition",
            current_coords
        )
        if (
            not isinstance(recognition_coords, (list, tuple))
            or len(recognition_coords) < 6
        ):
            recognition_coords = current_coords

        detection_center = detection.get(
            "center",
            (
                pick_plan["center_u"],
                pick_plan["center_v"]
            )
        )
        if (
            not isinstance(detection_center, (list, tuple))
            or len(detection_center) < 2
        ):
            detection_center = (
                pick_plan["center_u"],
                pick_plan["center_v"]
            )

        self.cfg["auto_pick_enabled"] = False
        self.pick_manual_calibration_active = True
        self.cfg["pending_direct_pick_manual_calibration"] = {
            "start_coords": [
                round(value, 3)
                for value in current_coords
            ],
            "tcp_coords_at_recognition": [
                round(value, 3)
                for value in recognition_coords[:6]
            ],
            "auto_target_coords": [
                round(value, 3)
                for value in pick_plan["target_coords"]
            ],
            "base_target_coords": [
                round(value, 3)
                for value in pick_plan["base_target_coords"]
            ],
            "detected_container": {
                "center": [
                    round(float(value), 3)
                    for value in detection_center[:2]
                ],
                "depth_mm": round(pick_plan["depth_mm"], 3),
                "confidence": round(
                    float(detection.get("confidence", 0.0)),
                    4
                ),
                "recognized_at": detection.get(
                    "recognized_at",
                    None
                )
            },
            "center_u_px": round(pick_plan["center_u"], 3),
            "center_v_px": round(pick_plan["center_v"], 3),
            "depth_mm": round(pick_plan["depth_mm"], 3),
            "move_x_mm": round(pick_plan["move_x_mm"], 3),
            "move_y_mm": round(pick_plan["move_y_mm"], 3),
            "move_z_mm": round(pick_plan["move_z_mm"], 3),
            "visual_move_x_mm": round(
                pick_plan["visual_move_x_mm"],
                3
            ),
            "visual_move_y_mm": round(
                pick_plan["visual_move_y_mm"],
                3
            ),
            "visual_move_z_mm": round(
                pick_plan["visual_move_z_mm"],
                3
            ),
            "old_direct_pick_final_robot_x_mm": round(
                float(
                    self.cfg.get(
                        "direct_pick_final_robot_x_mm",
                        0.0
                    )
                ),
                3
            ),
            "old_direct_pick_final_robot_y_mm": round(
                float(
                    self.cfg.get(
                        "direct_pick_final_robot_y_mm",
                        0.0
                    )
                ),
                3
            ),
            "old_direct_pick_final_extra_down_mm": round(
                float(
                    self.cfg.get(
                        "direct_pick_final_extra_down_mm",
                        0.0
                    )
                ),
                3
            ),
            "timestamp": round(time.time(), 3),
        }

        save_config(self.cfg)

        if bool(
            self.cfg.get(
                "pick_manual_calibration_open_gripper",
                True
            )
        ):
            self.arm.set_gripper_value(
                int(
                    self.cfg.get(
                        "gripper_open",
                        100
                    )
                ),
                int(
                    self.cfg.get(
                        "gripper_speed",
                        50
                    )
                ),
                wait=float(
                    self.cfg.get(
                        "lock_pick_gripper_open_wait_sec",
                        0.25
                    )
                )
            )

        if bool(
            self.cfg.get(
                "pick_manual_calibration_release_servos",
                False
            )
        ):
            release_response = self.arm.release_all_servos()

            if (
                not isinstance(release_response, dict)
                or not release_response.get("ok", False)
            ):
                print(
                    "[PICK CALIB WARNING] "
                    "서보 토크 해제 실패:",
                    release_response
                )
            else:
                print(
                    "[PICK CALIB] "
                    "서보 토크 해제 완료. 손으로 그리퍼를 물체 집기 위치에 맞추세요."
                )
        else:
            print(
                "[PICK CALIB] "
                "키보드 수동 조작 모드입니다. "
                "필요하면 i/k/j/l/u/m 또는 1~0/-/= 관절키로 그리퍼를 컨테이너 집기 위치까지 이동하세요."
            )

        print(
            "[PICK CALIB START] "
            "위치를 맞춘 뒤 y를 누르면 현재 좌표를 저장합니다. "
            "그 다음 w를 누르면 저장한 보정값으로 집습니다."
        )
        print(
            "[PICK CALIB START] "
            f"auto_target="
            f"{[round(value, 2) for value in pick_plan['target_coords'][:3]]}"
        )

        return True


    # 수동으로 맞춘 현재 TCP 좌표를 성공 좌표로 저장하고 보정값을 갱신합니다.
    def save_direct_pick_manual_calibration(self):

        pending = self.cfg.get(
            "pending_direct_pick_manual_calibration",
            None
        )

        if not isinstance(pending, dict):
            print("[PICK CALIB ERROR] 먼저 n을 눌러 보정을 시작하세요.")
            return False

        auto_target = pending.get("auto_target_coords", None)

        if (
            not isinstance(auto_target, list)
            or len(auto_target) < 6
        ):
            print("[PICK CALIB ERROR] 저장된 자동 목표 좌표가 없습니다.")
            return False

        actual_coords = self.get_coords()

        if (
            not isinstance(actual_coords, list)
            or len(actual_coords) < 6
        ):
            print("[PICK CALIB ERROR] 현재 TCP 좌표 읽기 실패")
            return False

        actual_coords = [
            float(value)
            for value in actual_coords[:6]
        ]

        focus_response = self.arm.focus_all_servos()

        if (
            not isinstance(focus_response, dict)
            or not focus_response.get("ok", False)
        ):
            print(
                "[PICK CALIB WARNING] "
                "서보 토크 고정 실패:",
                focus_response
            )
        else:
            print("[PICK CALIB] 서보 토크 고정 완료")

        auto_target = [
            float(value)
            for value in auto_target[:6]
        ]

        correction_x = actual_coords[0] - auto_target[0]
        correction_y = actual_coords[1] - auto_target[1]
        correction_z = actual_coords[2] - auto_target[2]
        actual_down_from_start = None

        start_coords = pending.get("start_coords", None)
        if (
            isinstance(start_coords, list)
            and len(start_coords) >= 3
        ):
            actual_down_from_start = (
                float(start_coords[2])
                - float(actual_coords[2])
            )

        new_offset_x = correction_x
        new_offset_y = correction_y
        new_offset_z = correction_z
        new_extra_down = max(
            0.0,
            -correction_z
        )

        self.cfg["direct_pick_final_robot_x_mm"] = round(
            new_offset_x,
            3
        )
        self.cfg["direct_pick_final_robot_y_mm"] = round(
            new_offset_y,
            3
        )
        self.cfg["direct_pick_final_extra_down_mm"] = round(
            new_extra_down,
            3
        )
        self.cfg["direct_pick_manual_offset_x_mm"] = round(
            new_offset_x,
            3
        )
        self.cfg["direct_pick_manual_offset_y_mm"] = round(
            new_offset_y,
            3
        )
        self.cfg["direct_pick_manual_offset_z_mm"] = round(
            new_offset_z,
            3
        )
        self.cfg["direct_pick_use_relative_manual_calibration"] = True
        self.cfg["direct_pick_require_fresh_detection"] = True
        self.cfg["direct_pick_manual_calibration_model"] = "visual_delta"

        self.cfg["last_direct_pick_manual_calibration"] = {
            "auto_target_coords": [
                round(value, 3)
                for value in auto_target
            ],
            "actual_success_coords": [
                round(value, 3)
                for value in actual_coords
            ],
            "correction_x_mm": round(correction_x, 3),
            "correction_y_mm": round(correction_y, 3),
            "correction_z_mm": round(correction_z, 3),
            "actual_down_from_start_mm": (
                None
                if actual_down_from_start is None
                else round(actual_down_from_start, 3)
            ),
            "direct_pick_final_robot_x_mm": round(
                new_offset_x,
                3
            ),
            "direct_pick_final_robot_y_mm": round(
                new_offset_y,
                3
            ),
            "direct_pick_final_extra_down_mm": round(
                new_extra_down,
                3
            ),
            "direct_pick_manual_offset_x_mm": round(
                new_offset_x,
                3
            ),
            "direct_pick_manual_offset_y_mm": round(
                new_offset_y,
                3
            ),
            "direct_pick_manual_offset_z_mm": round(
                new_offset_z,
                3
            ),
            "model": "visual_delta",
            "source": pending,
            "timestamp": round(time.time(), 3),
        }
        self.cfg["pending_direct_pick_manual_calibration"] = None
        self.pick_manual_calibration_active = False

        save_config(self.cfg)

        print(
            "[PICK CALIB SAVED] "
            f"correction=({correction_x:+.2f}, "
            f"{correction_y:+.2f}, {correction_z:+.2f})mm"
        )
        print(
            "[PICK CALIB SAVED] "
            f"offset=({new_offset_x:+.2f}, "
            f"{new_offset_y:+.2f}, "
            f"{new_offset_z:+.2f})"
        )

        if actual_down_from_start is not None:
            print(
                "[PICK CALIB SAVED] "
                f"actual_down_from_start={actual_down_from_start:.2f}mm"
            )

        if bool(
            self.cfg.get(
                "pick_manual_calibration_return_ready_after_save",
                True
            )
        ):
            print(
                "[PICK CALIB] "
                "저장 완료. w 재실행을 위해 카메라 준비 자세로 복귀합니다."
            )
            self.return_to_camera_ready_pose()

        return True


    # 첫 좌표 -> 바닥 카메라 -> 확대 정렬 -> 그리퍼 정렬 -> 수직 집기입니다.
    def pick_and_return_home(self):

        if bool(
            self.cfg.get(
                "direct_xyz_pick_enabled",
                True
            )
        ):
            return self.direct_xyz_pick_and_return_home()

        if not getattr(
            self,
            "object_ready",
            False
        ):

            print(
                "[INITIAL FLOOR PICK ERROR] "
                "인식된 컨테이너가 없습니다."
            )

            return False

        print(
            "[INITIAL FLOOR PICK START] "
            "초기 좌표 기반 바닥 카메라 자동 집기 시작"
        )

        # 시작 전에 그리퍼를 엽니다.
        open_response = self.arm.set_gripper_value(
            int(
                self.cfg.get(
                    "gripper_open",
                    100
                )
            ),
            int(
                self.cfg.get(
                    "gripper_speed",
                    50
                )
            ),
            wait=float(
                self.cfg.get(
                    "lock_pick_gripper_open_wait_sec",
                    0.25
                )
            )
        )

        if (
            not isinstance(open_response, dict)
            or not open_response.get(
                "ok",
                False
            )
        ):

            print(
                "[INITIAL FLOOR PICK ERROR] "
                "그리퍼 열기 실패:",
                open_response
            )

            return False

        if bool(
            self.cfg.get(
                "initial_coarse_approach_enabled",
                True
            )
        ):
            print(
                "[FLOW 0/5] 초기 카메라 기준 컨테이너 중심 접근"
            )

            coarse_ok = self.coarse_approach_to_object()

            if not coarse_ok:
                print(
                    "[INITIAL FLOOR PICK ERROR] "
                    "초기 카메라 중심 접근 실패"
                )
                return False

            time.sleep(
                float(
                    self.cfg.get(
                        "initial_relock_settle_sec",
                        0.12
                    )
                )
            )

            if not self.relock_visible_object_after_approach():
                print(
                    "[INITIAL FLOOR PICK ERROR] "
                    "초기 접근 후 목표 재잠금 실패"
                )
                return False

        # 1단계: 첫 화면 좌표를 저장합니다.
        initial_target = (
            self.capture_initial_container_coordinate()
        )

        if not isinstance(initial_target, dict):

            return False

        # 2단계: 초기 좌표 근처로 이동하고 카메라를 바닥 방향으로 전환합니다.
        floor_start_coords = (
            self.move_to_initial_floor_target(
                initial_target
            )
        )

        if floor_start_coords is None:

            return False

        # 3단계: 바닥 카메라로 확대하면서 X/Y 중심을 맞춥니다.
        floor_lock_result = (
            self.floor_visual_approach_and_center(
                initial_target,
                floor_start_coords
            )
        )

        if not isinstance(
            floor_lock_result,
            dict
        ):

            print(
                "[INITIAL FLOOR PICK ERROR] "
                "바닥 카메라 확대·중심 정렬 실패"
            )

            return False

        # 4단계: 최종 좌표를 그리퍼 중심 X/Y에 맞춥니다.
        gripper_coords = (
            self.align_gripper_to_floor_locked_container(
                floor_lock_result
            )
        )

        if gripper_coords is None:

            return False

        # 5단계: X/Y를 고정하고 수직 하강합니다.
        final_pick_coords = (
            self.vertical_pick_from_locked_coords(
                gripper_coords,
                floor_lock_result.get(
                    "detection",
                    None
                )
            )
        )

        if final_pick_coords is None:

            print(
                "[INITIAL FLOOR PICK ERROR] "
                "그리퍼 기준 수직 하강 실패"
            )

            return False

        print(
            "[INITIAL FLOOR PICK] "
            "그리퍼 닫기"
        )

        close_response = self.arm.set_gripper_value(
            int(
                self.cfg.get(
                    "gripper_close",
                    0
                )
            ),
            int(
                self.cfg.get(
                    "gripper_speed",
                    50
                )
            ),
            wait=float(
                self.cfg.get(
                    "lock_pick_gripper_close_wait_sec",
                    0.45
                )
            )
        )

        if (
            not isinstance(close_response, dict)
            or not close_response.get(
                "ok",
                False
            )
        ):

            print(
                "[INITIAL FLOOR PICK ERROR] "
                "그리퍼 닫기 실패:",
                close_response
            )

            return False

        time.sleep(
            float(
                self.cfg.get(
                    "lock_pick_after_grip_wait_sec",
                    0.15
                )
            )
        )

        lift_mm = abs(
            float(
                self.cfg.get(
                    "locked_vertical_lift_mm",
                    55.0
                )
            )
        )

        lift_coords = final_pick_coords[:6]

        lift_coords[2] = (
            float(lift_coords[2])
            + lift_mm
        )

        print(
            "[INITIAL FLOOR LIFT] "
            f"z={float(final_pick_coords[2]):.2f}"
            f"->{float(lift_coords[2]):.2f}"
        )

        moved_lift = self.send_coords(
            lift_coords,
            "initial_floor_vertical_lift",
            wait=float(
                self.cfg.get(
                    "locked_vertical_lift_wait_sec",
                    0.45
                )
            )
        )

        if not moved_lift:

            print(
                "[INITIAL FLOOR PICK ERROR] "
                "수직 상승 실패"
            )

            return False

        time.sleep(
            float(
                self.cfg.get(
                    "before_return_home_wait_sec",
                    0.15
                )
            )
        )

        returned = self.return_to_camera_ready_pose()

        if not returned:

            print(
                "[INITIAL FLOOR PICK ERROR] "
                "초기 자세 복귀 실패"
            )

            return False

        # 다음 작업을 위해 상태를 초기화합니다.
        self.object_ready = False
        self.object_ready_announced = False
        self.last_recognized_detection = None
        self.last_recognized_time = 0.0
        self.auto_pick_latched = False
        self.auto_stable_count = 0
        self.auto_missing_count = 0
        self.last_detect_center = None

        print(
            "[INITIAL FLOOR PICK COMPLETE] "
            "초기 좌표 기반 자동 집기 완료"
        )

        return True








    # 로봇팔 서버와 실제 로봇 통신이 준비될 때까지 기다리는 함수입니다.
    def wait_for_arm_server(self, timeout_sec=30.0):

        # 대기 종료 시간을 계산합니다.
        deadline = time.time() + float(timeout_sec)

        # 시도 횟수입니다.
        attempt = 0

        # 제한 시간 동안 서버 상태를 반복 확인합니다.
        while time.time() < deadline:

            # 시도 횟수를 증가시킵니다.
            attempt += 1

            # 서버에 현재 관절 각도를 요청합니다.
            response = self.arm.request({
                "cmd": "get_angles"
            })

            # 응답에서 관절 각도를 가져옵니다.
            angles = response.get(
                "angles",
                None
            )

            # 정상적인 관절값이 반환되면 서버와 로봇이 준비된 것입니다.
            if (
                isinstance(response, dict)
                and response.get("ok", False)
                and isinstance(angles, list)
                and len(angles) >= 6
            ):

                # 준비 완료 로그를 출력합니다.
                print(
                    "[ARM SERVER READY] "
                    f"attempt={attempt} | "
                    f"angles="
                    f"{[round(float(value), 2) for value in angles[:6]]}"
                )

                # 준비 완료를 반환합니다.
                return True

            # 아직 준비되지 않았음을 출력합니다.
            print(
                "[ARM SERVER WAIT] "
                f"{attempt}회 | "
                f"response={response}"
            )

            # 서버와 시리얼 초기화를 기다립니다.
            time.sleep(0.5)

        # 제한 시간 안에 준비되지 않았습니다.
        print(
            "[ARM SERVER TIMEOUT] "
            f"{float(timeout_sec):.1f}초 동안 준비되지 않았습니다."
        )

        # 준비 실패를 반환합니다.
        return False


    # 시작 기본 자세로 이동하는 함수입니다.
    def go_camera_ready_pose(self):

        # 설정된 초기 관절 각도를 가져옵니다.
        angles = self.cfg.get(
            "camera_ready_angles",
            None
        )

        # 관절 각도 형식을 확인합니다.
        if (
            not isinstance(angles, list)
            or len(angles) < 6
        ):

            # 설정 오류를 출력합니다.
            print(
                "[STARTUP ERROR] "
                "camera_ready_angles 값 오류:",
                angles
            )

            # 초기 자세 이동 실패입니다.
            return False

        # 이동할 6개 관절값을 실수형으로 변환합니다.
        target_angles = [
            float(value)
            for value in angles[:6]
        ]

        # 초기 자세 이동 속도입니다.
        angle_speed = int(
            self.cfg.get(
                "angle_speed",
                35
            )
        )

        # 초기 자세 이동 완료 대기 시간입니다.
        startup_wait_sec = float(
            self.cfg.get(
                "startup_move_wait_sec",
                3.0
            )
        )

        # 초기 자세 이동 정보를 출력합니다.
        print(
            "[STARTUP] camera_ready_angles로 이동:",
            target_angles,
            "| speed=",
            angle_speed
        )

        # 로봇팔 서버에 초기 자세 이동 명령을 보냅니다.
        response = self.arm.send_angles(
            target_angles,
            angle_speed,
            wait=startup_wait_sec
        )

        # 서버가 이동 명령을 거부한 경우입니다.
        if (
            not isinstance(response, dict)
            or not response.get("ok", False)
        ):

            # 이동 실패 내용을 출력합니다.
            print(
                "[STARTUP ERROR] "
                "초기 자세 이동 명령 실패:",
                response
            )

            # 실패를 반환합니다.
            return False

        # 좌표 이동 캐시를 초기화합니다.
        reset_response = self.arm.reset_tcp_cache()

        # 그리퍼를 열린 상태로 만듭니다.
        gripper_response = self.arm.set_gripper_value(
            int(
                self.cfg.get(
                    "gripper_open",
                    100
                )
            ),
            int(
                self.cfg.get(
                    "gripper_speed",
                    30
                )
            ),
            wait=0.5
        )

        # 좌표 캐시 초기화 실패를 출력합니다.
        if not reset_response.get("ok", False):
            print(
                "[STARTUP WARNING] "
                "TCP 좌표 캐시 초기화 실패:",
                reset_response
            )

        # 그리퍼 열기 실패를 출력합니다.
        if not gripper_response.get("ok", False):
            print(
                "[STARTUP WARNING] "
                "그리퍼 열기 실패:",
                gripper_response
            )

        # 초기 자세 이동 완료를 출력합니다.
        print(
            "[STARTUP OK] "
            "초기 카메라 자세 이동 명령 완료"
        )

        # 성공을 반환합니다.
        return True


    # 현재 자세를 시작 카메라 자세로 저장하는 함수입니다.
    def save_current_as_camera_ready(self):

        # 현재 좌표가 아니라 관절값 저장이 더 좋지만, 서버에 get_angles를 요청합니다.
        response = self.arm.request({"cmd": "get_angles"})

        # 관절값을 가져옵니다.
        angles = response.get("angles", None)

        # 값 검증입니다.
        if not isinstance(angles, list) or len(angles) < 6:
            print("[SAVE] get_angles 실패:", response)
            return

        # 현재 관절값을 시작 자세로 저장합니다.
        self.cfg["camera_ready_angles"] = angles[:6]

        # 설정 저장입니다.
        save_config(self.cfg)

        # 저장 로그입니다.
        print("[SAVE] camera_ready_angles:", angles[:6])

    # ROS 영상 콜백을 지속적으로 실행하는 별도 스레드입니다.
    def ros_spin_loop(self):

        # 프로그램이 실행 중인 동안 ROS 콜백을 빠르게 처리합니다.
        while (
            self.ros_spin_running
            and rclpy.ok()
        ):

            try:

                # 짧은 대기 시간으로 최신 영상 콜백을 처리합니다.
                rclpy.spin_once(
                    self.ros_node,
                    timeout_sec=0.01
                )

            except Exception as error:

                print(
                    "[ROS SPIN ERROR]",
                    type(error).__name__,
                    error
                )

                time.sleep(0.02)


    # /camera/image_rect 영상이 들어올 때 실행되는 함수입니다.
    def image_callback(self, msg):

        try:

            # ROS Image를 OpenCV BGR 영상으로 변환합니다.
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            # 이전 영상은 버리고 최신 영상으로 덮어씁니다.
            with self.camera_lock:
                self.latest_frame = frame

        except Exception as error:

            print(
                f"[CAMERA ERROR] 영상 변환 실패: {error}"
            )


    # 가장 최근에 받은 보정 영상을 반환합니다.
    def read_frame(self):

        # ROS 콜백은 별도 스레드가 처리하므로 여기서는 기다리지 않습니다.
        with self.camera_lock:

            # 아직 영상이 없으면 None을 반환합니다.
            if self.latest_frame is None:
                return None

            # 최신 프레임을 복사해서 반환합니다.
            return self.latest_frame.copy()


    def detect_rectangle_object(self, frame):

        # BGR 영상을 흑백으로 변환합니다.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 조명 차이를 줄이기 위해 CLAHE 명암 보정을 적용합니다.
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # 명암 보정된 흑백 영상입니다.
        gray_eq = clahe.apply(gray)

        # 노이즈를 줄이기 위해 블러를 적용합니다.
        blur = cv2.GaussianBlur(gray_eq, (5, 5), 0)

        # 에지 기반 검출입니다.
        edges = cv2.Canny(blur, 35, 120)

        # 적응형 이진화로 밝기 변화에도 대응합니다.
        adaptive = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            5
        )

        # Otsu 이진화도 추가합니다.
        _, otsu_inv = cv2.threshold(
            blur,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # 형태학 처리용 커널입니다.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # 에지를 굵게 만들어 끊어진 직사각형 선을 연결합니다.
        edges = cv2.dilate(edges, kernel, iterations=1)

        # 이진화 영상의 작은 노이즈를 제거하고 구멍을 메웁니다.
        adaptive = cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, kernel, iterations=2)
        otsu_inv = cv2.morphologyEx(otsu_inv, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 세 가지 결과를 합쳐서 검출 안정성을 높입니다.
        combined = cv2.bitwise_or(edges, adaptive)
        combined = cv2.bitwise_or(combined, otsu_inv)

        # 외곽선을 찾습니다.
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 실제 물체의 긴 변/짧은 변 mm입니다.
        real_long_mm = max(float(self.cfg["object_length_mm"]), float(self.cfg["object_width_mm"]))
        real_short_mm = min(float(self.cfg["object_length_mm"]), float(self.cfg["object_width_mm"]))

        # 실제 비율입니다.
        target_aspect = real_long_mm / real_short_mm

        # 최고 후보입니다.
        best = None

        # 최고 점수입니다.
        best_score = 999999.0

        # 모든 외곽선을 검사합니다.
        for contour in contours:

            # 외곽선 면적입니다.
            area = cv2.contourArea(contour)

            # 면적이 너무 작거나 크면 제외합니다.
            if area < float(self.cfg["min_area_px"]) or area > float(self.cfg["max_area_px"]):
                continue

            # 회전 직사각형을 구합니다.
            rect = cv2.minAreaRect(contour)

            # 중심, 크기, 각도입니다.
            (cx, cy), (rw, rh), angle = rect

            # 크기 검증입니다.
            if rw <= 3 or rh <= 3:
                continue

            # 긴 변과 짧은 변 픽셀입니다.
            long_px = max(rw, rh)
            short_px = min(rw, rh)

            # 화면상 비율입니다.
            aspect = long_px / short_px

            # 실제 비율과의 차이입니다.
            aspect_error = abs(aspect - target_aspect)

            # 비율 오차가 너무 크면 제외합니다.
            if aspect_error > float(self.cfg["aspect_tolerance"]):
                continue

            # 회전 직사각형 면적입니다.
            rect_area = rw * rh

            # 실제 외곽선이 직사각형을 얼마나 잘 채우는지 봅니다.
            rectangularity = area / rect_area if rect_area > 0 else 0.0

            # 직사각형성이 너무 낮으면 제외합니다.
            if rectangularity < float(self.cfg.get("rectangularity_min", 0.25)):
                continue

            # 실제 물체 크기 기준 px/mm를 계산합니다.
            px_per_mm_long = long_px / real_long_mm
            px_per_mm_short = short_px / real_short_mm
            px_per_mm = (px_per_mm_long + px_per_mm_short) / 2.0

            # px/mm가 비정상이면 제외합니다.
            if px_per_mm <= 0.01:
                continue

            # 긴 변/짧은 변 스케일 차이입니다.
            scale_error = abs(px_per_mm_long - px_per_mm_short) / px_per_mm

            # 원근이나 오검출이 너무 크면 제외합니다.
            if scale_error > float(self.cfg.get("scale_error_tolerance", 0.75)):
                continue

            # 꼭짓점입니다.
            box = cv2.boxPoints(rect).astype(int)

            # 컨테이너의 실제 가로세로 비율과 얼마나 가까운지 계산합니다.
            aspect_limit = max(
                float(
                    self.cfg.get(
                        "aspect_tolerance",
                        0.45
                    )
                ),
                0.001
            )

            # 비율이 정확할수록 1에 가까워집니다.
            aspect_score = max(
                0.0,
                min(
                    1.0,
                    1.0 - aspect_error / aspect_limit
                )
            )

            # 허용할 최소 직사각형성을 가져옵니다.
            rectangularity_min = float(
                self.cfg.get(
                    "rectangularity_min",
                    0.52
                )
            )

            # 외곽선이 직사각형을 잘 채울수록 1에 가까워집니다.
            rectangularity_score = max(
                0.0,
                min(
                    1.0,
                    (
                        rectangularity
                        - rectangularity_min
                    )
                    / max(
                        1.0 - rectangularity_min,
                        0.001
                    )
                )
            )

            # 긴 변과 짧은 변의 스케일 차이 허용값입니다.
            scale_limit = max(
                float(
                    self.cfg.get(
                        "scale_error_tolerance",
                        0.40
                    )
                ),
                0.001
            )

            # 긴 변과 짧은 변의 실제 크기 계산이 비슷할수록 1에 가까워집니다.
            scale_score = max(
                0.0,
                min(
                    1.0,
                    1.0 - scale_error / scale_limit
                )
            )

            # 영상 크기를 가져옵니다.
            frame_height, frame_width = frame.shape[:2]

            # 영상 중앙 좌표입니다.
            image_center_x = frame_width / 2.0
            image_center_y = frame_height / 2.0

            # 후보가 영상 중앙에서 얼마나 떨어져 있는지 계산합니다.
            center_distance = (
                (float(cx) - image_center_x) ** 2
                + (float(cy) - image_center_y) ** 2
            ) ** 0.5

            # 영상 중앙에서 가장 먼 거리입니다.
            maximum_center_distance = (
                image_center_x ** 2
                + image_center_y ** 2
            ) ** 0.5

            # 화면 중앙에 가까울수록 1에 가까워집니다.
            center_score = max(
                0.0,
                min(
                    1.0,
                    1.0
                    - center_distance
                    / max(
                        maximum_center_distance,
                        1.0
                    )
                )
            )

            # 컨테이너 일치 확률을 계산합니다.
            # 실제 78×33mm 비율을 가장 중요하게 평가합니다.
            confidence = (
                aspect_score * 0.50
                + rectangularity_score * 0.25
                + scale_score * 0.17
                + center_score * 0.08
            )

            # 컨테이너로 인정할 최소 확률입니다.
            minimum_confidence = float(
                self.cfg.get(
                    "container_confidence_min",
                    0.68
                )
            )

            # 확률이 낮으면 컨테이너 후보에서 제외합니다.
            if confidence < minimum_confidence:
                continue

            # 최고 확률 후보를 선택하기 위한 점수입니다.
            # 기존 코드는 작은 점수가 우선이므로 1에서 확률을 뺍니다.
            score = 1.0 - confidence

            # 더 높은 확률의 후보면 저장합니다.
            if score < best_score:
                best_score = score

                # 컨테이너 후보 정보를 저장합니다.
                best = {
                    "center": (
                        float(cx),
                        float(cy)
                    ),
                    "box": box,
                    "area": float(area),
                    "aspect": float(aspect),
                    "angle": float(angle),
                    "long_px": float(long_px),
                    "short_px": float(short_px),
                    "px_per_mm": float(px_per_mm),
                    "mm_per_px": float(
                        1.0 / px_per_mm
                    ),
                    "rectangularity": float(
                        rectangularity
                    ),
                    "scale_error": float(
                        scale_error
                    ),
                    "confidence": float(
                        confidence
                    )
                }

        # 최고 후보와 디버그 영상을 반환합니다.
        # 보정 영상의 초점거리와 검출된 px/mm로 카메라-컨테이너 거리를 계산합니다.
        if best is not None:

            # 검출된 영상 크기 비율입니다.
            px_per_mm_value = float(
                best.get(
                    "px_per_mm",
                    0.0
                )
            )

            # 보정 영상 projection_matrix의 fx와 fy 평균입니다.
            focal_px = (
                float(
                    self.cfg.get(
                        "camera_fx_rect_px",
                        973.32886
                    )
                )
                + float(
                    self.cfg.get(
                        "camera_fy_rect_px",
                        989.13171
                    )
                )
            ) / 2.0

            # 정상적인 영상 크기일 때만 거리 계산을 수행합니다.
            if px_per_mm_value > 0.01:

                # 거리 = 초점거리(px) / 영상 크기(px/mm)입니다.
                raw_depth_mm = focal_px / px_per_mm_value

                # 허용할 최소 깊이입니다.
                depth_min_mm = float(
                    self.cfg.get(
                        "depth_min_mm",
                        80.0
                    )
                )

                # 허용할 최대 깊이입니다.
                depth_max_mm = float(
                    self.cfg.get(
                        "depth_max_mm",
                        700.0
                    )
                )

                # 정상 거리 범위 안의 값만 사용합니다.
                if depth_min_mm <= raw_depth_mm <= depth_max_mm:

                    # 깊이값 흔들림을 줄이는 필터 계수입니다.
                    alpha = min(
                        1.0,
                        max(
                            0.05,
                            float(
                                self.cfg.get(
                                    "depth_filter_alpha",
                                    0.35
                                )
                            )
                        )
                    )

                    # 이전 프레임의 깊이값을 읽습니다.
                    previous_depth_mm = getattr(
                        self,
                        "latest_depth_mm",
                        None
                    )

                    # 첫 측정이면 원본 값을 그대로 사용합니다.
                    if previous_depth_mm is None:
                        filtered_depth_mm = raw_depth_mm

                    # 이후 프레임은 저역통과 필터를 적용합니다.
                    else:
                        filtered_depth_mm = (
                            (1.0 - alpha)
                            * float(previous_depth_mm)
                            + alpha
                            * raw_depth_mm
                        )

                    # 필터 적용 전 깊이를 검출 결과에 저장합니다.
                    best["depth_mm_raw"] = raw_depth_mm

                    # 필터 적용 후 깊이를 검출 결과에 저장합니다.
                    best["depth_mm"] = filtered_depth_mm

                    # 자동 집기에서 사용할 최신 깊이를 저장합니다.
                    self.latest_depth_mm = filtered_depth_mm

                    # 깊이를 측정한 시간을 저장합니다.
                    self.latest_depth_time = time.time()

        # 사각형 검출 결과와 이진 영상을 반환합니다.
        return best, combined


    # 목표 픽셀을 반환하는 함수입니다.
    def get_target_pixel(self, w, h):

        # 저장된 target_u가 없으면 화면 중앙입니다.
        target_u = self.cfg["target_u"] if self.cfg["target_u"] is not None else w / 2.0

        # 저장된 target_v가 없으면 화면 중앙입니다.
        target_v = self.cfg["target_v"] if self.cfg["target_v"] is not None else h / 2.0

        # 목표 픽셀을 반환합니다.
        return float(target_u), float(target_v)

    # 화면 표시를 그리는 함수입니다.
    def draw_overlay(self, frame, detection):

        # 화면 크기를 가져옵니다.
        h, w = frame.shape[:2]

        # 목표 픽셀입니다.
        target_u, target_v = self.get_target_pixel(w, h)

        # 목표 위치 십자 표시입니다.
        cv2.drawMarker(frame, (int(target_u), int(target_v)), (0, 0, 255),
                       markerType=cv2.MARKER_CROSS, markerSize=25, thickness=2)

        # 직사각형 검출 시 표시합니다.
        if detection is not None:

            # 중심을 가져옵니다.
            cx, cy = detection["center"]

            # 박스를 가져옵니다.
            box = detection["box"]

            # 직사각형 외곽선을 그립니다.
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

            # 중심점을 표시합니다.
            cv2.circle(frame, (int(cx), int(cy)), 6, (0, 255, 0), -1)

            # 오차를 계산합니다.
            err_x = cx - target_u
            err_y = cy - target_v

            # 검출 정보를 표시합니다.
            cv2.putText(frame,
                        f"RECT aspect={detection['aspect']:.2f} px/mm={detection.get('px_per_mm',0):.2f} rect={detection.get('rectangularity',0):.2f} err=({err_x:.0f},{err_y:.0f})",
                        (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (0, 255, 0),
                        2)

        # 상태 표시입니다.
        state = "BUSY" if self.task_busy else "READY"

        # 안내 문구입니다.
        lines = [
            f"{state}  rectangle 33x78x35mm",
            "1/2:M1 left/right  3/4:M2 left/right",
            "5/6:M3 left/right  7/8:M4 left/right",
            "9/0:M5 left/right  -/=:M6 left/right",
            "a: go camera ready pose",
            "t: save current angles as camera ready",
            "i/k: z +/-   j/l: y +/-   u/m: x +/-",
            "q: auto center rectangle",
            "z: move view to container zero",
            "v: save current container as zero",
            "f: save current pose as object memory",
            "n: start manual pick calib",
            "y: save manual pick pose",
            "w/b: pick recognized object + return home",
            "e: place",
            "s: save scan coords",
            "h: go scan coords",
            "p: save place coords",
            "o/c: gripper open/close",
            "[/]: pick_down -/+   ;/\': depth bias +/-",
            "ESC: exit"
        ]

        if self.pick_manual_calibration_active:
            lines.insert(
                1,
            "PICK CALIB: move arm by hand to real grip pose, press y, then w"
            )

        # 안내 문구를 출력합니다.
        for idx, line in enumerate(lines):
            cv2.putText(frame, line, (20, 65 + idx * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)

        # 하단 정보입니다.
        info = f"manual={self.cfg.get('manual_speed', self.cfg['speed'])} auto={self.cfg.get('auto_speed', self.cfg['speed'])} pick={self.cfg.get('pick_speed', self.cfg['speed'])} step={self.cfg.get('tcp_step_mm', self.cfg['jog_mm'])} down={self.cfg['pick_down_mm']}"

        # 하단 정보 표시입니다.
        cv2.putText(frame, info, (20, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        # 프레임을 반환합니다.
        return frame

    # 현재 TCP 좌표를 가져오는 함수입니다.
    def get_coords(self):

        # 서버에서 좌표를 가져옵니다.
        coords = self.arm.get_coords()

        # 좌표 검증입니다.
        if not isinstance(coords, list) or len(coords) < 6:
            print("[ERROR] get_coords 실패:", coords)
            return None

        # 좌표 6개를 반환합니다.
        return coords[:6]

    # 좌표 이동 함수입니다.
    # 로봇팔을 지정 TCP 좌표로 이동시키는 함수입니다.
    def send_coords(self, coords, label="move", wait=0.0):

        # 좌표가 비정상이면 명령을 보내지 않습니다.
        if coords is None or len(coords) < 6:
            print("[ERROR] 잘못된 좌표:", coords)
            return False

        # 이동할 좌표를 출력합니다.
        print(
            f"[{label}] "
            f"{[round(float(v), 2) for v in coords[:6]]}"
        )

        # 기본 이동 속도를 읽습니다.
        move_speed = int(
            self.cfg.get(
                "auto_speed",
                self.cfg["speed"]
            )
        )

        # 집기 및 놓기 동작은 별도의 느린 속도를 사용합니다.
        if (
            label.startswith("pick")
            or label.startswith("place")
            or label.startswith("go_scan")
        ):
            move_speed = int(
                self.cfg.get(
                    "pick_speed",
                    move_speed
                )
            )

        # 서버에 좌표 이동 요청을 보냅니다.
        response = self.arm.send_coords(
            coords[:6],
            move_speed,
            1,
            wait
        )

        # 서버 응답이 비정상이면 실패 처리합니다.
        if not isinstance(response, dict):
            print(
                f"[MOVE ERROR] {label}: "
                f"서버 응답 형식 오류: {response}"
            )
            return False

        # 서버가 이동 실패를 반환하면 실패 처리합니다.
        if not response.get("ok", False):
            print(
                f"[MOVE ERROR] {label}: "
                f"{response}"
            )
            return False

        # 이동 요청 성공입니다.
        return True


    # 수동 명령 설정입니다.
    def set_manual_cmd(self, axis, delta):

        # 현재 시간입니다.
        now = time.time()

        # lock으로 보호합니다.
        with self.manual_lock:

            # 명령을 저장합니다.
            self.manual_cmd = (axis, float(delta))

            # 마지막 키 시간을 갱신합니다.
            self.last_manual_key_time = now

    # 수동 연속 이동 루프입니다.
    def teleop_loop(self):

        # 계속 반복합니다.
        while True:

            # 기본 명령입니다.
            cmd = None

            # 현재 시간입니다.
            now = time.time()

            # 수동 명령을 읽습니다.
            with self.manual_lock:

                # 마지막 키 입력 후 0.20초 안이면 누르고 있는 상태로 봅니다.
                if self.manual_cmd is not None and now - self.last_manual_key_time < 0.20:
                    cmd = self.manual_cmd

            # 키가 눌린 상태입니다.
            if cmd is not None:

                # 축과 이동량을 분리합니다.
                axis, delta = cmd

                # 그리퍼 TCP 기준으로 x/y/z 좌표만 이동합니다.
                self.arm.tcp_step(axis, delta, int(self.cfg.get("manual_speed", self.cfg["speed"])))

            # 짧게 대기합니다.
            time.sleep(0.055)


    # 백그라운드 작업 실행 함수입니다.
    def run_task(self, task_name, func, *args):

        # 작업 lock입니다.
        with self.task_lock:

            # 이미 작업 중이면 무시합니다.
            if self.task_busy:
                print(f"[BUSY] 이미 실행 중입니다: {task_name}")
                return

            # 작업 중으로 바꿉니다.
            self.task_busy = True

        # 작업 함수입니다.
        def worker():

            # 시작 로그입니다.
            print(f"[TASK START] {task_name}")

            try:

                # 실제 함수를 실행합니다.
                func(*args)

            except Exception as e:

                # 오류 출력입니다.
                print(f"[TASK ERROR] {task_name}: {e}")

            finally:

                # 작업 상태를 해제합니다.
                with self.task_lock:
                    self.task_busy = False

                # 종료 로그입니다.
                print(f"[TASK DONE] {task_name}")

        # 스레드를 생성합니다.
        thread = threading.Thread(target=worker, daemon=True)

        # 스레드를 시작합니다.
        thread.start()

    # 직사각형 중심 맞추기 1회입니다.
    # 여러 프레임에서 안정적인 컨테이너 중심을 측정합니다.
    def get_stable_detection_center(self, attempts=20):

        # 검출된 중심 좌표를 저장합니다.
        centers = []

        # 여러 프레임을 확인합니다.
        for _ in range(attempts):

            # 최신 보정 영상을 읽습니다.
            frame = self.read_frame()

            # 영상이 없으면 잠시 대기합니다.
            if frame is None:
                time.sleep(0.05)
                continue

            # 컨테이너를 검출합니다.
            detection, _ = self.detect_rectangle_object(frame)

            # 검출된 경우 중심을 저장합니다.
            if detection is not None:
                centers.append(detection["center"])

            # 다섯 프레임 이상 모이면 중앙값을 사용합니다.
            if len(centers) >= 5:

                # X 좌표 중앙값입니다.
                center_u = float(
                    np.median([point[0] for point in centers])
                )

                # Y 좌표 중앙값입니다.
                center_v = float(
                    np.median([point[1] for point in centers])
                )

                # 안정적인 중심 좌표를 반환합니다.
                return center_u, center_v

            # 다음 프레임을 기다립니다.
            time.sleep(0.06)

        # 검출 실패입니다.
        return None

    # 카메라 영상 축과 로봇 X/Y축 관계를 자동으로 측정합니다.
    def calibrate_xy_mapping(self):

        # 자동 집기가 켜져 있으면 보정 중에는 끕니다.
        self.cfg["auto_pick_enabled"] = False

        # 설정을 저장합니다.
        save_config(self.cfg)

        # 시험 이동 거리입니다.
        test_mm = float(
            self.cfg.get("xy_calibration_step_mm", 5.0)
        )

        # 시험 이동 속도입니다.
        speed = int(
            self.cfg.get("xy_calibration_speed", 30)
        )

        # 현재 로봇 좌표를 읽습니다.
        origin = self.get_coords()

        # 좌표를 읽지 못하면 중단합니다.
        if origin is None:
            print("[XY CAL ERROR] 현재 로봇 좌표 읽기 실패")
            return False

        # 현재 좌표를 복사합니다.
        origin = origin[:6]

        # 시작 영상에서 컨테이너 중심을 측정합니다.
        base_center = self.get_stable_detection_center()

        # 컨테이너를 찾지 못하면 중단합니다.
        if base_center is None:
            print("[XY CAL ERROR] 컨테이너 중심 검출 실패")
            return False

        # 시작 중심 좌표입니다.
        base_u, base_v = base_center

        # 시작 중심을 출력합니다.
        print(
            "[XY CAL] 시작 중심: "
            f"u={base_u:.2f}, v={base_v:.2f}"
        )

        # ------------------------------
        # 로봇 X축 +5mm 시험 이동
        # ------------------------------

        # X축 시험 좌표입니다.
        x_test_coords = origin[:6]

        # X축으로 시험 거리만큼 이동합니다.
        x_test_coords[0] += test_mm

        # X축 이동 명령을 보냅니다.
        response = self.arm.send_coords(
            x_test_coords,
            speed,
            mode=1,
            wait=0.7
        )

        # 이동 실패를 확인합니다.
        if not isinstance(response, dict) or not response.get("ok", False):
            print("[XY CAL ERROR] X축 시험 이동 실패:", response)
            return False

        # 영상이 안정될 때까지 기다립니다.
        time.sleep(0.5)

        # X축 이동 후 컨테이너 중심을 측정합니다.
        x_center = self.get_stable_detection_center()

        # 원래 위치로 돌아갑니다.
        self.arm.send_coords(
            origin,
            speed,
            mode=1,
            wait=0.7
        )

        # 원위치 안정화를 기다립니다.
        time.sleep(0.5)

        # X축 이동 후 검출 실패입니다.
        if x_center is None:
            print("[XY CAL ERROR] X축 이동 후 컨테이너 검출 실패")
            return False

        # ------------------------------
        # 로봇 Y축 +5mm 시험 이동
        # ------------------------------

        # Y축 시험 좌표입니다.
        y_test_coords = origin[:6]

        # Y축으로 시험 거리만큼 이동합니다.
        y_test_coords[1] += test_mm

        # Y축 이동 명령을 보냅니다.
        response = self.arm.send_coords(
            y_test_coords,
            speed,
            mode=1,
            wait=0.7
        )

        # 이동 실패를 확인합니다.
        if not isinstance(response, dict) or not response.get("ok", False):
            print("[XY CAL ERROR] Y축 시험 이동 실패:", response)
            return False

        # 영상이 안정될 때까지 기다립니다.
        time.sleep(0.5)

        # Y축 이동 후 컨테이너 중심을 측정합니다.
        y_center = self.get_stable_detection_center()

        # 원래 위치로 돌아갑니다.
        self.arm.send_coords(
            origin,
            speed,
            mode=1,
            wait=0.7
        )

        # 원위치 안정화를 기다립니다.
        time.sleep(0.5)

        # Y축 이동 후 검출 실패입니다.
        if y_center is None:
            print("[XY CAL ERROR] Y축 이동 후 컨테이너 검출 실패")
            return False

        # X축 이동 후 픽셀 중심입니다.
        x_u, x_v = x_center

        # Y축 이동 후 픽셀 중심입니다.
        y_u, y_v = y_center

        # 로봇 X축 1mm 이동당 영상 픽셀 변화량입니다.
        du_dx = (x_u - base_u) / test_mm
        dv_dx = (x_v - base_v) / test_mm

        # 로봇 Y축 1mm 이동당 영상 픽셀 변화량입니다.
        du_dy = (y_u - base_u) / test_mm
        dv_dy = (y_v - base_v) / test_mm

        # 로봇 이동량을 영상 픽셀 변화량으로 바꾸는 행렬입니다.
        jacobian = np.array(
            [
                [du_dx, du_dy],
                [dv_dx, dv_dy]
            ],
            dtype=float
        )

        # 행렬이 역변환 가능한지 확인합니다.
        determinant = float(np.linalg.det(jacobian))

        # 행렬이 거의 0이면 측정에 실패한 것입니다.
        if abs(determinant) < 0.01:
            print(
                "[XY CAL ERROR] 방향 행렬 계산 실패:",
                jacobian.tolist()
            )
            return False

        # 계산한 행렬을 설정에 저장합니다.
        self.cfg["pixel_jacobian"] = jacobian.tolist()

        # 설정 파일에 저장합니다.
        save_config(self.cfg)

        # 결과를 출력합니다.
        print("[XY CAL OK] 자동 방향 보정 완료")
        print("[XY CAL MATRIX]", jacobian.tolist())
        print("[XY CAL DET]", determinant)
        print("[XY CAL] 로봇이 원래 위치로 복귀했습니다.")

        # 성공을 반환합니다.
        return True

    # 컨테이너 중심을 목표 픽셀로 한 번 이동시키는 함수입니다.
    def center_once(self, direction_sign=None, target_pixel=None, use_simple=False):

        # 최신 보정 영상을 읽습니다.
        frame = self.read_frame()

        # 영상이 없으면 실패입니다.
        if frame is None:
            print("[ERROR] frame 없음")
            return False

        # 컨테이너를 검출합니다.
        detection, _ = self.detect_rectangle_object(frame)

        # 컨테이너를 찾지 못하면 실패입니다.
        if detection is None:
            print("[WARN] 컨테이너를 찾지 못했습니다.")
            return False

        # 영상 크기를 가져옵니다.
        height, width = frame.shape[:2]

        # 목표 픽셀을 가져옵니다. z 영점 맞춤은 저장된 십자가가 아니라 화면 중앙을 씁니다.
        if (
            isinstance(target_pixel, (list, tuple))
            and len(target_pixel) >= 2
        ):
            target_u = float(target_pixel[0])
            target_v = float(target_pixel[1])
        else:
            target_u, target_v = self.get_target_pixel(
                width,
                height
            )

        # 검출된 컨테이너 중심입니다.
        center_u, center_v = detection["center"]

        # 영상 X축 오차입니다.
        error_u = float(center_u - target_u)

        # 영상 Y축 오차입니다.
        error_v = float(center_v - target_v)

        # 현재 오차를 출력합니다.
        print(
            "[CENTER] "
            f"err_px=({error_u:.1f},{error_v:.1f}) "
            f"confidence="
            f"{float(detection.get('confidence', 0.0)) * 100.0:.1f}%"
        )

        # 허용 오차입니다.
        tolerance = float(
            self.cfg.get("center_tolerance_px", 18)
        )

        # 중심 오차가 충분히 작으면 완료입니다.
        if (
            abs(error_u) < tolerance
            and abs(error_v) < tolerance
        ):
            print("[CENTER] 정렬 완료")

            # 현재 위치를 물체 위치로 저장합니다.
            self.remember_current_object_pose(
                "auto_center"
            )

            # 정렬 완료입니다.
            return True

        # 현재 로봇 좌표를 읽습니다.
        coords = self.get_coords()

        # 좌표 읽기 실패입니다.
        if coords is None:
            return False

        jacobian = None

        if not bool(use_simple):
            # 저장된 방향 변환 행렬을 가져옵니다.
            matrix_data = self.cfg.get(
                "pixel_jacobian",
                None
            )

            if matrix_data is not None:
                # 방향 행렬을 numpy 배열로 변환합니다.
                jacobian = np.array(
                    matrix_data,
                    dtype=float
                )

        # 현재 픽셀 오차 벡터입니다.
        # 영상의 좌우 오차만 반대로 적용합니다.
        # 물체가 화면 오른쪽에 있으면 기존과 반대 방향으로 이동합니다.
        pixel_error = np.array(
            [error_u, error_v],
            dtype=float
        )

        # 이동 게인입니다.
        gain = float(
            self.cfg.get("center_gain", 0.70)
        )

        if direction_sign is None:
            direction_sign = float(
                self.cfg.get(
                    "center_direction_sign",
                    1.0
                )
            )
        else:
            direction_sign = float(direction_sign)

        try:
            # 픽셀 오차를 줄이는 로봇 X/Y 이동량을 계산합니다.
            if jacobian is None:
                raise np.linalg.LinAlgError()

            robot_delta = (
                -np.linalg.inv(jacobian)
                @ pixel_error
                * gain
                * direction_sign
            )

        except np.linalg.LinAlgError:
            print("[CENTER ERROR] 방향 행렬 역변환 실패")
            robot_delta = np.array(
                [
                    float(
                        self.cfg.get(
                            "zero_focus_simple_x_from_image_y",
                            -0.025
                        )
                    )
                    * error_v,
                    float(
                        self.cfg.get(
                            "zero_focus_simple_y_from_image_x",
                            -0.025
                        )
                    )
                    * error_u,
                ],
                dtype=float
            ) * gain * direction_sign

        # 계산된 로봇 X축 이동량입니다.
        delta_x = float(robot_delta[0])

        # 계산된 로봇 Y축 이동량입니다.
        delta_y = float(robot_delta[1])

        # 한 번에 이동할 최대 거리입니다.
        max_step = float(
            self.cfg.get("max_step_mm", 6.0)
        )

        # X축 이동량을 제한합니다.
        delta_x = max(
            -max_step,
            min(max_step, delta_x)
        )

        # Y축 이동량을 제한합니다.
        delta_y = max(
            -max_step,
            min(max_step, delta_y)
        )

        # CENTER_SMOOTH_MIN_STEP
        # 목표에서 충분히 멀리 떨어졌는데 계산 이동량이 너무 작으면
        # 반복적으로 끊어 움직이지 않도록 최소 이동량을 적용합니다.
        minimum_center_step = float(
            self.cfg.get(
                "minimum_center_step_mm",
                1.5
            )
        )

        # 목표 오차가 허용값의 두 배보다 큰 경우에만 최소 이동량을 적용합니다.
        far_from_target = (
            abs(error_u) > tolerance * 2.0
            or abs(error_v) > tolerance * 2.0
        )

        # X축 이동량이 지나치게 작으면 기존 방향을 유지한 채 키웁니다.
        if (
            far_from_target
            and 0.0 < abs(delta_x) < minimum_center_step
        ):
            delta_x = (
                minimum_center_step
                if delta_x > 0.0
                else -minimum_center_step
            )

        # Y축 이동량이 지나치게 작으면 기존 방향을 유지한 채 키웁니다.
        if (
            far_from_target
            and 0.0 < abs(delta_y) < minimum_center_step
        ):
            delta_y = (
                minimum_center_step
                if delta_y > 0.0
                else -minimum_center_step
            )

        # 현재 좌표에 계산된 이동량을 더합니다.
        coords[0] += delta_x
        coords[1] += delta_y

        # 이동량을 출력합니다.
        print(
            "[CENTER MATRIX] "
            f"dx={delta_x:.2f}, "
            f"dy={delta_y:.2f} "
            f"sign={direction_sign:+.0f}"
        )

        # 로봇을 이동합니다.
        moved = self.send_coords(
            coords,
            "center_matrix",
            wait=0.25
        )

        # 이동 명령 결과를 반환합니다.
        return False if moved else False



    # 자동 중심 맞추기입니다.
    def auto_center(
        self,
        max_iter=5,
        direction_sign=None,
        target_pixel=None,
        use_simple=False
    ):

        # 여러 번 반복합니다.
        for _ in range(max_iter):

            # 1회 중심 맞추기입니다.
            done = self.center_once(
                direction_sign=direction_sign,
                target_pixel=target_pixel,
                use_simple=use_simple
            )

            # 완료되면 True입니다.
            if done:
                return True

            # 짧게 대기합니다.
            time.sleep(0.15)

        # 실패 출력입니다.
        print("[CENTER] 최대 반복 초과")
        return False

    # 자동 집기입니다.
    # 물체 방향으로 Z축을 여러 단계로 나누어 하강합니다.
    # 현재 그리퍼 높이를 실제 컨테이너 집기 높이로 저장합니다.
    def save_pick_target_z(self):

        # 현재 로봇팔 TCP 좌표를 읽습니다.
        coords = self.get_coords()

        # 좌표 읽기 실패 시 저장하지 않습니다.
        if coords is None:
            print("[PICK Z ERROR] 현재 좌표 읽기 실패")
            return False

        # 현재 Z 좌표를 실제 집기 높이로 저장합니다.
        self.cfg["pick_target_z_mm"] = round(
            float(coords[2]),
            2
        )

        # 설정 파일에 저장합니다.
        save_config(self.cfg)

        # 저장 결과를 출력합니다.
        print(
            "[PICK Z SAVED] "
            f"pick_target_z_mm={self.cfg['pick_target_z_mm']:.2f}"
        )

        # 성공을 반환합니다.
        return True

    # 컨테이너 위로 빠르게 접근한 뒤 마지막 구간만 단계 하강합니다.
    # 카메라 깊이를 확인하며 컨테이너까지 단계적으로 하강합니다.
    def move_down_stepwise(self, approach_coords, down_mm):

        # 시작 좌표를 복사합니다.
        start_coords = approach_coords[:6]

        # 시작 Z 좌표입니다.
        start_z = float(start_coords[2])

        # 현재 명령 좌표입니다.
        current_coords = start_coords[:6]

        # 안전상 허용할 최소 Z 좌표입니다.
        minimum_safe_z = float(
            self.cfg.get(
                "minimum_safe_z_mm",
                25.0
            )
        )

        # 카메라와 컨테이너 사이의 목표 깊이입니다.
        target_depth_mm = float(
            self.cfg.get(
                "visual_descent_target_depth_mm",
                120.0
            )
        )

        # 깊이 차이를 실제 로봇 Z 이동량으로 바꾸는 비율입니다.
        depth_to_z_gain = float(
            self.cfg.get(
                "visual_depth_to_z_gain",
                0.45
            )
        )

        # 한 번에 내려갈 최소 거리입니다.
        minimum_step_mm = abs(
            float(
                self.cfg.get(
                    "visual_descent_min_step_mm",
                    3.0
                )
            )
        )

        # 한 번에 내려갈 최대 거리입니다.
        maximum_step_mm = abs(
            float(
                self.cfg.get(
                    "visual_descent_max_step_mm",
                    10.0
                )
            )
        )

        # 한 번의 집기에서 허용할 최대 전체 하강 거리입니다.
        maximum_total_down_mm = abs(
            float(
                self.cfg.get(
                    "visual_descent_max_total_mm",
                    55.0
                )
            )
        )

        # 목표 깊이에 대한 허용 오차입니다.
        depth_tolerance_mm = abs(
            float(
                self.cfg.get(
                    "visual_descent_tolerance_mm",
                    8.0
                )
            )
        )

        # 최대 하강 반복 횟수입니다.
        maximum_iterations = int(
            self.cfg.get(
                "visual_descent_max_iter",
                10
            )
        )

        # 컨테이너가 그리퍼에 가려졌을 때 마지막으로 조금 더 내려갈 거리입니다.
        blind_finish_mm = abs(
            float(
                self.cfg.get(
                    "visual_descent_blind_finish_mm",
                    5.0
                )
            )
        )

        # 현재까지 내려간 전체 거리입니다.
        total_down_mm = 0.0

        # 마지막 정상 깊이값입니다.
        last_valid_depth_mm = None

        # 깊이값이 사라진 연속 횟수입니다.
        lost_depth_count = 0

        print(
            "[VISUAL DESCENT] 시작 | "
            f"start_z={start_z:.2f} | "
            f"target_depth={target_depth_mm:.2f}mm | "
            f"max_down={maximum_total_down_mm:.2f}mm"
        )

        for iteration in range(1, maximum_iterations + 1):

            # 최신 보정 영상을 읽습니다.
            frame = self.read_frame()

            # 이번 반복에서 사용할 깊이값입니다.
            current_depth_mm = None

            # 영상이 있으면 컨테이너를 다시 검출합니다.
            if frame is not None:

                # 컨테이너 검출과 깊이 계산입니다.
                detection, _ = self.detect_rectangle_object(frame)

                # 검출 결과에 깊이가 있으면 사용합니다.
                if detection is not None:
                    detected_depth = detection.get(
                        "depth_mm",
                        None
                    )

                    if detected_depth is not None:
                        current_depth_mm = float(
                            detected_depth
                        )

            # 현재 프레임에서 깊이를 못 얻으면 최근 깊이 캐시를 확인합니다.
            if current_depth_mm is None:

                cached_depth = getattr(
                    self,
                    "latest_depth_mm",
                    None
                )

                cached_time = float(
                    getattr(
                        self,
                        "latest_depth_time",
                        0.0
                    )
                )

                if (
                    cached_depth is not None
                    and time.time() - cached_time
                    <= float(
                        self.cfg.get(
                            "depth_max_age_sec",
                            3.0
                        )
                    )
                ):
                    current_depth_mm = float(
                        cached_depth
                    )

            # 정상 깊이값인지 확인합니다.
            if current_depth_mm is not None:

                valid_min_depth = float(
                    self.cfg.get(
                        "depth_min_mm",
                        80.0
                    )
                )

                valid_max_depth = float(
                    self.cfg.get(
                        "depth_max_mm",
                        700.0
                    )
                )

                if not (
                    valid_min_depth
                    <= current_depth_mm
                    <= valid_max_depth
                ):

                    print(
                        "[VISUAL DESCENT WARNING] "
                        f"깊이 범위 오류: {current_depth_mm:.2f}mm"
                    )

                    current_depth_mm = None

            # 유효한 깊이값이 있는 경우입니다.
            if current_depth_mm is not None:

                lost_depth_count = 0

                previous_depth_mm = last_valid_depth_mm

                last_valid_depth_mm = current_depth_mm

                depth_error_mm = (
                    current_depth_mm
                    - target_depth_mm
                )

                print(
                    "[VISUAL DESCENT] "
                    f"{iteration}/{maximum_iterations} | "
                    f"depth={current_depth_mm:.2f}mm | "
                    f"error={depth_error_mm:.2f}mm | "
                    f"down={total_down_mm:.2f}mm"
                )

                # 목표 깊이에 충분히 가까워졌으면 하강을 끝냅니다.
                if depth_error_mm <= depth_tolerance_mm:

                    print(
                        "[VISUAL DESCENT OK] "
                        "컨테이너 집기 깊이에 도달했습니다."
                    )

                    break

                # 하강 중 깊이가 비정상적으로 증가하면 중단합니다.
                if (
                    previous_depth_mm is not None
                    and current_depth_mm
                    > previous_depth_mm
                    + float(
                        self.cfg.get(
                            "visual_depth_increase_abort_mm",
                            35.0
                        )
                    )
                ):

                    print(
                        "[VISUAL DESCENT STOP] "
                        "하강 중 측정 깊이가 비정상적으로 증가했습니다."
                    )

                    return None

                # 깊이 차이를 Z 하강량으로 변환합니다.
                step_mm = (
                    depth_error_mm
                    * depth_to_z_gain
                )

                # 최소/최대 한 단계 이동량으로 제한합니다.
                step_mm = max(
                    minimum_step_mm,
                    min(
                        maximum_step_mm,
                        step_mm
                    )
                )

            # 깊이를 잃은 경우입니다.
            else:

                lost_depth_count += 1

                print(
                    "[VISUAL DESCENT WARNING] "
                    f"깊이 인식 손실 {lost_depth_count}회"
                )

                # 한 번 정도의 순간 손실은 재검출합니다.
                if lost_depth_count <= 1:
                    time.sleep(0.12)
                    continue

                # 컨테이너 가까이에서 가려졌다면 제한된 거리만 추가 하강합니다.
                if (
                    last_valid_depth_mm is not None
                    and last_valid_depth_mm
                    <= target_depth_mm
                    + float(
                        self.cfg.get(
                            "visual_blind_finish_start_margin_mm",
                            35.0
                        )
                    )
                ):

                    step_mm = blind_finish_mm

                    print(
                        "[VISUAL DESCENT BLIND FINISH] "
                        f"마지막 {step_mm:.2f}mm 하강"
                    )

                # 멀리 있는 상태에서 깊이를 잃으면 중단합니다.
                else:

                    print(
                        "[VISUAL DESCENT STOP] "
                        "컨테이너가 먼 상태에서 깊이 인식을 잃었습니다."
                    )

                    return None

            # 남은 최대 하강 거리를 계산합니다.
            remaining_down_mm = (
                maximum_total_down_mm
                - total_down_mm
            )

            # 더 내려갈 안전 여유가 없으면 중단합니다.
            if remaining_down_mm <= 0.5:

                print(
                    "[VISUAL DESCENT STOP] "
                    "최대 하강 거리에 도달했습니다."
                )

                break

            # 이번 단계가 최대 전체 하강량을 넘지 않게 제한합니다.
            step_mm = min(
                step_mm,
                remaining_down_mm
            )

            # 현재 실제 좌표를 다시 읽습니다.
            measured_coords = self.get_coords()

            if (
                isinstance(measured_coords, list)
                and len(measured_coords) >= 6
            ):
                current_coords = measured_coords[:6]

            # 다음 목표 좌표를 만듭니다.
            next_coords = current_coords[:6]

            # Z축으로 아래쪽 이동합니다.
            next_target_z = (
                float(current_coords[2])
                - step_mm
            )

            # 안전 최소 Z 아래로 내려가지 않게 제한합니다.
            next_target_z = max(
                minimum_safe_z,
                next_target_z
            )

            # 실제로 이동 가능한 거리입니다.
            actual_step_mm = (
                float(current_coords[2])
                - next_target_z
            )

            # 이동 거리가 너무 작으면 중단합니다.
            if actual_step_mm <= 0.5:

                print(
                    "[VISUAL DESCENT STOP] "
                    "minimum_safe_z 제한에 도달했습니다."
                )

                break

            # 목표 Z를 적용합니다.
            next_coords[2] = next_target_z

            print(
                "[VISUAL DESCENT MOVE] "
                f"z={float(current_coords[2]):.2f} "
                f"-> {next_target_z:.2f} "
                f"({actual_step_mm:.2f}mm)"
            )

            # 로봇을 하강시킵니다.
            moved = self.send_coords(
                next_coords,
                "visual_pick_down",
                wait=float(
                    self.cfg.get(
                        "visual_descent_move_wait_sec",
                        0.30
                    )
                )
            )

            if not moved:

                print(
                    "[VISUAL DESCENT ERROR] "
                    "로봇 하강 명령 실패"
                )

                return None

            # 현재 명령 좌표를 갱신합니다.
            current_coords = next_coords[:6]

            # 전체 하강 거리를 누적합니다.
            total_down_mm += actual_step_mm

            # 이동 후 안정화 시간을 기다립니다.
            time.sleep(
                float(
                    self.cfg.get(
                        "visual_descent_settle_sec",
                        0.18
                    )
                )
            )

        # 최종 실제 좌표를 읽습니다.
        final_coords = self.get_coords()

        # 좌표 읽기 실패 시 마지막 명령 좌표를 사용합니다.
        if (
            not isinstance(final_coords, list)
            or len(final_coords) < 6
        ):
            final_coords = current_coords[:6]

        # 실제 하강 거리가 너무 작으면 집기하지 않습니다.
        minimum_required_down_mm = float(
            self.cfg.get(
                "visual_descent_min_total_mm",
                12.0
            )
        )

        if total_down_mm < minimum_required_down_mm:

            print(
                "[VISUAL DESCENT ERROR] "
                f"하강 거리가 부족합니다: {total_down_mm:.2f}mm"
            )

            return None

        print(
            "[VISUAL DESCENT COMPLETE] "
            f"total_down={total_down_mm:.2f}mm | "
            f"final_z={float(final_coords[2]):.2f}"
        )

        return final_coords[:6]




    # 물체를 자동으로 집는 전체 동작입니다.
    def pick(self):

        # 자동 집기 시작 로그입니다.
        print("[AUTO PICK] 자동 집기 시작")

        # 보조 접근 단계에서 이미 물체 근처까지 이동한 경우입니다.
        if getattr(self, "skip_auto_center_once", False):

            # 이번 자동 집기에서는 잘못된 보정 행렬 재사용을 막습니다.
            centered = True

            # 다음 실행에는 다시 기본 상태가 되도록 초기화합니다.
            self.skip_auto_center_once = False

            print(
                "[AUTO PICK] "
                "보조 접근 완료: 이번에는 정밀 중심 정렬을 생략합니다."
            )

        else:

            # 일반 실행에서는 기존 정밀 중심 정렬을 사용합니다.
            centered = self.auto_center(
                max_iter=int(
                    self.cfg.get(
                        "auto_center_max_iter",
                        10
                    )
                )
            )

        # 중심 정렬이 실패하면 재시도 가능 상태로 돌립니다.
        if not centered:
            print("[AUTO PICK ERROR] 중심 정렬 실패")
            self.auto_pick_latched = False
            return False

        # 정렬 완료 후 현재 TCP 좌표를 읽습니다.
        approach_coords = self.get_coords()

        # 현재 좌표를 읽지 못하면 중단합니다.
        if approach_coords is None:
            print(
                "[AUTO PICK ERROR] "
                "현재 TCP 좌표 읽기 실패"
            )
            self.auto_pick_latched = False
            return False

        # 좌표 6개를 복사합니다.
        approach_coords = approach_coords[:6]

        # 설정된 전체 하강 거리입니다.
        down_mm = float(
            self.cfg.get(
                "auto_pick_down_mm",
                self.cfg.get(
                    "pick_down_mm",
                    38.0
                )
            )
        )

        # 잡은 후 상승할 거리입니다.
        lift_mm = float(
            self.cfg.get(
                "auto_lift_mm",
                70.0
            )
        )

        # 하강 전에 그리퍼를 엽니다.
        print("[AUTO PICK] 그리퍼 열기")

        # 그리퍼 열기 명령을 보냅니다.
        open_response = self.arm.set_gripper_value(
            int(self.cfg["gripper_open"]),
            int(self.cfg["gripper_speed"]),
            wait=0.7
        )

        # 그리퍼 열기 실패 시 중단합니다.
        if not open_response.get("ok", False):
            print(
                "[AUTO PICK ERROR] "
                "그리퍼 열기 실패"
            )
            self.auto_pick_latched = False
            return False

        # 물체를 향해 단계적으로 하강합니다.
        down_coords = self.move_down_stepwise(
            approach_coords,
            down_mm
        )

        # 하강 실패 시 그리퍼를 닫지 않습니다.
        if down_coords is None:
            print(
                "[AUTO PICK ERROR] "
                "물체 방향 하강 실패"
            )
            self.auto_pick_latched = False
            return False

        # 물체 위치에서 그리퍼를 닫습니다.
        print("[AUTO PICK] 그리퍼 닫기")

        # 그리퍼 닫기 명령을 보냅니다.
        close_response = self.arm.set_gripper_value(
            int(self.cfg["gripper_close"]),
            int(self.cfg["gripper_speed"]),
            wait=1.0
        )

        # 그리퍼 닫기 실패 시 중단합니다.
        if not close_response.get("ok", False):
            print(
                "[AUTO PICK ERROR] "
                "그리퍼 닫기 실패"
            )
            self.auto_pick_latched = False
            return False

        # 집게가 물체를 잡을 시간을 기다립니다.
        time.sleep(0.4)

        # 상승 좌표를 만듭니다.
        lift_coords = down_coords[:6]

        # 기존 접근 위치 이상으로 상승하도록 계산합니다.
        lift_coords[2] = max(
            float(approach_coords[2]),
            float(down_coords[2]) + lift_mm
        )

        # 물체를 들어 올립니다.
        print(
            "[AUTO PICK] 자동 상승: "
            f"목표 z={lift_coords[2]:.2f}"
        )

        # 상승 명령을 보냅니다.
        lifted = self.send_coords(
            lift_coords,
            "pick_lift",
            wait=float(
                self.cfg.get(
                    "auto_lift_wait_sec",
                    1.2
                )
            )
        )

        # 상승 실패를 확인합니다.
        if not lifted:
            print(
                "[AUTO PICK ERROR] "
                "물체 상승 실패"
            )
            self.auto_pick_latched = False
            return False

        # 상승 안정화를 기다립니다.
        time.sleep(
            float(
                self.cfg.get(
                    "auto_lift_settle_sec",
                    0.5
                )
            )
        )

        # 자동 집기 완료입니다.
        print("[AUTO PICK] 자동 집기 완료")

        # 성공을 반환합니다.
        return True



    # 놓기입니다.
    def place(self):

        # place 좌표가 없으면 종료합니다.
        if self.cfg["place_coords"] is None:
            print("[PLACE] place_coords 없음. 놓을 위치에서 p를 누르세요.")
            return

        # place 좌표입니다.
        place_coords = self.cfg["place_coords"][:6]

        # 위쪽 좌표입니다.
        above_coords = place_coords[:6]

        # 위쪽으로 올립니다.
        above_coords[2] += float(self.cfg["lift_up_mm"])

        # 놓을 위치 위로 이동합니다.
        self.send_coords(above_coords, "place_above", wait=0.8)

        # 내려갑니다.
        self.send_coords(place_coords, "place_down", wait=0.8)

        # 그리퍼를 엽니다.
        print("[PLACE] gripper open")
        if not self.open_gripper():
            print("[PLACE ERROR] 그리퍼 열기 실패")
            return False

        # 다시 위로 올라갑니다.
        self.send_coords(above_coords, "place_lift", wait=0.8)

        # 완료입니다.
        print("[PLACE] 완료")
        return True

    # scan 좌표 저장입니다.
    def save_scan_pose(self):

        # 현재 좌표를 읽습니다.
        coords = self.get_coords()

        # 좌표가 정상이면 저장합니다.
        if coords is not None:
            self.cfg["scan_coords"] = coords
            save_config(self.cfg)
            print("[SAVE] scan_coords:", coords)

    # scan 좌표로 이동합니다.
    def go_scan_pose(self):

        # scan 좌표가 없으면 종료합니다.
        if self.cfg["scan_coords"] is None:
            print("[SCAN] scan_coords 없음. s로 먼저 저장하세요.")
            return

        # scan 좌표로 이동합니다.
        self.send_coords(self.cfg["scan_coords"][:6], "go_scan", wait=0.8)

    # place 좌표 저장입니다.
    def save_place_pose(self):

        # 현재 좌표를 읽습니다.
        coords = self.get_coords()

        # 좌표가 정상이면 저장합니다.
        if coords is not None:
            self.cfg["place_coords"] = coords
            save_config(self.cfg)
            print("[SAVE] place_coords:", coords)

    # 현재 직사각형 중심을 화면 십자가와 직접 집기 영점으로 저장합니다.
    def save_target_pixel(self, use_image_center=False):

        # 프레임을 읽습니다.
        frame = self.read_frame()

        # 실패 시 종료합니다.
        if frame is None:
            print("[ERROR] frame 없음")
            return

        if bool(use_image_center):
            height, width = frame.shape[:2]
            cx = width / 2.0
            cy = height / 2.0
        else:
            # 여러 프레임 중앙값을 우선 사용해서 흔들림을 줄입니다.
            stable_center = self.get_stable_detection_center(
                attempts=int(
                    self.cfg.get(
                        "zero_calibration_sample_count",
                        18
                    )
                )
            )

            detection = None

            if stable_center is None:
                # 직사각형 검출입니다.
                detection, _ = self.detect_rectangle_object(frame)

                # 검출 실패 시 종료합니다.
                if detection is None:
                    print("[ZERO CALIB ERROR] 컨테이너 인식값 없음")
                    return False

                # 중심을 가져옵니다.
                cx, cy = detection["center"]
            else:
                cx, cy = stable_center

        cx = float(cx)
        cy = float(cy)

        # 화면 십자가 기준점과 직접 집기 계산 기준점을 같이 저장합니다.
        self.cfg["target_u"] = cx
        self.cfg["target_v"] = cy
        self.cfg["direct_pick_target_u"] = cx
        self.cfg["direct_pick_target_v"] = cy
        self.cfg["last_zero_focus_calibration"] = {
            "target_u": round(cx, 3),
            "target_v": round(cy, 3),
            "source": (
                "image_center"
                if bool(use_image_center)
                else "container_center"
            ),
            "timestamp": round(time.time(), 3)
        }

        # 설정 저장입니다.
        save_config(self.cfg)

        # 출력입니다.
        print(
            "[ZERO CALIB] "
            f"screen/direct_pick target=({cx:.1f}, {cy:.1f})"
        )
        print(
            "[ZERO CALIB] "
            "빨간 십자가와 현재 컨테이너 중심을 영점으로 맞췄습니다."
        )
        return True


    # z 영점 이동 방향이 맞는지 작은 시험 이동으로 자동 판별합니다.
    def auto_tune_zero_focus_direction(self, target_pixel):

        if not bool(
            self.cfg.get(
                "zero_focus_auto_direction_tune",
                True
            )
        ):
            return True

        frame = self.read_frame()

        if frame is None:
            print("[ZERO FOCUS TUNE] 카메라 영상 없음")
            return False

        detection, _ = self.detect_rectangle_object(frame)

        if not isinstance(detection, dict):
            print("[ZERO FOCUS TUNE] 컨테이너 인식값 없음")
            return False

        center = detection.get("center", None)

        if (
            not isinstance(center, (list, tuple))
            or len(center) < 2
        ):
            print("[ZERO FOCUS TUNE] 컨테이너 중심값 없음")
            return False

        target_u = float(target_pixel[0])
        target_v = float(target_pixel[1])
        before_error_u = float(center[0]) - target_u
        before_error_v = float(center[1]) - target_v
        before_error = (
            before_error_u ** 2
            + before_error_v ** 2
        ) ** 0.5

        if before_error < float(
            self.cfg.get(
                "center_tolerance_px",
                18.0
            )
        ):
            return True

        coords = self.get_coords()

        if (
            not isinstance(coords, list)
            or len(coords) < 6
        ):
            print("[ZERO FOCUS TUNE] 현재 좌표 읽기 실패")
            return False

        start_coords = [
            float(value)
            for value in coords[:6]
        ]

        x_gain = float(
            self.cfg.get(
                "zero_focus_simple_x_from_image_y",
                0.025
            )
        )
        y_gain = float(
            self.cfg.get(
                "zero_focus_simple_y_from_image_x",
                0.025
            )
        )
        sign = float(
            self.cfg.get(
                "zero_focus_center_direction_sign",
                1.0
            )
        )
        probe_scale = float(
            self.cfg.get(
                "zero_focus_probe_scale",
                0.35
            )
        )
        max_probe_mm = abs(
            float(
                self.cfg.get(
                    "zero_focus_probe_max_mm",
                    2.5
                )
            )
        )

        probe_dx = x_gain * before_error_v * sign * probe_scale
        probe_dy = y_gain * before_error_u * sign * probe_scale
        probe_dx = max(-max_probe_mm, min(max_probe_mm, probe_dx))
        probe_dy = max(-max_probe_mm, min(max_probe_mm, probe_dy))

        if abs(probe_dx) < 0.05 and abs(probe_dy) < 0.05:
            return True

        probe_coords = start_coords[:6]
        probe_coords[0] += probe_dx
        probe_coords[1] += probe_dy

        print(
            "[ZERO FOCUS TUNE] "
            f"probe dx={probe_dx:+.2f}, dy={probe_dy:+.2f}, "
            f"before={before_error:.1f}px"
        )

        moved = self.send_coords(
            probe_coords,
            "zero_focus_direction_probe",
            wait=float(
                self.cfg.get(
                    "zero_focus_probe_wait_sec",
                    0.25
                )
            )
        )

        if not moved:
            print("[ZERO FOCUS TUNE] 시험 이동 실패")
            return False

        time.sleep(
            float(
                self.cfg.get(
                    "zero_focus_probe_settle_sec",
                    0.12
                )
            )
        )

        after_frame = self.read_frame()
        after_detection = None

        if after_frame is not None:
            after_detection, _ = self.detect_rectangle_object(after_frame)

        after_error = before_error

        if isinstance(after_detection, dict):
            after_center = after_detection.get("center", None)

            if (
                isinstance(after_center, (list, tuple))
                and len(after_center) >= 2
            ):
                after_error_u = float(after_center[0]) - target_u
                after_error_v = float(after_center[1]) - target_v
                after_error = (
                    after_error_u ** 2
                    + after_error_v ** 2
                ) ** 0.5

        # 시험 이동 후 원위치로 복귀하고, 실제 정렬은 판별된 방향으로 다시 수행합니다.
        self.send_coords(
            start_coords,
            "zero_focus_direction_probe_return",
            wait=float(
                self.cfg.get(
                    "zero_focus_probe_wait_sec",
                    0.25
                )
            )
        )
        time.sleep(
            float(
                self.cfg.get(
                    "zero_focus_probe_settle_sec",
                    0.12
                )
            )
        )

        print(
            "[ZERO FOCUS TUNE] "
            f"after={after_error:.1f}px"
        )

        if after_error > before_error + float(
            self.cfg.get(
                "zero_focus_probe_margin_px",
                3.0
            )
        ):
            self.cfg["zero_focus_simple_x_from_image_y"] = round(
                -x_gain,
                6
            )
            self.cfg["zero_focus_simple_y_from_image_x"] = round(
                -y_gain,
                6
            )
            save_config(self.cfg)
            print(
                "[ZERO FOCUS TUNE] "
                "방향이 반대라서 simple gain 부호를 뒤집어 저장했습니다."
            )

        return True


    # 현재 화면에서 컨테이너 중심을 읽습니다.
    def read_zero_focus_center(self):

        frame = self.read_frame()

        if frame is None:
            return None

        detection, _ = self.detect_rectangle_object(frame)

        if not isinstance(detection, dict):
            return None

        center = detection.get("center", None)

        if (
            not isinstance(center, (list, tuple))
            or len(center) < 2
        ):
            return None

        return (
            float(center[0]),
            float(center[1])
        )


    # X/Y 시험 이동으로 실제 화면-로봇 방향 행렬을 측정해 한 번 중심으로 이동합니다.
    def zero_focus_center_once_live(self, target_pixel):

        start_center = self.read_zero_focus_center()

        if start_center is None:
            print("[ZERO FOCUS LIVE] 컨테이너 인식값 없음")
            return False

        target_u = float(target_pixel[0])
        target_v = float(target_pixel[1])
        error = np.array(
            [
                start_center[0] - target_u,
                start_center[1] - target_v
            ],
            dtype=float
        )
        error_norm = float(np.linalg.norm(error))

        tolerance = float(
            self.cfg.get(
                "zero_focus_accept_px",
                self.cfg.get(
                    "center_tolerance_px",
                    18.0
                )
            )
        )

        print(
            "[ZERO FOCUS LIVE] "
            f"err=({error[0]:+.1f},{error[1]:+.1f}) "
            f"norm={error_norm:.1f}px"
        )

        if error_norm <= tolerance:
            print("[ZERO FOCUS LIVE] 정렬 완료")
            return True

        coords = self.get_coords()

        if (
            not isinstance(coords, list)
            or len(coords) < 6
        ):
            print("[ZERO FOCUS LIVE] 현재 좌표 읽기 실패")
            return False

        start_coords = [
            float(value)
            for value in coords[:6]
        ]
        probe_mm = abs(
            float(
                self.cfg.get(
                    "zero_focus_live_probe_mm",
                    8.0
                )
            )
        )
        wait_sec = float(
            self.cfg.get(
                "zero_focus_probe_wait_sec",
                0.65
            )
        )
        settle_sec = float(
            self.cfg.get(
                "zero_focus_probe_settle_sec",
                0.35
            )
        )

        jacobian = None
        cached_jacobian = self.cfg.get(
            "zero_focus_cached_jacobian",
            None
        )
        cached_time = float(
            self.cfg.get(
                "zero_focus_cached_jacobian_time",
                0.0
            )
        )
        cache_sec = float(
            self.cfg.get(
                "zero_focus_jacobian_cache_sec",
                120.0
            )
        )

        if (
            isinstance(cached_jacobian, list)
            and time.time() - cached_time <= cache_sec
        ):
            try:
                candidate = np.array(
                    cached_jacobian,
                    dtype=float
                )
                if candidate.shape == (2, 2):
                    jacobian = candidate
                    print("[ZERO FOCUS LIVE] cached direction matrix 사용")
            except Exception:
                jacobian = None

        if jacobian is None:
            columns = []

            for axis_index, axis_name in ((0, "x"), (1, "y")):
                plus_coords = start_coords[:6]
                plus_coords[axis_index] += probe_mm

                moved = self.send_coords(
                    plus_coords,
                    f"zero_focus_live_probe_{axis_name}_plus",
                    wait=wait_sec
                )

                if not moved:
                    print(
                        "[ZERO FOCUS LIVE] "
                        f"{axis_name} 시험 이동 실패"
                    )
                    return False

                time.sleep(settle_sec)

                plus_center = self.read_zero_focus_center()

                self.send_coords(
                    start_coords,
                    f"zero_focus_live_probe_{axis_name}_plus_return",
                    wait=wait_sec
                )
                time.sleep(settle_sec)

                minus_coords = start_coords[:6]
                minus_coords[axis_index] -= probe_mm

                moved = self.send_coords(
                    minus_coords,
                    f"zero_focus_live_probe_{axis_name}_minus",
                    wait=wait_sec
                )

                if not moved:
                    print(
                        "[ZERO FOCUS LIVE] "
                        f"{axis_name} -시험 이동 실패"
                    )
                    return False

                time.sleep(settle_sec)

                minus_center = self.read_zero_focus_center()

                self.send_coords(
                    start_coords,
                    f"zero_focus_live_probe_{axis_name}_minus_return",
                    wait=wait_sec
                )
                time.sleep(settle_sec)

                if plus_center is None or minus_center is None:
                    print(
                        "[ZERO FOCUS LIVE] "
                        f"{axis_name} 시험 후 컨테이너 인식 실패"
                    )
                    return False

                columns.append(
                    [
                        (plus_center[0] - minus_center[0]) / (2.0 * probe_mm),
                        (plus_center[1] - minus_center[1]) / (2.0 * probe_mm)
                    ]
                )

            jacobian = np.array(
                [
                    [columns[0][0], columns[1][0]],
                    [columns[0][1], columns[1][1]]
                ],
                dtype=float
            )

        determinant = float(np.linalg.det(jacobian))

        if abs(determinant) < float(
            self.cfg.get(
                "zero_focus_live_min_det",
                0.01
            )
        ):
            print(
                "[ZERO FOCUS LIVE] 방향 행렬 불안정:",
                jacobian.tolist()
            )
            return False

        self.cfg["zero_focus_cached_jacobian"] = jacobian.tolist()
        self.cfg["zero_focus_cached_jacobian_time"] = round(time.time(), 3)

        gain = float(
            self.cfg.get(
                "zero_focus_live_gain",
                0.8
            )
        )

        try:
            robot_delta = -np.linalg.inv(jacobian) @ error * gain
        except np.linalg.LinAlgError:
            print("[ZERO FOCUS LIVE] 방향 행렬 역변환 실패")
            return False

        max_step = abs(
            float(
                self.cfg.get(
                    "zero_focus_live_max_step_mm",
                    18.0
                )
            )
        )
        delta_x = max(
            -max_step,
            min(max_step, float(robot_delta[0]))
        )
        delta_y = max(
            -max_step,
            min(max_step, float(robot_delta[1]))
        )

        target_coords = start_coords[:6]
        target_coords[0] += delta_x
        target_coords[1] += delta_y

        print(
            "[ZERO FOCUS LIVE] "
            f"J={jacobian.tolist()} "
            f"move=({delta_x:+.2f},{delta_y:+.2f})"
        )

        moved = self.send_coords(
            target_coords,
            "zero_focus_live_center",
            wait=float(
                self.cfg.get(
                    "zero_focus_live_move_wait_sec",
                    0.45
                )
            )
        )

        time.sleep(
            float(
                self.cfg.get(
                    "zero_focus_live_settle_sec",
                    0.15
                )
            )
        )

        if not moved:
            return False

        final_center = self.read_zero_focus_center()

        if final_center is None:
            return False

        final_error = np.array(
            [
                final_center[0] - float(target_pixel[0]),
                final_center[1] - float(target_pixel[1])
            ],
            dtype=float
        )
        final_error_norm = float(np.linalg.norm(final_error))

        print(
            "[ZERO FOCUS LIVE] "
            f"after_err=({final_error[0]:+.1f},{final_error[1]:+.1f}) "
            f"norm={final_error_norm:.1f}px"
        )

        return final_error_norm <= tolerance


    # 로봇팔 카메라 시점을 컨테이너 중앙으로 이동한 뒤 그 위치를 영점으로 저장합니다.
    def zero_focus_to_container(self):

        print(
            "[ZERO FOCUS] "
            "컨테이너 중심이 카메라 시점 중앙에 오도록 로봇팔을 이동합니다."
        )

        max_iter = int(
            self.cfg.get(
                "zero_focus_center_max_iter",
                self.cfg.get(
                    "auto_center_max_iter",
                    8
                )
            )
        )

        frame = self.read_frame()

        if frame is None:
            print("[ZERO FOCUS ERROR] 카메라 영상 없음")
            return False

        frame_height, frame_width = frame.shape[:2]
        image_center_target = (
            frame_width / 2.0,
            frame_height / 2.0
        )

        centered = False

        if bool(
            self.cfg.get(
                "zero_focus_use_live_jacobian",
                True
            )
        ):
            for _ in range(max_iter):
                centered = self.zero_focus_center_once_live(
                    image_center_target
                )

                if centered:
                    break

                time.sleep(0.12)

        if not centered:
            if bool(
                self.cfg.get(
                    "zero_focus_allow_fallback_centering",
                    False
                )
            ):
                self.auto_tune_zero_focus_direction(
                    image_center_target
                )

                centered = self.auto_center(
                    max_iter=max_iter,
                    direction_sign=float(
                        self.cfg.get(
                            "zero_focus_center_direction_sign",
                            1.0
                        )
                    ),
                    target_pixel=image_center_target,
                    use_simple=bool(
                        self.cfg.get(
                            "zero_focus_use_simple_centering",
                            True
                        )
                    )
                )

        if not centered:
            print(
                "[ZERO FOCUS ERROR] "
                "중앙 정렬 실패. 영점 저장을 중단합니다."
            )
            return False

        self.arm.reset_tcp_cache()

        verified_center = self.read_zero_focus_center()

        if verified_center is None:
            print(
                "[ZERO FOCUS ERROR] "
                "정렬 후 컨테이너 인식 실패. 영점 저장을 중단합니다."
            )
            return False

        verify_error = (
            (
                verified_center[0]
                - image_center_target[0]
            ) ** 2
            + (
                verified_center[1]
                - image_center_target[1]
            ) ** 2
        ) ** 0.5

        max_save_error_px = float(
            self.cfg.get(
                "zero_focus_save_max_error_px",
                self.cfg.get(
                    "zero_focus_accept_px",
                    35.0
                )
            )
        )

        if verify_error > max_save_error_px:
            print(
                "[ZERO FOCUS ERROR] "
                f"정렬 오차가 큽니다: {verify_error:.1f}px "
                f"> {max_save_error_px:.1f}px. 저장을 중단합니다."
            )
            return False

        saved = self.save_target_pixel(
            use_image_center=True
        )

        if not saved:
            print("[ZERO FOCUS ERROR] 영점 저장 실패")
            return False

        coords = self.get_coords()

        self.cfg["last_zero_focus_calibration"]["tcp_coords"] = (
            None
            if not isinstance(coords, list)
            else [
                round(float(value), 3)
                for value in coords[:6]
            ]
        )
        self.cfg["last_zero_focus_calibration"]["centered"] = bool(centered)
        save_config(self.cfg)

        frame = self.read_frame()
        coords = self.get_coords()

        if (
            frame is not None
            and isinstance(coords, list)
            and len(coords) >= 6
        ):
            detection, _ = self.detect_rectangle_object(frame)

            if isinstance(detection, dict):
                pick_plan = self.calculate_direct_xyz_pick_target(
                    [
                        float(value)
                        for value in coords[:6]
                    ],
                    detection,
                    frame.shape,
                    apply_manual_calibration=False
                )

                if isinstance(pick_plan, dict):
                    self.cfg["last_zero_focus_pick_plan"] = {
                        "center_u_px": round(
                            float(pick_plan["center_u"]),
                            3
                        ),
                        "center_v_px": round(
                            float(pick_plan["center_v"]),
                            3
                        ),
                        "depth_mm": round(
                            float(pick_plan["depth_mm"]),
                            3
                        ),
                        "current_coords": [
                            round(float(value), 3)
                            for value in coords[:6]
                        ],
                        "base_target_coords": [
                            round(float(value), 3)
                            for value in pick_plan["base_target_coords"]
                        ],
                        "target_coords": [
                            round(float(value), 3)
                            for value in pick_plan["target_coords"]
                        ],
                        "visual_move_x_mm": round(
                            float(pick_plan["visual_move_x_mm"]),
                            3
                        ),
                        "visual_move_y_mm": round(
                            float(pick_plan["visual_move_y_mm"]),
                            3
                        ),
                        "visual_move_z_mm": round(
                            float(pick_plan["visual_move_z_mm"]),
                            3
                        ),
                        "timestamp": round(time.time(), 3)
                    }
                    save_config(self.cfg)
                    print(
                        "[ZERO FOCUS PLAN] "
                        f"target="
                        f"{self.cfg['last_zero_focus_pick_plan']['target_coords'][:3]} "
                        f"depth={self.cfg['last_zero_focus_pick_plan']['depth_mm']:.1f}mm"
                    )

        print(
            "[ZERO FOCUS] "
            "카메라 시점 중앙 정렬과 영점 저장 완료"
        )
        return True

    # 현재 TCP 좌표를 물체 집기 기준 위치로 기억하는 함수입니다.
    # 지정한 모터 하나만 현재 각도에서 미세하게 움직입니다.
    # 관절 키 명령을 영상 루프를 멈추지 않고 큐에 추가합니다.
    def enqueue_joint_jog(self, joint_index, direction, label):

        try:

            # 관절 번호와 방향을 큐에 넣습니다.
            self.joint_jog_queue.put_nowait(
                (
                    int(joint_index),
                    float(direction),
                    str(label)
                )
            )

            # 키가 정상적으로 인식됐음을 출력합니다.
            print(
                "[JOINT KEY] "
                f"{label} -> motor {int(joint_index) + 1}"
            )

            return True

        except queue.Full:

            # 명령이 너무 많이 쌓이면 안전을 위해 추가 명령을 버립니다.
            print(
                "[JOINT QUEUE FULL] 천천히 다시 누르세요:",
                label
            )

            return False


    # 관절 명령을 카메라 영상과 별도 스레드에서 순서대로 실행합니다.
    def joint_jog_worker_loop(self):

        while True:

            # 다음 관절 명령을 기다립니다.
            joint_index, direction, label = (
                self.joint_jog_queue.get()
            )

            try:

                # TCP 통신과 실제 모터 이동은 이 스레드에서 실행합니다.
                self.jog_single_joint(
                    joint_index,
                    direction,
                    label
                )

            except Exception as error:

                # 관절 오류가 나도 카메라 프로그램은 계속 실행합니다.
                print(
                    "[JOINT WORKER ERROR]",
                    type(error).__name__,
                    ":",
                    error
                )

            finally:

                # 현재 큐 명령 처리가 끝났음을 표시합니다.
                self.joint_jog_queue.task_done()


    # 로봇팔 서버에 한 줄 JSON TCP 명령을 보냅니다.
    def arm_tcp_request(self, request_data, timeout_sec=3.0):

        # 로봇팔 서버 주소입니다.
        server_host = str(
            self.cfg.get(
                "arm_server_host",
                "127.0.0.1"
            )
        )

        # 로봇팔 서버 포트입니다.
        server_port = int(
            self.cfg.get(
                "arm_server_port",
                15000
            )
        )

        try:

            # JSON 뒤에 줄바꿈을 붙여 서버 형식에 맞춥니다.
            request_bytes = (
                json.dumps(request_data)
                + "\n"
            ).encode("utf-8")

            # 로봇팔 서버에 연결합니다.
            with socket.create_connection(
                (server_host, server_port),
                timeout=float(timeout_sec)
            ) as sock:

                # 응답 대기 제한 시간을 설정합니다.
                sock.settimeout(float(timeout_sec))

                # 요청을 전송합니다.
                sock.sendall(request_bytes)

                # 응답 버퍼입니다.
                response_bytes = b""

                # 한 줄 응답을 모두 받을 때까지 반복합니다.
                while not response_bytes.endswith(b"\n"):

                    # 서버 응답을 읽습니다.
                    chunk = sock.recv(4096)

                    # 연결이 종료되면 반복을 끝냅니다.
                    if not chunk:
                        break

                    # 읽은 데이터를 버퍼에 추가합니다.
                    response_bytes += chunk

            # 응답이 비어 있으면 실패입니다.
            if not response_bytes:
                return {
                    "ok": False,
                    "error": "empty server response"
                }

            # 서버 응답을 JSON으로 변환합니다.
            return json.loads(
                response_bytes.decode("utf-8").strip()
            )

        # 연결 오류가 발생해도 프로그램을 종료하지 않습니다.
        except Exception as error:
            return {
                "ok": False,
                "error": (
                    f"{type(error).__name__}: "
                    f"{error}"
                )
            }


    # 선택한 모터 하나만 현재 각도를 기준으로 미세조정합니다.
    # 선택한 모터 하나만 지정된 방향으로 미세조정합니다.
    def jog_single_joint(self, joint_index, direction, label):

        try:

            # 내부 관절 번호가 0~5인지 검사합니다.
            if not 0 <= int(joint_index) < 6:
                print(
                    "[JOINT JOG ERROR] 잘못된 모터 번호:",
                    joint_index
                )
                return False

            # 키 한 번당 이동할 각도입니다.
            step_deg = abs(
                float(
                    self.cfg.get(
                        "joint_jog_deg",
                        1.0
                    )
                )
            )

            # 왼쪽 또는 오른쪽 방향을 적용한 실제 증감값입니다.
            delta_deg = (
                step_deg
                * float(direction)
            )

            # 현재 모터 번호입니다.
            joint_id = int(joint_index) + 1

            # 관절 이동 속도입니다.
            speed = int(
                self.cfg.get(
                    "joint_jog_speed",
                    35
                )
            )

            # 모터별 최소 안전 각도입니다.
            minimum_angle = float(
                self.cfg.get(
                    f"joint{joint_id}_min_deg",
                    self.cfg.get(
                        "joint_jog_min_deg",
                        -165.0
                    )
                )
            )

            # 모터별 최대 안전 각도입니다.
            maximum_angle = float(
                self.cfg.get(
                    f"joint{joint_id}_max_deg",
                    self.cfg.get(
                        "joint_jog_max_deg",
                        165.0
                    )
                )
            )

            # 로봇팔 서버에 선택한 모터 하나의 이동 명령을 보냅니다.
            response = self.arm_tcp_request(
                {
                    "cmd": "joint_step",
                    "joint": joint_id,
                    "delta": delta_deg,
                    "speed": speed,
                    "wait": 0.0,
                    "min_angle": minimum_angle,
                    "max_angle": maximum_angle
                },
                timeout_sec=5.0
            )

            # 서버가 이동을 거부한 경우입니다.
            if not response.get("ok", False):
                print(
                    "[JOINT JOG ERROR]",
                    label,
                    ":",
                    response.get(
                        "error",
                        response
                    )
                )
                return False

            # 실제 이동 결과를 출력합니다.
            print(
                "[JOINT JOG OK] "
                f"키={label} | "
                f"모터={response.get('joint')} | "
                f"이전={float(response.get('before', 0.0)):.2f}도 | "
                f"목표={float(response.get('target', 0.0)):.2f}도 | "
                f"변화={float(response.get('delta', 0.0)):+.2f}도 | "
                f"방법={response.get('method')}"
            )

            # 성공입니다.
            return True

        # 오류가 발생해도 프로그램을 종료하지 않습니다.
        except Exception as error:

            print(
                "[JOINT JOG EXCEPTION]",
                type(error).__name__,
                ":",
                error
            )

            return False






    def remember_current_object_pose(self, reason="manual"):

        # 현재 로봇팔 TCP 좌표를 읽습니다.
        coords = self.get_coords()

        # 좌표가 비정상이면 저장하지 않습니다.
        if coords is None:
            print("[MEMORY] 현재 좌표 읽기 실패")
            return False

        # 현재 좌표를 마지막 물체 위치로 저장합니다.
        self.cfg["last_object_coords"] = coords[:6]

        # 저장 시간을 기록합니다.
        self.cfg["last_object_time"] = time.time()

        # 설정 파일에 저장합니다.
        save_config(self.cfg)

        # 저장 로그를 출력합니다.
        print(f"[MEMORY] object pose saved by {reason}: {[round(v, 2) for v in coords[:6]]}")

        # 성공입니다.
        return True

    # 저장된 물체 위치를 가져오는 함수입니다.
    def get_memory_object_pose(self):

        # 저장된 좌표를 가져옵니다.
        coords = self.cfg.get("last_object_coords", None)

        # 저장된 좌표가 없으면 실패입니다.
        if not isinstance(coords, list) or len(coords) < 6:
            print("[MEMORY] 저장된 물체 좌표 없음")
            return None

        # 저장 시간을 가져옵니다.
        saved_time = float(self.cfg.get("last_object_time", 0.0))

        # 시간 제한을 가져옵니다.
        timeout = float(self.cfg.get("memory_pick_timeout_sec", 9999.0))

        # 너무 오래된 좌표면 사용하지 않습니다.
        if time.time() - saved_time > timeout:
            print("[MEMORY] 저장된 물체 좌표가 너무 오래됨")
            return None

        # 저장된 좌표를 반환합니다.
        print("[MEMORY] saved object pose 사용:", [round(v, 2) for v in coords[:6]])
        return coords[:6]


    # 직사각형 인식 상태를 확인하고 자동 집기를 시작하는 함수입니다.
    def update_auto_pick(self, detection):

        # 자동 집기가 꺼져 있으면 아무 작업도 하지 않습니다.
        if not bool(self.cfg.get("auto_pick_enabled", True)):
            return

        # 이미 로봇팔 작업이 실행 중이면 새 작업을 시작하지 않습니다.
        if self.task_busy:
            return

        # 물체를 인식하지 못한 경우입니다.
        if detection is None:

            # 안정 인식 카운터를 초기화합니다.
            self.auto_stable_count = 0

            # 물체가 사라진 프레임 수를 증가시킵니다.
            self.auto_missing_count += 1

            # 마지막 중심을 초기화합니다.
            self.last_detect_center = None

            # 일정 프레임 동안 물체가 사라지면 다음 물체를 집을 수 있게 재무장합니다.
            if self.auto_missing_count >= int(
                self.cfg.get("auto_rearm_missing_frames", 20)
            ):
                self.auto_pick_latched = False

            return

        # 물체가 검출됐으므로 missing 카운터를 초기화합니다.
        self.auto_missing_count = 0

        # 현재 검출 중심입니다.
        cx, cy = detection["center"]

        # 이전 중심이 없으면 첫 검출로 처리합니다.
        if self.last_detect_center is None:

            # 첫 안정 프레임입니다.
            self.auto_stable_count = 1

        else:

            # 이전 중심 좌표입니다.
            last_x, last_y = self.last_detect_center

            # 이전 프레임과 현재 프레임 사이의 중심 이동량입니다.
            movement_px = (
                (cx - last_x) ** 2
                + (cy - last_y) ** 2
            ) ** 0.5

            # 물체가 거의 움직이지 않았다면 안정 프레임을 증가시킵니다.
            if movement_px <= float(
                self.cfg.get("auto_stable_center_px", 12.0)
            ):
                self.auto_stable_count += 1

            # 갑자기 다른 위치로 검출되면 카운터를 다시 시작합니다.
            else:
                self.auto_stable_count = 1

        # 현재 중심을 다음 프레임 비교용으로 저장합니다.
        self.last_detect_center = (float(cx), float(cy))

        # 이미 이 물체에 대해 자동 집기를 실행했다면 중복 실행하지 않습니다.
        if self.auto_pick_latched:
            return

        # 자동 집기에 필요한 연속 안정 프레임 수입니다.
        required_frames = int(
            self.cfg.get("auto_stable_frames_required", 10)
        )

        # 아직 필요한 프레임 수에 도달하지 못했으면 대기합니다.
        if self.auto_stable_count < required_frames:
            return

        # 마지막 집기 이후 최소 대기 시간입니다.
        cooldown = float(
            self.cfg.get("auto_pick_cooldown_sec", 8.0)
        )

        # 최소 대기 시간이 지나지 않았으면 실행하지 않습니다.
        if time.time() - self.last_auto_pick_time < cooldown:
            return

        # 같은 물체를 다시 집지 않게 latch를 켭니다.
        self.auto_pick_latched = True

        # 마지막 자동 집기 시간을 기록합니다.
        self.last_auto_pick_time = time.time()

        # 자동 인식 카운터를 초기화합니다.
        self.auto_stable_count = 0

        # 자동 집기 시작 로그입니다.
        print(
            "[AUTO PICK] 컨테이너 우선 인식 완료 "
            f"confidence={float(detection.get('confidence', 0.0)) * 100.0:.1f}%"
        )

        # 카메라 화면은 계속 갱신하면서 자동 집기를 백그라운드에서 실행합니다.
        self.run_task("full_auto_pick", self.pick)


    # 키 입력 처리입니다.
    def handle_key(self, key):

        # g는 현재 TCP 회전값을 카메라 바닥 보기 자세로 저장합니다.
        if key == ord("g"):

            self.save_current_floor_camera_rpy()

            return

        # 한 번 이동 거리입니다.
        step = float(self.cfg.get("tcp_step_mm", self.cfg["jog_mm"]))

        # i는 위로 이동입니다.
        if key == ord("i"):
            self.set_manual_cmd("z", step)

        # k는 아래로 이동입니다.
        elif key == ord("k"):
            self.set_manual_cmd("z", -step)

        # j는 y+ 이동입니다.
        elif key == ord("j"):
            self.set_manual_cmd("y", step)

        # l은 y- 이동입니다.
        elif key == ord("l"):
            self.set_manual_cmd("y", -step)

        # u는 x+ 이동입니다.
        elif key == ord("u"):
            self.set_manual_cmd("x", step)

        # m은 x- 이동입니다.
        elif key == ord("m"):
            self.set_manual_cmd("x", -step)

        # a는 시작 카메라 자세로 이동입니다.
        elif key == ord("a"):
            self.run_task("camera_ready_pose", self.go_camera_ready_pose)

        # t는 현재 관절 자세를 시작 카메라 자세로 저장합니다.
        elif key == ord("t"):
            self.run_task("save_camera_ready", self.save_current_as_camera_ready)

        # q는 중심 맞추기입니다.
        elif key == ord("q"):
            self.run_task("auto_center", self.auto_center)

        # z는 로봇팔 카메라 시점을 컨테이너 중앙에 맞추고 영점으로 저장합니다.
        elif key == ord("z"):
            self.run_task("zero_focus_calibration", self.zero_focus_to_container)

        # w 또는 b는 기억된 물체를 집고 초기 자세로 복귀합니다.
        elif key == ord("w") or key == ord("b"):
            if isinstance(
                self.cfg.get(
                    "pending_direct_pick_manual_calibration",
                    None
                ),
                dict
            ):
                print(
                    "[PICK CALIB] "
                    "수동 보정 중입니다. 먼저 y로 현재 집기 좌표를 저장하세요."
                )
                return

            self.run_task(
                "pick_and_return_home",
                self.pick_and_return_home
            )

        # e는 놓기입니다.
        elif key == ord("e"):
            self.run_task("place", self.place)

        # s는 scan 좌표 저장입니다.
        elif key == ord("s"):
            self.run_task("save_scan_pose", self.save_scan_pose)

        # h는 scan 좌표 이동입니다.
        elif key == ord("h"):
            self.run_task("go_scan_pose", self.go_scan_pose)

        # p는 place 좌표 저장입니다.
        elif key == ord("p"):
            self.run_task("save_place_pose", self.save_place_pose)

        # v는 로봇팔을 움직이지 않고 현재 물체 중심만 목표 픽셀로 저장합니다.
        elif key == ord("v"):
            self.run_task("zero_focus_calibration", self.save_target_pixel)

        # f는 현재 그리퍼 TCP 좌표를 물체 위치로 강제 저장합니다.
        elif key == ord("f"):
            self.remember_current_object_pose("manual_f_key")

        # n은 자동 계산 목표를 저장하고 키보드 수동 보정을 시작합니다.
        elif key == ord("n"):
            self.run_task(
                "start_pick_manual_calibration",
                self.start_direct_pick_manual_calibration
            )

        # y는 손으로 맞춘 현재 TCP 좌표를 성공 집기 좌표로 저장합니다.
        elif key == ord("y"):
            self.run_task(
                "save_pick_manual_calibration",
                self.save_direct_pick_manual_calibration
            )

        # [ 는 덜 내려가게 합니다.
        elif key == ord("["):
            self.cfg["pick_down_mm"] = max(5.0, float(self.cfg["pick_down_mm"]) - 2.0)
            save_config(self.cfg)
            print("[DEPTH] pick_down_mm:", self.cfg["pick_down_mm"])

        # ] 는 더 내려가게 합니다.
        elif key == ord("]"):
            self.cfg["pick_down_mm"] = float(self.cfg["pick_down_mm"]) + 2.0
            save_config(self.cfg)
            print("[DEPTH] pick_down_mm:", self.cfg["pick_down_mm"])

        # ; 는 깊이 보정값을 올립니다. 즉 덜 내려갑니다.
        elif key == ord(";"):
            self.cfg["depth_bias_mm"] = float(self.cfg.get("depth_bias_mm", 0.0)) + 1.0
            save_config(self.cfg)
            print("[DEPTH] depth_bias_mm:", self.cfg["depth_bias_mm"])

        # ' 는 깊이 보정값을 내립니다. 즉 더 내려갑니다.
        elif key == ord("'"):
            self.cfg["depth_bias_mm"] = float(self.cfg.get("depth_bias_mm", 0.0)) - 1.0
            save_config(self.cfg)
            print("[DEPTH] depth_bias_mm:", self.cfg["depth_bias_mm"])


        # g는 현재 Z 좌표를 실제 컨테이너 집기 높이로 저장합니다.
        elif key == ord("g"):
            self.save_pick_target_z()

        # d는 카메라 방향과 로봇 X/Y 방향을 자동으로 측정합니다.
        elif key == ord("d"):
            self.run_task(
                "xy_direction_calibration",
                self.calibrate_xy_mapping
            )

        # x는 자동 집기 기능을 켜거나 끕니다.
        elif key == ord("x"):

            # 현재 자동 집기 상태를 반전합니다.
            self.cfg["auto_pick_enabled"] = not bool(
                self.cfg.get("auto_pick_enabled", True)
            )

            # 변경된 설정을 저장합니다.
            save_config(self.cfg)

            # 현재 상태를 출력합니다.
            print(
                "[AUTO PICK]",
                "ON" if self.cfg["auto_pick_enabled"] else "OFF"
            )

        # r은 현재 자동 집기 latch를 수동으로 초기화합니다.
        elif key == ord("r"):

            # 중복 집기 방지 상태를 해제합니다.
            self.auto_pick_latched = False

            # 안정 인식 카운터를 초기화합니다.
            self.auto_stable_count = 0

            # 물체 미검출 카운터를 초기화합니다.
            self.auto_missing_count = 0

            # 마지막 검출 중심을 초기화합니다.
            self.last_detect_center = None

            # 재무장 상태를 출력합니다.
            print("[AUTO PICK] rearmed")

        # o는 그리퍼 열기입니다.
        elif key == ord("o"):
            self.run_task("gripper_open", self.open_gripper)

        # c는 그리퍼 닫기입니다.
        elif key == ord("c"):
            self.run_task("gripper_close", self.close_gripper)

        # 1은 x 방향 반전입니다.
        elif key == ord("1"):
            self.cfg["x_from_image_y"] *= -1.0
            save_config(self.cfg)
            print("[TUNE] x_from_image_y:", self.cfg["x_from_image_y"])

        # 2는 y 방향 반전입니다.
        elif key == ord("2"):
            self.cfg["y_from_image_x"] *= -1.0
            save_config(self.cfg)
            print("[TUNE] y_from_image_x:", self.cfg["y_from_image_x"])

        # -는 하강 깊이 감소입니다.
        elif key == ord("-"):
            self.cfg["pick_down_mm"] = max(5.0, float(self.cfg["pick_down_mm"]) - 5.0)
            save_config(self.cfg)
            print("[TUNE] pick_down_mm:", self.cfg["pick_down_mm"])

        # =는 하강 깊이 증가입니다.
        elif key == ord("=") or key == ord("+"):
            self.cfg["pick_down_mm"] = float(self.cfg["pick_down_mm"]) + 5.0
            save_config(self.cfg)
            print("[TUNE] pick_down_mm:", self.cfg["pick_down_mm"])

    # 메인 루프입니다.
    def run(self):

        # 계속 반복합니다.
        while True:

            # 프레임을 읽습니다.
            frame = self.read_frame()

            # 프레임 없으면 대기합니다.
            if frame is None:
                print("[WARN] frame 없음")
                time.sleep(0.1)
                continue

            # 직사각형 검출입니다.
            # 화면 프레임 번호를 증가시킵니다.
            self.vision_frame_count += 1

            # 무거운 영상 검출은 설정된 프레임 간격마다 실행합니다.
            if (
                self.vision_frame_count
                % self.vision_process_every_n_frames
                == 0
                or self.cached_edges is None
            ):

                detection, edges = (
                    self.detect_rectangle_object(
                        frame
                    )
                )

                # 다음 프레임에서 재사용할 검출 결과를 저장합니다.
                self.cached_detection = detection
                self.cached_edges = edges

            else:

                # 검출을 생략한 프레임에서는 직전 결과를 사용합니다.
                detection = self.cached_detection
                edges = self.cached_edges

            # 카메라에 물체가 보이면 집기 준비 상태로 기억합니다.
            self.remember_visible_object(detection)

            # 기존 자동 집기 상태 계산을 유지합니다.
            # 설정에서 auto_pick_enabled=False이므로 버튼을 누르기 전에는 집지 않습니다.
            self.update_auto_pick(detection)

            # 화면 표시입니다.
            frame = self.draw_overlay(frame, detection)

            # 원본 화면입니다.
            cv2.imshow(self.window_name, frame)

            # 에지 화면입니다.
            # 디버그가 필요할 때만 에지 영상을 표시합니다.
            if self.show_edges:
                cv2.imshow(
                    "rect_edges",
                    edges
                )

            # 키 입력입니다.
            # waitKeyEx는 숫자 키패드와 기호키의 원시 코드도 읽을 수 있습니다.
            key_raw = cv2.waitKeyEx(1)

            # 일반 문자 비교용 하위 8비트 값입니다.
            key = key_raw & 0xFF

            # 각 관절의 왼쪽 방향 부호를 읽습니다.
            joint1_left = float(self.cfg.get("joint1_left_sign", -1.0))
            joint2_left = float(self.cfg.get("joint2_left_sign", -1.0))
            joint3_left = float(self.cfg.get("joint3_left_sign", -1.0))
            joint4_left = float(self.cfg.get("joint4_left_sign", -1.0))
            joint5_left = float(self.cfg.get("joint5_left_sign", -1.0))
            joint6_left = float(self.cfg.get("joint6_left_sign", -1.0))

            # 키보드 위쪽 숫자열과 기호키입니다.
            joint_key_map = {
                ord("1"): (0, joint1_left, "motor1_left"),
                ord("2"): (0, -joint1_left, "motor1_right"),
                ord("3"): (1, joint2_left, "motor2_left"),
                ord("4"): (1, -joint2_left, "motor2_right"),
                ord("5"): (2, joint3_left, "motor3_left"),
                ord("6"): (2, -joint3_left, "motor3_right"),
                ord("7"): (3, joint4_left, "motor4_left"),
                ord("8"): (3, -joint4_left, "motor4_right"),
                ord("9"): (4, joint5_left, "motor5_left"),
                ord("0"): (4, -joint5_left, "motor5_right"),
                ord("-"): (5, joint6_left, "motor6_left"),
                ord("_"): (5, joint6_left, "motor6_left"),
                ord("="): (5, -joint6_left, "motor6_right"),
                ord("+"): (5, -joint6_left, "motor6_right"),
            }

            # Ubuntu 숫자 키패드의 원시 키 코드입니다.
            keypad_joint_key_map = {
                65457: (0, joint1_left, "motor1_left"),
                65458: (0, -joint1_left, "motor1_right"),
                65459: (1, joint2_left, "motor2_left"),
                65460: (1, -joint2_left, "motor2_right"),
                65461: (2, joint3_left, "motor3_left"),
                65462: (2, -joint3_left, "motor3_right"),
                65463: (3, joint4_left, "motor4_left"),
                65464: (3, -joint4_left, "motor4_right"),
                65465: (4, joint5_left, "motor5_left"),
                65456: (4, -joint5_left, "motor5_right"),
                65453: (5, joint6_left, "motor6_left"),
                65451: (5, -joint6_left, "motor6_right"),
            }

            # 일반 키보드 입력을 확인합니다.
            joint_command = joint_key_map.get(
                key,
                None
            )

            # 일반 키가 아니면 숫자 키패드 입력을 확인합니다.
            if joint_command is None:
                joint_command = keypad_joint_key_map.get(
                    key_raw,
                    None
                )

            # 관절 제어 키가 눌린 경우입니다.
            if joint_command is not None:

                # 키에 연결된 관절 번호와 방향을 가져옵니다.
                joint_index, direction, label = joint_command

                # 관절 명령은 큐에만 넣으므로 카메라 화면이 멈추지 않습니다.
                self.enqueue_joint_jog(
                    joint_index,
                    direction,
                    label
                )

                # 기존 숫자 및 기호 튜닝 기능과 충돌하지 않습니다.
                continue


            # ESC면 종료합니다.
            if key == 27:
                break

            # 키 입력이 없으면 넘깁니다.
            if key == 255:
                continue

            # 키 처리입니다.
            self.handle_key(key)

        # ROS 영상 수신 스레드를 먼저 중지합니다.
        self.ros_spin_running = False

        # 스레드가 반복문을 빠져나올 시간을 줍니다.
        time.sleep(0.03)

        # ROS 영상 수신 노드를 제거합니다.
        self.ros_node.destroy_node()

        # 현재 프로그램의 ROS 2 기능을 종료합니다.
        if rclpy.ok():
            rclpy.shutdown()

        # 창 닫기입니다.
        cv2.destroyAllWindows()


# 메인 함수입니다.
def main():

    # 앱 객체를 생성합니다.
    app = ArmCameraRectAutoPickClient()

    # 실행합니다.
    app.run()


# 직접 실행 시 main을 호출합니다.
if __name__ == "__main__":
    main()
