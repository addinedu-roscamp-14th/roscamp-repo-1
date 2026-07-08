#!/usr/bin/env python3

# ROS2 Python 라이브러리입니다.
import rclpy

# ROS2 노드 클래스를 사용합니다.
from rclpy.node import Node

# ROS2 액션 서버를 사용합니다.
from rclpy.action import ActionServer

# MoveIt이 보내는 FollowJointTrajectory 액션 타입입니다.
from control_msgs.action import FollowJointTrajectory

# 현재 관절 상태를 RViz/MoveIt에 보내기 위한 메시지입니다.
from sensor_msgs.msg import JointState

# 실제 MyCobot280 로봇팔 제어 라이브러리입니다.
from pymycobot.mycobot280 import MyCobot280

# radian과 degree 변환에 사용합니다.
import math

# 짧은 대기 시간에 사용합니다.
import time

# TCP 서버와 ROS 노드를 동시에 돌리기 위해 사용합니다.
import threading

# 카메라 클라이언트와 통신하기 위해 사용합니다.
import socket

# TCP 명령을 JSON으로 주고받기 위해 사용합니다.
import json


# 실제 로봇팔 포트입니다.
ROBOT_PORT = "/dev/ttyUSB0"

# MyCobot280 통신 속도입니다.
ROBOT_BAUD = 1000000

# 카메라 클라이언트 접속 주소입니다.
TCP_HOST = "127.0.0.1"

# 카메라 클라이언트 접속 포트입니다.
TCP_PORT = 15000


# radian을 degree로 변환하는 함수입니다.
def rad_to_deg(value):
    return float(value) * 180.0 / math.pi


# degree를 radian으로 변환하는 함수입니다.
def deg_to_rad(value):
    return float(value) * math.pi / 180.0


# RViz와 카메라 클라이언트가 함께 쓰는 통합 로봇팔 서버입니다.
class RobotArmServer(Node):

    # 초기화 함수입니다.
    def __init__(self):

        # ROS2 노드 이름을 설정합니다.
        super().__init__("robot_arm_server")

        # MoveIt/URDF 관절 이름입니다.
        self.joint_names = [
            "1_Joint",
            "2_Joint",
            "3_Joint",
            "4_Joint",
            "5_Joint",
            "6_Joint",
        ]

        # 로봇팔 시리얼 접근 충돌을 막는 lock입니다.
        self.lock = threading.Lock()

        # 실제 로봇팔 객체를 생성합니다.
        self.robot = MyCobot280(ROBOT_PORT, ROBOT_BAUD)

        # 연결 안정화 대기입니다.
        time.sleep(1.0)

        # 마지막으로 정상 읽은 관절 각도입니다.
        self.last_angles = None

        # 마지막으로 정상 읽은 TCP 좌표입니다.
        self.last_coords = None

        # 빠른 수동 조작용 목표 좌표 캐시입니다.
        self.command_coords = None

        # 시리얼 오류 출력 횟수를 줄이기 위한 카운터입니다.
        self.serial_error_count = 0

        # /joint_states 퍼블리셔를 생성합니다.
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # 0.1초마다 현재 관절 상태를 발행합니다.
        self.timer = self.create_timer(1.5, self.publish_joint_states)

        # MoveIt이 사용하는 FollowJointTrajectory 액션 서버입니다.
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_group_controller/follow_joint_trajectory",
            self.execute_trajectory_callback,
        )

        # TCP 서버를 별도 스레드에서 실행합니다.
        self.tcp_thread = threading.Thread(target=self.tcp_server_loop, daemon=True)

        # TCP 서버 스레드를 시작합니다.
        self.tcp_thread.start()

        # 시작 로그입니다.
        self.get_logger().info(f"robot_arm_server started: {ROBOT_PORT} / {ROBOT_BAUD}")
        self.get_logger().info(f"TCP command server: {TCP_HOST}:{TCP_PORT}")

    # 현재 로봇팔 관절 각도를 안전하게 읽습니다.
    def get_angles_safe(self):

        # 시리얼 접근을 lock으로 보호합니다.
        with self.lock:

            try:

                # 실제 로봇팔 관절 각도를 읽습니다.
                angles = self.robot.get_angles()

            except Exception as e:

                # 시리얼 오류가 나도 서버를 죽이지 않고 마지막 정상값을 사용합니다.
                self.serial_error_count += 1

                # 로그가 너무 많이 찍히지 않게 20번에 한 번만 출력합니다.
                if self.serial_error_count % 20 == 1:
                    self.get_logger().warn(f"get_angles failed, using last value: {e}")

                # 마지막 정상 관절값을 반환합니다.
                return self.last_angles

        # 값이 비정상이면 마지막 정상값을 반환합니다.
        if not isinstance(angles, list) or len(angles) < 6:
            return self.last_angles

        # 6개 관절값만 저장합니다.
        self.last_angles = angles[:6]

        # 정상 읽기 성공 시 오류 카운터를 초기화합니다.
        self.serial_error_count = 0

        # 6개 관절값을 반환합니다.
        return self.last_angles


    # 현재 로봇팔 TCP 좌표를 안전하게 읽습니다.
    def get_coords_safe(self):

        # 시리얼 접근을 lock으로 보호합니다.
        with self.lock:

            try:

                # 실제 로봇팔 TCP 좌표를 읽습니다.
                coords = self.robot.get_coords()

            except Exception as e:

                # 시리얼 오류가 나도 서버를 죽이지 않고 마지막 정상값을 사용합니다.
                self.serial_error_count += 1

                # 로그가 너무 많이 찍히지 않게 20번에 한 번만 출력합니다.
                if self.serial_error_count % 20 == 1:
                    self.get_logger().warn(f"get_coords failed, using last value: {e}")

                # 마지막 정상 좌표를 반환합니다.
                return self.last_coords

        # 값이 비정상이면 마지막 정상값을 반환합니다.
        if not isinstance(coords, list) or len(coords) < 6:
            return self.last_coords

        # 6개 좌표만 저장합니다.
        self.last_coords = coords[:6]

        # 정상 읽기 성공 시 오류 카운터를 초기화합니다.
        self.serial_error_count = 0

        # 6개 좌표를 반환합니다.
        return self.last_coords


    # 현재 관절 상태를 /joint_states로 발행합니다.
    def publish_joint_states(self):

        # 현재 관절 각도를 읽습니다.
        angles = self.get_angles_safe()

        # 읽기 실패 시 발행하지 않습니다.
        if angles is None:
            return

        # JointState 메시지를 생성합니다.
        msg = JointState()

        # 현재 시간을 넣습니다.
        msg.header.stamp = self.get_clock().now().to_msg()

        # 관절 이름을 넣습니다.
        msg.name = self.joint_names

        # degree를 radian으로 변환해서 넣습니다.
        msg.position = [deg_to_rad(a) for a in angles]

        # velocity는 비워둡니다.
        msg.velocity = []

        # effort도 비워둡니다.
        msg.effort = []

        # 메시지를 발행합니다.
        self.joint_pub.publish(msg)

    # RViz/MoveIt trajectory 실행 콜백입니다.
    def execute_trajectory_callback(self, goal_handle):

        # MoveIt이 보낸 trajectory를 가져옵니다.
        trajectory = goal_handle.request.trajectory

        # point가 없으면 실패 처리합니다.
        if not trajectory.points:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = -1
            result.error_string = "empty trajectory"
            return result

        # trajectory 개수를 로그로 출력합니다.
        self.get_logger().info(f"MoveIt trajectory received: {len(trajectory.points)} points")

        # trajectory point를 순서대로 실행합니다.
        for point in trajectory.points:

            # 관절 position이 부족하면 건너뜁니다.
            if len(point.positions) < 6:
                continue

            # radian을 degree로 변환합니다.
            target_deg = [rad_to_deg(v) for v in point.positions[:6]]

            # MoveIt 실행 속도입니다.
            speed = 45

            # 실제 로봇팔에 관절 이동 명령을 보냅니다.
            with self.lock:
                self.robot.send_angles(target_deg, speed)

            # 너무 길게 기다리지 않고 바로 다음 point로 넘어갑니다.
            time.sleep(0.03)

        # 마지막 안정화 대기입니다.
        time.sleep(0.1)

        # 성공 처리합니다.
        goal_handle.succeed()

        # 결과 메시지를 생성합니다.
        result = FollowJointTrajectory.Result()

        # 성공 코드는 0입니다.
        result.error_code = 0

        # 성공 문자열입니다.
        result.error_string = "Goal executed by robot_arm_server"

        # 결과를 반환합니다.
        return result

    # TCP 요청을 처리합니다.
    def handle_tcp_request(self, request):

        # 명령 이름을 가져옵니다.
        cmd = request.get("cmd", "")

        # 현재 좌표 요청입니다.
        if cmd == "get_coords":
            coords = self.get_coords_safe()
            return {"ok": coords is not None, "coords": coords}

        # 현재 관절값 요청입니다.
        if cmd == "get_angles":
            angles = self.get_angles_safe()
            return {"ok": angles is not None, "angles": angles}

        # 특정 관절 하나만 지정한 각도만큼 이동하는 명령입니다.
        if cmd == "joint_step":

            # 움직일 관절 번호입니다. 1~6을 사용합니다.
            joint_id = int(
                request.get(
                    "joint",
                    0
                )
            )

            # 현재 각도에 더할 이동량입니다.
            delta_deg = float(
                request.get(
                    "delta",
                    1.0
                )
            )

            # 관절 이동 속도입니다.
            speed = int(
                request.get(
                    "speed",
                    35
                )
            )

            # 이동 명령 후 대기 시간입니다.
            wait = float(
                request.get(
                    "wait",
                    0.25
                )
            )

            # 관절 최소 안전 각도입니다.
            minimum_angle = float(
                request.get(
                    "min_angle",
                    -165.0
                )
            )

            # 관절 최대 안전 각도입니다.
            maximum_angle = float(
                request.get(
                    "max_angle",
                    165.0
                )
            )

            # 관절 번호를 검사합니다.
            if joint_id < 1 or joint_id > 6:
                return {
                    "ok": False,
                    "error": f"invalid joint: {joint_id}"
                }

            try:

                # 로봇팔 시리얼 접근을 보호합니다.
                with self.lock:

                    # 현재 실제 관절 각도를 읽습니다.
                    current_angles = self.robot.get_angles()

                    # 관절값 형식을 검사합니다.
                    if (
                        not isinstance(current_angles, list)
                        or len(current_angles) < 6
                    ):
                        return {
                            "ok": False,
                            "error": (
                                "get_angles failed: "
                                f"{current_angles}"
                            )
                        }

                    # 현재 6개 관절값을 실수형으로 복사합니다.
                    target_angles = [
                        float(value)
                        for value in current_angles[:6]
                    ]

                    # 리스트 인덱스는 0부터 시작합니다.
                    joint_index = joint_id - 1

                    # 이동 전 현재 각도입니다.
                    before_angle = target_angles[joint_index]

                    # 해당 관절의 목표 각도를 계산합니다.
                    target_angle = (
                        before_angle
                        + delta_deg
                    )

                    # 목표 각도를 안전 범위로 제한합니다.
                    target_angle = max(
                        minimum_angle,
                        min(
                            maximum_angle,
                            target_angle
                        )
                    )

                    # 목표 리스트에도 해당 관절값을 적용합니다.
                    target_angles[joint_index] = target_angle

                    # send_angle 함수가 있으면 관절 하나만 직접 움직입니다.
                    if hasattr(self.robot, "send_angle"):

                        # joint_id 관절 하나만 이동합니다.
                        self.robot.send_angle(
                            joint_id,
                            target_angle,
                            speed
                        )

                        # 사용한 제어 방법입니다.
                        method = "send_angle"

                    # send_angle이 없는 경우에만 전체 각도 전송을 사용합니다.
                    else:

                        # 변경한 관절을 포함한 전체 각도를 전송합니다.
                        self.robot.send_angles(
                            target_angles,
                            speed
                        )

                        # 사용한 제어 방법입니다.
                        method = "send_angles_fallback"

                    # 마지막 정상 관절값도 새 목표값으로 갱신합니다.
                    self.last_angles = target_angles[:6]

                    # 관절 이동 후 TCP 좌표 캐시는 초기화합니다.
                    self.command_coords = None

                # 필요한 경우 잠시 기다립니다.
                if wait > 0:
                    time.sleep(wait)

                # 성공 결과를 반환합니다.
                return {
                    "ok": True,
                    "joint": joint_id,
                    "before": before_angle,
                    "target": target_angle,
                    "delta": delta_deg,
                    "method": method
                }

            except Exception as e:

                # 오류가 발생해도 서버는 종료하지 않습니다.
                return {
                    "ok": False,
                    "error": str(e)
                }


        # 빠른 수동 조작용 jog 명령입니다.
        if cmd == "jog":

            # 이동 축을 가져옵니다.
            axis = request.get("axis", "z")

            # 한 번에 이동할 거리 mm입니다.
            delta = float(request.get("delta", 1.0))

            # 이동 속도입니다.
            speed = int(request.get("speed", 70))

            # 축 이름을 좌표 인덱스로 변환합니다.
            axis_map = {"x": 0, "y": 1, "z": 2}

            # 잘못된 축이면 실패 응답을 반환합니다.
            if axis not in axis_map:
                return {"ok": False, "error": f"invalid axis: {axis}"}

            try:

                # 시리얼 접근을 lock으로 보호합니다.
                with self.lock:

                    # 목표 좌표 캐시가 없으면 현재 좌표를 한 번만 읽습니다.
                    if self.command_coords is None:

                        # 현재 TCP 좌표를 읽습니다.
                        coords = self.robot.get_coords()

                        # 좌표가 비정상이면 실패 응답을 반환합니다.
                        if not isinstance(coords, list) or len(coords) < 6:
                            return {"ok": False, "error": f"get_coords failed: {coords}"}

                        # 현재 좌표를 목표 좌표 캐시로 저장합니다.
                        self.command_coords = coords[:6]

                    # 선택한 축에 delta를 더합니다.
                    self.command_coords[axis_map[axis]] += delta

                    # 실제 로봇팔에 새 목표 좌표를 보냅니다.
                    self.robot.send_coords(self.command_coords[:6], speed, 1)

                # 바로 응답합니다. 여기서 기다리지 않습니다.
                return {"ok": True, "coords": self.command_coords}

            except Exception as e:

                # 오류가 나도 서버가 죽지 않도록 실패 응답만 반환합니다.
                return {"ok": False, "error": str(e)}


        # 연속 수동 조작 시작 명령입니다.
        if cmd == "jog_start":

            # 이동 축입니다.
            axis = request.get("axis", "z")

            # 이동 방향입니다. 1이면 +, -1이면 -입니다.
            direction_sign = int(request.get("direction", 1))

            # 이동 속도입니다.
            speed = int(request.get("speed", 70))

            # myCobot 좌표축 번호입니다.
            axis_map = {"x": 1, "y": 2, "z": 3, "rx": 4, "ry": 5, "rz": 6}

            # 잘못된 축이면 실패입니다.
            if axis not in axis_map:
                return {"ok": False, "error": f"invalid axis: {axis}"}

            # myCobot jog 방향값입니다.
            direction = 1 if direction_sign > 0 else 0

            try:

                # 시리얼 접근을 lock으로 보호합니다.
                with self.lock:

                    # jog_coord가 있으면 연속 이동을 사용합니다.
                    if hasattr(self.robot, "jog_coord"):

                        # myCobot 연속 좌표 이동 명령입니다.
                        self.robot.jog_coord(axis_map[axis], direction, speed)

                        # 연속 이동 방식 사용 응답입니다.
                        return {"ok": True, "method": "jog_coord"}

                    # jog_coord가 없으면 기존 좌표 이동 방식으로 fallback합니다.
                    if self.command_coords is None:

                        # 현재 TCP 좌표를 읽습니다.
                        coords = self.robot.get_coords()

                        # 좌표 검증입니다.
                        if not isinstance(coords, list) or len(coords) < 6:
                            return {"ok": False, "error": f"get_coords failed: {coords}"}

                        # 현재 좌표를 캐시에 저장합니다.
                        self.command_coords = coords[:6]

                    # fallback 이동량입니다.
                    delta = 2.0 if direction_sign > 0 else -2.0

                    # 좌표 인덱스입니다.
                    coord_index = {"x": 0, "y": 1, "z": 2}[axis]

                    # 목표 좌표를 갱신합니다.
                    self.command_coords[coord_index] += delta

                    # 좌표 이동 명령입니다.
                    self.robot.send_coords(self.command_coords[:6], speed, 1)

                    # fallback 응답입니다.
                    return {"ok": True, "method": "send_coords_fallback"}

            except Exception as e:

                # 오류가 나도 서버를 죽이지 않습니다.
                return {"ok": False, "error": str(e)}

        # 연속 수동 조작 정지 명령입니다.
        if cmd == "jog_stop":

            try:

                # 시리얼 접근을 lock으로 보호합니다.
                with self.lock:

                    # stop 함수가 있으면 사용합니다.
                    if hasattr(self.robot, "stop"):
                        self.robot.stop()

                    # pause 함수가 있으면 보조로 사용합니다.
                    elif hasattr(self.robot, "pause"):
                        self.robot.pause()

                # 성공 응답입니다.
                return {"ok": True}

            except Exception as e:

                # stop 실패도 서버를 죽이지 않습니다.
                return {"ok": False, "error": str(e)}


        # 그리퍼 TCP 기준 한 단계 이동 명령입니다.
        if cmd == "tcp_step":

            # 이동 축입니다. x, y, z 중 하나입니다.
            axis = request.get("axis", "z")

            # 이동 거리 mm입니다.
            delta = float(request.get("delta", 1.0))

            # 이동 속도입니다.
            speed = int(request.get("speed", 80))

            # 좌표 인덱스입니다.
            axis_map = {"x": 0, "y": 1, "z": 2}

            # 잘못된 축이면 실패 응답을 반환합니다.
            if axis not in axis_map:
                return {"ok": False, "error": f"invalid axis: {axis}"}

            try:

                # 시리얼 접근을 lock으로 보호합니다.
                with self.lock:

                    # 목표 좌표 캐시가 없으면 현재 그리퍼 TCP 좌표를 한 번만 읽습니다.
                    if self.command_coords is None:

                        # 현재 TCP 좌표를 읽습니다.
                        coords = self.robot.get_coords()

                        # 좌표가 비정상이면 실패 응답을 반환합니다.
                        if not isinstance(coords, list) or len(coords) < 6:
                            return {"ok": False, "error": f"get_coords failed: {coords}"}

                        # 현재 좌표를 목표 좌표 캐시로 저장합니다.
                        self.command_coords = coords[:6]

                    # x/y/z 중 선택한 축만 변경합니다.
                    self.command_coords[axis_map[axis]] += delta

                    # rx/ry/rz는 그대로 유지한 채 그리퍼 TCP 좌표 이동을 보냅니다.
                    self.robot.send_coords(self.command_coords[:6], speed, 1)

                # 성공 응답입니다.
                return {"ok": True, "coords": self.command_coords[:6]}

            except Exception as e:

                # 오류가 나도 서버가 죽지 않게 실패 응답만 반환합니다.
                return {"ok": False, "error": str(e)}

        # 그리퍼 TCP 좌표 캐시 초기화 명령입니다.
        if cmd == "reset_tcp_cache":

            # lock으로 보호합니다.
            with self.lock:

                # 다음 이동 때 현재 좌표를 다시 읽게 합니다.
                self.command_coords = None

            # 성공 응답입니다.
            return {"ok": True}


        # 좌표 이동 명령입니다.
        if cmd == "send_coords":

            # 목표 좌표입니다.
            coords = request.get("coords", None)

            # 이동 속도입니다.
            speed = int(request.get("speed", 60))

            # 좌표 이동 모드입니다.
            mode = int(request.get("mode", 1))

            # 대기 시간입니다. 기본값은 0입니다.
            wait = float(request.get("wait", 0.0))

            # 좌표 형식을 검사합니다.
            if not isinstance(coords, list) or len(coords) < 6:
                return {"ok": False, "error": "invalid coords"}

            # 실제 로봇팔에 좌표 이동 명령을 보냅니다.
            with self.lock:
                self.command_coords = coords[:6]
                self.robot.send_coords(coords[:6], speed, mode)

            # wait가 있을 때만 대기합니다.
            if wait > 0:
                time.sleep(wait)

            # 성공 응답입니다.
            return {"ok": True}

        # 관절 이동 명령입니다.
        if cmd == "send_angles":

            # 목표 각도입니다.
            angles = request.get("angles", None)

            # 이동 속도입니다.
            speed = int(request.get("speed", 60))

            # 대기 시간입니다. 기본값은 0입니다.
            wait = float(request.get("wait", 0.0))

            # 각도 형식을 검사합니다.
            if not isinstance(angles, list) or len(angles) < 6:
                return {"ok": False, "error": "invalid angles"}

            # 실제 로봇팔에 관절 이동 명령을 보냅니다.
            with self.lock:
                self.command_coords = None
                self.robot.send_angles(angles[:6], speed)

            # wait가 있을 때만 대기합니다.
            if wait > 0:
                time.sleep(wait)

            # 성공 응답입니다.
            return {"ok": True}

        # 그리퍼 제어 명령입니다.
        if cmd == "set_gripper":

            # 그리퍼 목표값입니다.
            value = int(request.get("value", 100))

            # 그리퍼 속도입니다.
            speed = int(request.get("speed", 30))

            # 그리퍼는 너무 빠르게 연속 명령을 보내면 씹힐 수 있어서 기본 대기를 둡니다.
            wait = float(request.get("wait", 0.5))

            # 실제 그리퍼 명령을 보냅니다.
            with self.lock:
                self.robot.set_gripper_value(value, speed)

            # wait가 있을 때만 대기합니다.
            if wait > 0:
                time.sleep(wait)

            # 성공 응답입니다.
            return {"ok": True}

        # 알 수 없는 명령입니다.
        return {"ok": False, "error": f"unknown cmd: {cmd}"}

    # TCP 서버 루프입니다.
    def tcp_server_loop(self):

        # TCP 소켓을 생성합니다.
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 포트 재사용 옵션입니다.
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 서버 주소를 바인딩합니다.
        server.bind((TCP_HOST, TCP_PORT))

        # 접속 대기를 시작합니다.
        server.listen(5)

        # 시작 로그입니다.
        self.get_logger().info("TCP server loop started")

        # 계속 클라이언트 요청을 받습니다.
        while True:

            # 클라이언트 연결을 받습니다.
            conn, _ = server.accept()

            # 연결을 자동으로 닫기 위해 with를 사용합니다.
            with conn:

                # 수신 버퍼입니다.
                data = b""

                # 한 줄 JSON을 받을 때까지 반복합니다.
                while not data.endswith(b"\n"):

                    # 데이터를 받습니다.
                    chunk = conn.recv(4096)

                    # 데이터가 없으면 중단합니다.
                    if not chunk:
                        break

                    # 버퍼에 추가합니다.
                    data += chunk

                # 수신 데이터가 없으면 건너뜁니다.
                if not data:
                    continue

                try:

                    # JSON 요청을 파싱합니다.
                    request = json.loads(data.decode("utf-8").strip())

                    # 요청을 처리합니다.
                    response = self.handle_tcp_request(request)

                except Exception as e:

                    # 오류 응답을 생성합니다.
                    response = {"ok": False, "error": str(e)}

                # JSON 응답을 전송합니다.
                conn.sendall((json.dumps(response) + "\n").encode("utf-8"))


# 메인 함수입니다.
def main():

    # ROS2를 초기화합니다.
    rclpy.init()

    # 서버 노드를 생성합니다.
    node = RobotArmServer()

    try:

        # 노드를 계속 실행합니다.
        rclpy.spin(node)

    finally:

        # 노드를 정리합니다.
        node.destroy_node()

        # ROS2를 종료합니다.
        rclpy.shutdown()


# 직접 실행할 때 main을 실행합니다.
if __name__ == "__main__":
    main()
