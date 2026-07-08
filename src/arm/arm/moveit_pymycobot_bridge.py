#!/usr/bin/env python3

# math는 degree와 radian 변환에 사용합니다.
import math

# time은 로봇팔 이동 대기 시간 계산에 사용합니다.
import time

# ROS2 파이썬 라이브러리입니다.
import rclpy

# ROS2 노드 클래스입니다.
from rclpy.node import Node

# ROS2 Action Server를 만들기 위한 클래스입니다.
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# FollowJointTrajectory 액션 타입입니다.
from control_msgs.action import FollowJointTrajectory

# JointState 메시지 타입입니다.
from sensor_msgs.msg import JointState

# MyCobot280 실제 로봇팔 제어 클래스입니다.
from pymycobot.mycobot280 import MyCobot280


# MoveIt trajectory 명령을 실제 MyCobot280 명령으로 변환하는 노드입니다.
class MoveItPymycobotBridge(Node):

    # 노드 초기화 함수입니다.
    def __init__(self):

        # ROS2 노드 이름을 설정합니다.
        super().__init__("moveit_pymycobot_bridge")

        # 실제 로봇팔 포트입니다.
        self.port = "/dev/ttyUSB0"

        # 실제 로봇팔 통신 속도입니다.
        self.baud = 1000000

        # MoveIt과 URDF에서 사용하는 관절 이름입니다.
        self.joint_names = [
            "1_Joint",
            "2_Joint",
            "3_Joint",
            "4_Joint",
            "5_Joint",
            "6_Joint",
        ]

        # 로봇팔 이동 속도입니다. 처음에는 낮게 둡니다.
        self.speed = 30

        # 실제 로봇팔 객체를 생성합니다.
        self.robot = MyCobot280(self.port, self.baud)

        # 연결 안정화를 위해 잠깐 기다립니다.
        time.sleep(1.0)

        # JointState 퍼블리셔를 생성합니다.
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # 10Hz로 실제 로봇팔 각도를 읽어서 /joint_states로 발행합니다.
        self.joint_timer = self.create_timer(0.1, self.publish_joint_states)

        # MoveIt이 Execute할 때 사용하는 FollowJointTrajectory 액션 서버를 생성합니다.
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_group_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # 시작 로그를 출력합니다.
        self.get_logger().info("moveit_pymycobot_bridge started")

        # 실제 로봇팔 연결 정보를 출력합니다.
        self.get_logger().info(f"Real robot: MyCobot280({self.port}, {self.baud})")

        # 액션 서버 이름을 출력합니다.
        self.get_logger().info("Action server: /arm_group_controller/follow_joint_trajectory")

        # joint_states 발행 정보를 출력합니다.
        self.get_logger().info("Publishing real joint states: /joint_states")

    # degree를 radian으로 변환하는 함수입니다.
    def deg_to_rad(self, value_deg):

        # degree를 radian으로 바꿔 반환합니다.
        return float(value_deg) * math.pi / 180.0

    # radian을 degree로 변환하는 함수입니다.
    def rad_to_deg(self, value_rad):

        # radian을 degree로 바꿔 반환합니다.
        return float(value_rad) * 180.0 / math.pi

    # 실제 로봇팔 각도를 읽고 /joint_states로 발행하는 함수입니다.
    def publish_joint_states(self):

        try:
            # 실제 로봇팔 현재 각도를 degree 단위로 읽습니다.
            angles_deg = self.robot.get_angles()

            # 읽기 실패 시 -1이 나올 수 있으므로 검사합니다.
            if not isinstance(angles_deg, list) or len(angles_deg) < 6:
                return

            # JointState 메시지를 생성합니다.
            msg = JointState()

            # 현재 ROS 시간을 넣습니다.
            msg.header.stamp = self.get_clock().now().to_msg()

            # 관절 이름을 넣습니다.
            msg.name = self.joint_names

            # degree 각도를 radian으로 변환해서 넣습니다.
            msg.position = [self.deg_to_rad(a) for a in angles_deg[:6]]

            # 속도값은 사용하지 않으므로 0으로 채웁니다.
            msg.velocity = [0.0] * 6

            # effort값은 사용하지 않으므로 0으로 채웁니다.
            msg.effort = [0.0] * 6

            # /joint_states로 발행합니다.
            self.joint_pub.publish(msg)

        except Exception as e:
            # 너무 자주 에러 로그가 나오지 않도록 간단히 경고만 출력합니다.
            self.get_logger().warn(f"joint state read failed: {e}")

    # MoveIt에서 goal이 들어왔을 때 수락 여부를 결정하는 함수입니다.
    def goal_callback(self, goal_request):

        # 관절 이름을 가져옵니다.
        requested_joints = list(goal_request.trajectory.joint_names)

        # 관절 이름이 비어 있으면 거부합니다.
        if not requested_joints:
            self.get_logger().error("Rejected goal: empty joint_names")
            return GoalResponse.REJECT

        # 필요한 관절이 모두 포함되어 있는지 확인합니다.
        for name in self.joint_names:
            if name not in requested_joints:
                self.get_logger().error(f"Rejected goal: missing joint {name}")
                return GoalResponse.REJECT

        # 정상 goal이면 수락합니다.
        self.get_logger().info("Accepted trajectory goal")
        return GoalResponse.ACCEPT

    # 취소 요청이 들어왔을 때 처리하는 함수입니다.
    def cancel_callback(self, goal_handle):

        # 취소 요청을 수락합니다.
        self.get_logger().warn("Cancel requested")
        return CancelResponse.ACCEPT

    # MoveIt trajectory를 실제 로봇팔 명령으로 실행하는 함수입니다.
    def execute_callback(self, goal_handle):

        # goal에서 trajectory를 꺼냅니다.
        trajectory = goal_handle.request.trajectory

        # trajectory의 관절 이름 순서를 가져옵니다.
        incoming_joint_names = list(trajectory.joint_names)

        # 결과 메시지를 생성합니다.
        result = FollowJointTrajectory.Result()

        # trajectory point가 없으면 실패 처리합니다.
        if not trajectory.points:
            self.get_logger().error("Trajectory has no points")
            result.error_code = -1
            result.error_string = "Trajectory has no points"
            goal_handle.abort()
            return result

        # 각 관절 이름이 trajectory에서 몇 번째 인덱스인지 저장합니다.
        joint_index_map = {}

        # 필요한 관절 이름을 하나씩 확인합니다.
        for joint_name in self.joint_names:

            # 현재 관절 이름의 인덱스를 찾습니다.
            joint_index_map[joint_name] = incoming_joint_names.index(joint_name)

        # 이전 point 시간을 저장합니다.
        prev_time_sec = 0.0

        # trajectory point들을 순서대로 실행합니다.
        for idx, point in enumerate(trajectory.points):

            # 취소 요청이 들어왔으면 중단합니다.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = -1
                result.error_string = "Goal canceled"
                return result

            # position 값이 6개 미만이면 실패 처리합니다.
            if len(point.positions) < 6:
                self.get_logger().error("Point positions length is too short")
                result.error_code = -1
                result.error_string = "Point positions length is too short"
                goal_handle.abort()
                return result

            # MoveIt의 radian 값을 MyCobot의 degree 값으로 변환합니다.
            target_deg = []

            # 관절 순서를 URDF 기준 순서로 맞춥니다.
            for joint_name in self.joint_names:

                # trajectory 안에서 해당 관절의 인덱스를 가져옵니다.
                input_index = joint_index_map[joint_name]

                # radian 위치값을 가져옵니다.
                value_rad = point.positions[input_index]

                # degree로 변환해서 리스트에 추가합니다.
                target_deg.append(self.rad_to_deg(value_rad))

            # 현재 point의 목표 시간을 초 단위로 계산합니다.
            target_time_sec = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9

            # 이전 point 대비 기다릴 시간을 계산합니다.
            wait_sec = max(0.1, target_time_sec - prev_time_sec)

            # 이전 시간을 갱신합니다.
            prev_time_sec = target_time_sec

            # 실행 로그를 출력합니다.
            self.get_logger().info(f"Point {idx + 1}/{len(trajectory.points)} -> deg: {[round(v, 2) for v in target_deg]}")

            # 실제 로봇팔에 관절 각도 명령을 보냅니다.
            self.robot.send_angles(target_deg, self.speed)

            # 로봇팔이 움직일 시간을 줍니다.
            time.sleep(min(wait_sec, 0.2))

        # 마지막 이동 안정화 대기입니다.
        time.sleep(0.5)

        # 성공 처리합니다.
        goal_handle.succeed()

        # FollowJointTrajectory 성공 코드는 0입니다.
        result.error_code = 0

        # 성공 문자열입니다.
        result.error_string = "Goal successfully executed on real MyCobot280"

        # 결과를 반환합니다.
        return result

    # 노드 종료 시 정리 함수입니다.
    def destroy_node(self):

        # 정리 로그를 출력합니다.
        self.get_logger().info("Shutting down moveit_pymycobot_bridge")

        # 부모 클래스 종료 함수를 호출합니다.
        super().destroy_node()


# 메인 함수입니다.
def main(args=None):

    # ROS2 파이썬 시스템을 초기화합니다.
    rclpy.init(args=args)

    # 브릿지 노드를 생성합니다.
    node = MoveItPymycobotBridge()

    try:
        # 노드를 계속 실행합니다.
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Ctrl+C 종료는 조용히 처리합니다.
        pass

    finally:
        # 노드를 정리합니다.
        node.destroy_node()

        # ROS2가 살아 있으면 종료합니다.
        if rclpy.ok():
            rclpy.shutdown()


# 직접 실행 시 main을 호출합니다.
if __name__ == "__main__":
    main()
