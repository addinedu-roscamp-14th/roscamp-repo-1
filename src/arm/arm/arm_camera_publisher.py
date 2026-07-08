#!/usr/bin/env python3

import cv2  # OpenCV로 USB 카메라를 열고 화면을 출력하기 위한 라이브러리입니다.
import rclpy  # ROS2 파이썬 노드를 만들기 위한 라이브러리입니다.

from rclpy.node import Node  # ROS2 노드 클래스를 가져옵니다.
from sensor_msgs.msg import Image  # ROS2 이미지 메시지 타입을 가져옵니다.


class ArmCameraPublisher(Node):  # 로봇팔 카메라 영상을 ROS2 토픽으로 발행하고 화면에도 출력하는 노드입니다.

    def __init__(self):  # 노드가 처음 실행될 때 동작하는 초기화 함수입니다.
        super().__init__("arm_camera_publisher")  # ROS2 노드 이름을 설정합니다.

        self.video_device = "/dev/video3"  # 로봇팔 USB 카메라 장치 경로입니다.
        self.frame_id = "arm_camera_frame"  # ROS2 이미지 메시지에 들어갈 카메라 프레임 이름입니다.
        self.image_topic = "/arm_camera/image_raw"  # 카메라 영상을 발행할 ROS2 토픽 이름입니다.

        self.width = 640  # 카메라 가로 해상도입니다.
        self.height = 480  # 카메라 세로 해상도입니다.
        self.fps = 30  # 목표 FPS입니다.

        self.show_window = True  # True이면 OpenCV 화면 창을 띄웁니다.
        self.window_name = "arm_camera"  # OpenCV 화면 창 이름입니다.
        self.stop_requested = False  # q 또는 ESC로 종료 요청이 들어왔는지 저장하는 변수입니다.

        self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)  # V4L2 방식으로 USB 카메라를 엽니다.

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)  # 카메라 가로 해상도를 설정합니다.
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)  # 카메라 세로 해상도를 설정합니다.
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)  # 카메라 FPS를 설정합니다.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("Y", "U", "Y", "V"))  # 카메라 포맷을 YUYV로 설정합니다.

        if not self.cap.isOpened():  # 카메라가 정상적으로 열렸는지 확인합니다.
            self.get_logger().error(f"카메라를 열 수 없습니다: {self.video_device}")  # 실패 로그를 출력합니다.
            raise RuntimeError(f"카메라 열기 실패: {self.video_device}")  # 프로그램을 중단합니다.

        self.image_pub = self.create_publisher(
            Image,
            self.image_topic,
            10
        )  # ROS2 Image 메시지를 발행할 퍼블리셔를 생성합니다.

        self.timer = self.create_timer(
            1.0 / float(self.fps),
            self.timer_callback
        )  # 설정한 FPS 주기로 timer_callback 함수를 실행합니다.

        if self.show_window:  # 화면 출력 기능이 켜져 있으면 실행합니다.
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  # 크기 조절 가능한 OpenCV 창을 생성합니다.

        self.get_logger().info("arm_camera_publisher started")  # 노드 시작 로그를 출력합니다.
        self.get_logger().info(f"video device: {self.video_device}")  # 사용 중인 카메라 장치를 출력합니다.
        self.get_logger().info(f"publishing topic: {self.image_topic}")  # 발행 중인 토픽 이름을 출력합니다.
        self.get_logger().info("OpenCV window enabled. Press q or ESC to quit.")  # 화면 종료 키 안내를 출력합니다.

    def timer_callback(self):  # 설정한 FPS마다 실행되는 함수입니다.
        ret, frame = self.cap.read()  # 카메라에서 한 장의 프레임을 읽습니다.

        if not ret or frame is None:  # 프레임 읽기에 실패했는지 확인합니다.
            self.get_logger().warn("카메라 프레임을 읽지 못했습니다.")  # 경고 로그를 출력합니다.
            return  # 이번 주기 처리를 건너뜁니다.

        if len(frame.shape) == 2:  # 흑백 1채널 영상인지 확인합니다.
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)  # 흑백 영상을 BGR 3채널 영상으로 변환합니다.

        if len(frame.shape) == 3 and frame.shape[2] == 4:  # BGRA 4채널 영상인지 확인합니다.
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # BGRA 영상을 BGR 3채널 영상으로 변환합니다.

        frame = frame.copy()  # ROS2 메시지 변환을 위해 연속된 메모리 배열로 복사합니다.

        msg = Image()  # ROS2 Image 메시지 객체를 생성합니다.
        msg.header.stamp = self.get_clock().now().to_msg()  # 현재 ROS2 시간을 메시지에 넣습니다.
        msg.header.frame_id = self.frame_id  # 카메라 프레임 이름을 메시지에 넣습니다.

        msg.height = frame.shape[0]  # 이미지 높이를 메시지에 넣습니다.
        msg.width = frame.shape[1]  # 이미지 너비를 메시지에 넣습니다.
        msg.encoding = "bgr8"  # OpenCV 기본 색상 형식인 BGR 8비트 3채널로 설정합니다.
        msg.is_bigendian = 0  # 일반 PC는 little-endian이므로 0으로 설정합니다.
        msg.step = frame.shape[1] * 3  # 이미지 한 줄의 바이트 수를 설정합니다.
        msg.data = frame.tobytes()  # OpenCV 이미지를 바이트 데이터로 변환해서 메시지에 넣습니다.

        self.image_pub.publish(msg)  # 카메라 이미지를 ROS2 토픽으로 발행합니다.

        if self.show_window:  # 화면 출력 기능이 켜져 있으면 실행합니다.
            cv2.imshow(self.window_name, frame)  # OpenCV 창에 현재 프레임을 출력합니다.

            key = cv2.waitKey(1) & 0xFF  # 키 입력을 1ms 동안 확인합니다.

            if key == ord("q") or key == 27:  # q 또는 ESC 키가 눌렸는지 확인합니다.
                self.get_logger().info("종료 키 입력됨. 카메라 노드를 종료합니다.")  # 종료 로그를 출력합니다.
                self.stop_requested = True  # 메인 루프에 종료 요청을 전달합니다.

    def destroy_node(self):  # 노드가 종료될 때 실행되는 정리 함수입니다.
        if hasattr(self, "timer") and self.timer is not None:  # 타이머가 존재하는지 확인합니다.
            self.timer.cancel()  # 타이머를 중지합니다.

        if hasattr(self, "cap") and self.cap is not None:  # 카메라 객체가 존재하는지 확인합니다.
            self.cap.release()  # 카메라 장치를 해제합니다.

        cv2.destroyAllWindows()  # OpenCV로 열린 모든 창을 닫습니다.

        super().destroy_node()  # ROS2 노드를 정상적으로 종료합니다.


def main(args=None):  # 프로그램 시작 함수입니다.
    rclpy.init(args=args)  # ROS2 파이썬 시스템을 초기화합니다.

    node = ArmCameraPublisher()  # 카메라 퍼블리셔 노드를 생성합니다.

    try:  # Ctrl+C 또는 q 종료를 처리하기 위한 예외 처리 구조입니다.
        while rclpy.ok() and not node.stop_requested:  # ROS2가 살아 있고 종료 요청이 없으면 반복합니다.
            rclpy.spin_once(node, timeout_sec=0.1)  # ROS2 콜백을 한 번씩 처리합니다.

    except KeyboardInterrupt:  # Ctrl+C가 눌렸을 때 실행됩니다.
        pass  # 조용히 종료합니다.

    finally:  # 프로그램이 끝날 때 항상 실행됩니다.
        node.destroy_node()  # 노드와 카메라 장치를 정리합니다.

        if rclpy.ok():  # ROS2가 아직 종료되지 않았는지 확인합니다.
            rclpy.shutdown()  # ROS2 시스템을 종료합니다.


if __name__ == "__main__":  # 이 파일을 직접 실행했을 때만 아래 코드를 실행합니다.
    main()  # main 함수를 실행합니다.
