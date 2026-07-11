
import math
import time

from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

from ._config import (
    SPEED, VERTICAL_RX, VERTICAL_RY, VERTICAL_RZ, 
    DESIRED_IMAGE_ANGLE, J6_SIGN, J6_MIN, J6_MAX, WAIT)
from ._angle_utils import normalize_angle, clamp
#==============================================#
# 로봇 연결 함수 

def connect_robot(port, baud):
    mc = MyCobot280(port, baud)
    time.sleep(WAIT)
    print("로봇 연결 완료")
    return mc

#==============================================#
# Photo Pose로 이동

PHOTO_ANGLE = [0, 40, -40, -60, 2, -45]

def move_photo_pose(mc, speed=SPEED):
    print("사진 촬영 자세로 이동합니다.")

    mc.send_angles(PHOTO_ANGLE, speed)

    time.sleep(WAIT)  # 로봇팔 움직임 대기
    print("사진 촬영 자세 이동 완료")

#==============================================#
# coords (좌표값) 로 이동

def move_coords(mc, coords, speed=SPEED, mode=1, wait=WAIT):
    if coords is None or len(coords) < 6:
        print("이동 좌표가 올바르지 않습니다.")
        return False

    print("이동:", coords)
    mc.send_coords(coords, speed, mode)
    time.sleep(wait)

    return True

#==============================================#
# 그리퍼 open 함수

def open_gripper(mc):
    print("그리퍼 열기")
    mc.set_gripper_value(100, 30)
    time.sleep(WAIT)

#==============================================#
# 그리퍼 close 함수

def close_gripper(mc):
    print("그리퍼 닫기")
    mc.set_gripper_value(20, 30)  # 필요시 20 수정 필요
    time.sleep(WAIT)

#==============================================#
"""
    현재 x, y, z 위치는 유지하고
    rx, ry, rz 자세만 변경하는 함수

    target_rx, target_ry, target_rz 중 None인 값은 현재 자세를 유지함
"""

def change_only_rpy(mc, target_rx=None, target_ry=None, target_rz=None, speed=SPEED, wait=WAIT):
    
    coords = mc.get_coords()

    if coords is None or len(coords) < 6:
        print("좌표를 읽지 못했습니다.")
        return False

    x, y, z, rx, ry, rz = coords

    new_rx = rx if target_rx is None else target_rx
    new_ry = ry if target_ry is None else target_ry
    new_rz = rz if target_rz is None else target_rz

    target = [x, y, z, new_rx, new_ry, new_rz]

    mc.send_coords(target, speed, 1)
    time.sleep(wait)

    return True

#==============================================#
"""
    현재 rx, ry, rz 자세를 유지한 채 x, y, z로 이동 / #필요없을듯
"""
    
def move_to_xy_keep_current_rxyz(mc, x, y, z, speed=SPEED):
    coords = mc.get_coords()

    if coords is None or len(coords) < 6:
        print("현재 좌표를 읽지 못했습니다.")
        return False

    rx, ry, rz = coords[3], coords[4], coords[5]

    target = [x, y, z, rx, ry, rz]
    print("현재 자세 유지 이동:", target)

    mc.send_coords(target, speed, 1)
    time.sleep(2)

    return True

#===============================================#
 
"""
    그리퍼를 지면과 수직인 자세로 변경하는 함수
    위치 x, y, z는 유지하고 rx, ry, rz만 고정값으로 변경
"""

def _rpy_to_quaternion(rx, ry, rz):
    """Convert degree RPY to a normalized XYZW quaternion."""
    roll, pitch, yaw = map(math.radians, (rx, ry, rz))
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _orientation_error_degrees(current_rpy, target_rpy):
    """Return the shortest 3D rotation difference between two RPY poses."""
    current_q = _rpy_to_quaternion(*current_rpy)
    target_q = _rpy_to_quaternion(*target_rpy)
    dot = abs(sum(a * b for a, b in zip(current_q, target_q)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def move_vertical_gripper(
    mc,
    speed=SPEED,
    wait=WAIT,
    max_attempts=3,
    orientation_tolerance=8.0,  # 그리퍼 수직상태 허용 오차 (degree)
):
    """Move to the vertical RPY and verify the measured pose, with retries."""
    target_rpy = (VERTICAL_RX, VERTICAL_RY, VERTICAL_RZ)

    for attempt in range(1, max_attempts + 1):
        print(f"그리퍼 수직 자세 변경 시도 {attempt}/{max_attempts}")

        command_sent = change_only_rpy(
            mc,
            target_rx=VERTICAL_RX,
            target_ry=VERTICAL_RY,
            target_rz=VERTICAL_RZ,
            speed=speed,
            wait=wait,
        )
        if not command_sent:
            print("수직 자세 명령 전송 실패")
            continue

        actual_coords = mc.get_coords()
        if not isinstance(actual_coords, (list, tuple)) or len(actual_coords) < 6:
            print(f"수직 자세 확인 실패: 현재 좌표={actual_coords}")
            continue

        actual_rpy = tuple(actual_coords[3:6])
        error = _orientation_error_degrees(actual_rpy, target_rpy)
        print(
            f"수직 자세 확인: 실제 RPY={actual_rpy}, "
            f"목표와 회전 오차={error:.2f}도"
        )

        if error <= orientation_tolerance:
            print("그리퍼 수직 자세 변경 및 확인 완료")
            return True

        print(
            f"수직 자세 미도달(허용 오차 {orientation_tolerance:.1f}도), "
            "명령을 다시 실행합니다."
        )

    print(f"그리퍼 수직 자세 변경 실패: {max_attempts}회 시도 후 미도달")
    return False
#===============================================#

"""
    카메라에서 검출한 컨테이너 각도를 이용해서
    현재 J6 각도에서 상대적으로 보정 회전한다.
"""

def rotate_j6_by_camera_angle(mc, camera_angle, speed=SPEED, wait=WAIT):

    angles = mc.get_angles()

    if angles is None or len(angles) < 6:
        print("현재 관절각을 읽지 못했습니다.")
        return False

    current_j6 = angles[5]

    error_angle = normalize_angle(camera_angle - DESIRED_IMAGE_ANGLE)

    correction_angle = J6_SIGN * error_angle

    target_j6 = current_j6 + correction_angle
    target_j6 = clamp(target_j6, J6_MIN, J6_MAX)

    print("================================")
    print(f"현재 J6 각도: {current_j6:.1f}도")
    print(f"카메라 검출 각도: {camera_angle:.1f}도")
    print(f"목표 이미지 각도: {DESIRED_IMAGE_ANGLE:.1f}도")
    print(f"J6 보정각: {correction_angle:.1f}도")
    print(f"J6 목표각: {target_j6:.1f}도")

    mc.send_angle(Angle.J6.value, target_j6, speed)
    time.sleep(wait)

    print("회전 후 관절각:", mc.get_angles())

    return True
#===============================================#
def move_z_keep_current_pose(mc, z, speed=SPEED, wait=WAIT):
    """
    현재 x, y, rx, ry, rz를 유지한 채 z만 이동
    J6 보정 후 내려갈 때 사용
    """
    coords = mc.get_coords()

    if coords is None or len(coords) < 6:
        print("현재 좌표를 읽지 못했습니다.")
        return False

    target = [
        coords[0],
        coords[1],
        z,
        coords[3],
        coords[4],
        coords[5]]
    
    print("Z만 이동:", target)

    mc.send_coords(target, speed, 1)
    time.sleep(wait)

    return True
    
#===========================================#
# 잡기와 놓기
#===========================================#
