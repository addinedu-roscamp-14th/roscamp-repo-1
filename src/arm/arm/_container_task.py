import time

from ._config import PICK_Z1, PLACE_Z1, SAFE_Z, WAIT
from ._vision_utils import find_clicked_container, pixel_to_robot
from ._robot_utils import (
    move_coords,
    move_z_keep_current_pose,
    open_gripper,
    close_gripper,
    move_vertical_gripper,
    rotate_j6_by_camera_angle,
    move_photo_pose,
)


def move_above_pixel(mc, pixel_x, pixel_y):
    """
    클릭한 픽셀 위치 위의 SAFE_Z 높이로 이동
    """
    robot_x, robot_y = pixel_to_robot(pixel_x, pixel_y)

    coords = mc.get_coords()

    if coords is None or len(coords) < 6:
        print("현재 좌표를 읽지 못했습니다.")
        return False

    target = [
        robot_x,
        robot_y,
        SAFE_Z,
        coords[3],
        coords[4],
        coords[5],
    ]

    return move_coords(mc, target)


def pick_clicked_container(mc, frame, click_x, click_y):
    """
    사진에서 좌클릭한 컨테이너를 찾아 집는 함수
    """

    result = find_clicked_container(frame, click_x, click_y)

    if result is None:
        print("클릭한 위치에서 컨테이너를 찾지 못했습니다.")
        return False

    center_u, center_v = result["center_pixel"]
    container_angle = result["angle"]

    robot_x, robot_y = pixel_to_robot(center_u, center_v)

    print("================================")
    print("좌클릭 집기")
    print(f"클릭 픽셀: ({click_x}, {click_y})")
    print(f"컨테이너 중심 픽셀: ({center_u:.1f}, {center_v:.1f})")
    print(f"로봇 좌표: x={robot_x:.1f}, y={robot_y:.1f}")
    print(f"컨테이너 회전각: {container_angle:.1f}도")

    # 1. 그리퍼 열기
    open_gripper(mc)

    # 2. 컨테이너 중심 위 SAFE_Z로 이동
    coords = mc.get_coords()

    if coords is None or len(coords) < 6:
        print("현재 좌표를 읽지 못했습니다.")
        return False

    safe_target = [
        robot_x,
        robot_y,
        SAFE_Z,
        coords[3],
        coords[4],
        coords[5],
    ]

    ok = move_coords(mc, safe_target)
    if not ok:
        return False

    # 3. 그리퍼 수직 자세
    time.sleep(WAIT)
    ok = move_vertical_gripper(mc)
    if not ok:
        return False

    # 4. 컨테이너 각도에 맞춰 J6 보정
    ok = rotate_j6_by_camera_angle(mc, container_angle)
    if not ok:
        return False

    # 5. 집기 높이로 하강
    ok = move_z_keep_current_pose(mc, PICK_Z1)
    if not ok:
        return False

    # 6. 그리퍼 닫기
    close_gripper(mc)

    # 7. 안전 높이로 상승
    ok = move_z_keep_current_pose(mc, SAFE_Z)
    if not ok:
        return False

    print("집기 완료")

    # 8. 안정적인 자세로 이동
    move_photo_pose(mc, speed=30)

    return True


def place_clicked_position(mc, click_x, click_y, place_z=PLACE_Z1):
    """
    사진에서 우클릭한 위치에 현재 들고 있는 컨테이너를 놓는 함수
    """

    robot_x, robot_y = pixel_to_robot(click_x, click_y)

    print("================================")
    print("우클릭 놓기")
    print(f"클릭 픽셀: ({click_x}, {click_y})")
    print(f"놓을 로봇 좌표: x={robot_x:.1f}, y={robot_y:.1f}")
    print(f"놓기 Z 높이: {place_z}")

    # 1. 우클릭 위치 위 SAFE_Z로 이동
    coords = mc.get_coords()

    if coords is None or len(coords) < 6:
        print("현재 좌표를 읽지 못했습니다.")
        return False

    safe_target = [
        robot_x,
        robot_y,
        SAFE_Z,
        coords[3],
        coords[4],
        coords[5],
    ]

    ok = move_coords(mc, safe_target)
    if not ok:
        return False

    # 2. XY 이동 명령이 안정적으로 끝난 뒤 그리퍼 수직 자세 적용
    time.sleep(WAIT)
    ok = move_vertical_gripper(mc)
    if not ok:
        return False

    # 3. 놓기 높이로 하강
    ok = move_z_keep_current_pose(mc, place_z)
    if not ok:
        return False

    # 4. 그리퍼 열기
    open_gripper(mc)

    # 5. 안전 높이로 상승
    ok = move_z_keep_current_pose(mc, SAFE_Z)
    if not ok:
        return False

    print("놓기 완료")

    # 6. 안정적인 자세로 복귀
    move_photo_pose(mc, speed=30)

    return True
