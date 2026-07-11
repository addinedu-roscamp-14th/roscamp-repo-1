import cv2
import numpy as np
import math

from ._config import H, MIN_AREA, ROBOT_X_OFFSET, ROBOT_Y_OFFSET
from ._angle_utils import normalize_angle


#=============================================#
# 회전사각형의 가장긴변을 기준으로 그 변의 각도를 구함

def get_long_side_angle(box):
    edges = []

    for i in range(4):
        p1 = box[i]
        p2 = box[(i + 1) % 4]

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        length = math.sqrt(dx * dx + dy * dy)
        angle = math.degrees(math.atan2(dy, dx))

        edges.append((length, angle))

    longest_edge = max(edges, key=lambda x: x[0])
    angle = longest_edge[1]

    return normalize_angle(angle)

#===============================================#


def find_clicked_container(frame, click_x, click_y):
    """
    클릭한 위치에 있는 컨테이너 contour를 찾고
    중심 좌표와 회전각 반환
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  #이미지를 Gray(흑백)으로 함

    blur = cv2.GaussianBlur(gray, (5, 5), 0)  #이미지를 블러(흐리게)함

    edges = cv2.Canny(blur, 50, 150)  #컨테이너의 Edge검출

    kernel = np.ones((5, 5), np.uint8)  # 5X5크기의 커널을 만듬 (커널은 이미지 보정을 위한 행렬 / 선형대수학)
    edges = cv2.dilate(edges, kernel, iterations=1)  #선명도를 높이기 위해 엣지를 두껍게함

    contours, _ = cv2.findContours(   #윤곽선 검출 / 외부윤곽선만 검출
        edges,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []  #빈 리스트선언

    for contour in contours:
        area = cv2.contourArea(contour)  #윤곽선의 면적 계산

        if area < MIN_AREA:
            continue   #너무 작으면 무시 / MIN AREA 보다 작으면

        inside = cv2.pointPolygonTest(  # 클릭한 점이 contour 내부에 있는지 확인 (매우중요)
            contour,
            (float(click_x), float(click_y)),
            False)

        if inside >= 0:
            candidates.append(contour)

    if len(candidates) == 0:   #컨테이너가 없는 공간을 클릭하면 None반환
        return None

    # 클릭 위치를 포함하는 contour 중 가장 큰 것을 선택 / 여러 외곽선(컨테이너 포함) 중 가장 큰거 선택이라는 뜻
    target_contour = max(candidates, key=cv2.contourArea)

    rect = cv2.minAreaRect(target_contour)  # 비스듬한 사각형도 구함 / 구한 외곽선(contour)을 포함하는 가장 작은 회전 사각형을 구함
    box = cv2.boxPoints(rect)  #결과 / 꼭짓점 좌표 형태
    box = np.intp(box)  #결과 값을 정수로 변환

    center_x, center_y = rect[0]  #중심 각도와 좌표 구하기
    angle = get_long_side_angle(box)  #get_long은 회전각을 검출하는 함수 (사진한방 찍음)

    return {
        "contour": target_contour,
        "box": box,
        "center_pixel": (center_x, center_y),
        "angle": angle
    }

#=============================================#
def pixel_to_robot(u, v):
    """
    카메라 픽셀 좌표를 로봇 XY 좌표로 변환
    H는 반드시 현재 카메라 화면 기준으로 보정된 값이어야 함
    """
    point = np.array([[[u, v]]], dtype=np.float32)
    robot_point = cv2.perspectiveTransform(point, H)

    x = robot_point[0][0][0] + ROBOT_X_OFFSET
    y = robot_point[0][0][1] + ROBOT_Y_OFFSET

    return float(x), float(y)
