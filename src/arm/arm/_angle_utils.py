#=============================================#
# angle을 +-90도로 정리 

def normalize_angle(angle):
    while angle < -90:
        angle += 180

    while angle >= 90:
        angle -= 180

    return angle

#=============================================#
"""
    value를 min_value ~ max_value 범위 안으로 제한
"""
def clamp(value, min_value, max_value):

    return max(min_value, min(max_value, value))

#=============================================#
