import time
import numpy as np

from pymycobot.mycobot280 import MyCobot280

PORT = "/dev/ttyUSB0"
BAUD = 1_000_000

mc = MyCobot280(PORT, BAUD)
time.sleep(2)


def read_coords(label):
    coords = mc.get_coords()
    print(f"{label}: {coords}")
    return np.array(coords[:3], dtype=float)


# 1. 플랜지 좌표 기준으로 읽기
mc.set_tool_reference([0, 0, 0, 0, 0, 0])
mc.set_end_type(0)  # FLANGE
time.sleep(0.5)

flange_xyz = read_coords("플랜지 위치")

tests = {
    "+Tool X": [20, 0, 0, 0, 0, 0],
    "+Tool Y": [0, 20, 0, 0, 0, 0],
    "+Tool Z": [0, 0, 20, 0, 0, 0],
}

for name, tool_offset in tests.items():
    mc.set_tool_reference(tool_offset)
    mc.set_end_type(1)  # TOOL
    time.sleep(0.5)

    tcp_xyz = read_coords(name)
    difference = tcp_xyz - flange_xyz

    print(f"{name}의 현재 베이스 좌표계 방향: {difference}")
    print("-" * 50)

# 원상 복구
mc.set_tool_reference([0, 0, 0, 0, 0, 0])
mc.set_end_type(0)