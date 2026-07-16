"""Small helpers shared by nodes that open the JetCobot serial port."""

import time

from pymycobot.mycobot280 import MyCobot280

from ._config import WAIT


def connect_robot(port, baud):
    """Open the serial connection and allow the controller to initialize."""
    robot = MyCobot280(port, baud)
    time.sleep(WAIT)
    print('로봇 연결 완료')
    return robot
