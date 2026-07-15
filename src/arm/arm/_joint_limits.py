"""JetCobot joint limits shared by planning and hardware validation."""

import math


# Keep these values synchronized with jetcobot_description/urdf/jetcobot.urdf.
JOINT_LIMITS_RAD = (
    (-3.14, 3.14),
    (-2.9, 2.9),
    (-2.3, 2.3),
    (-2.6, 2.6),
    (-2.5, 2.5),
    (-2.84, 2.84),
)
JOINT_LIMITS_DEG = tuple(
    tuple(math.degrees(value) for value in limits)
    for limits in JOINT_LIMITS_RAD
)
