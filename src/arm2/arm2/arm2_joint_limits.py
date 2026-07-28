"""JetCobot joint limits shared by planning and hardware validation."""

import math


# Keep these values synchronized with jetcobot_description/urdf/jetcobot.urdf.
JOINT_LIMITS_RAD = (
    (math.radians(-168.0), math.radians(168.0)),
    (math.radians(-140.0), math.radians(140.0)),
    (math.radians(-150.0), math.radians(150.0)),
    (math.radians(-150.0), math.radians(150.0)),
    (math.radians(-155.0), math.radians(160.0)),
    (math.radians(-150.0), math.radians(150.0)),
)
JOINT_LIMITS_DEG = tuple(
    tuple(math.degrees(value) for value in limits)
    for limits in JOINT_LIMITS_RAD
)
