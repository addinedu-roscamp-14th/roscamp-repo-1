"""JetCobot joint limits shared by planning and hardware validation."""

import math


# Expanded working range without narrowing the model's previously validated
# J1/J2 limits. Keep synchronized with description and MoveIt configuration.
JOINT_LIMITS_RAD = (
    (-3.14, 3.14),
    (-2.9, 2.9),
    (-math.radians(165.0), math.radians(165.0)),
    (-math.radians(165.0), math.radians(165.0)),
    (-math.radians(165.0), math.radians(165.0)),
    (-math.radians(175.0), math.radians(175.0)),
)
JOINT_LIMITS_DEG = tuple(
    tuple(math.degrees(value) for value in limits)
    for limits in JOINT_LIMITS_RAD
)
