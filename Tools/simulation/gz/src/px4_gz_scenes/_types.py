"""
Lightweight type aliases shared across the package.

All spatial quantities use the ENU (East-North-Up) convention:
  x = East, y = North, z = Up

Rotations are stored as Hamilton quaternions (w, x, y, z).
"""

from __future__ import annotations

import math

# (x, y, z) position or size in metres.
Vec3 = tuple[float, float, float]

# (w, x, y, z) unit quaternion — Hamilton convention.
Quaternion = tuple[float, float, float, float]

# (r, g, b, a) colour, each channel in [0, 1].
Color = tuple[float, float, float, float]

IDENTITY_QUAT: Quaternion = (1.0, 0.0, 0.0, 0.0)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Convert intrinsic ZYX Euler angles (radians) to a Hamilton quaternion.

    The rotation order matches the aerospace convention used by PX4 and Gazebo:
    first yaw about Z, then pitch about Y, then roll about X.

    Args:
        roll:  rotation about X axis (radians)
        pitch: rotation about Y axis (radians)
        yaw:   rotation about Z axis (radians)

    Returns:
        (w, x, y, z) unit quaternion
    """
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)
