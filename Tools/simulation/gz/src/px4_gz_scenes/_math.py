"""
Private quaternion / rotation utilities shared by exporters.

These helpers are not part of the public API — import from ``px4_gz_scenes``
directly for public symbols.

All quaternions follow the Hamilton convention ``(w, x, y, z)`` used
throughout the package.  Rotation matrices are ``(3, 3)`` float64 NumPy
arrays that act on **column vectors**: ``p_rotated = R @ p``.  For
batch operations use :func:`apply_rotation`, which accepts any leading
batch dimensions ``(..., 3)``.
"""

from __future__ import annotations

import numpy as np

from px4_gz_scenes._types import Quaternion


def quat_to_rotation_matrix(q: Quaternion) -> np.ndarray:
    """
    Build a ``(3, 3)`` rotation matrix from a unit Hamilton quaternion.

    Args:
        q: ``(w, x, y, z)`` unit quaternion.

    Returns:
        Orthonormal rotation matrix ``R`` such that ``R @ v`` rotates the
        column vector ``v`` by ``q``.
    """
    w, x, y, z = q
    x2, y2, z2 = x * x, y * y, z * z
    return np.array(
        [
            [1 - 2 * (y2 + z2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x2 + z2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x2 + y2)],
        ],
        dtype=np.float64,
    )


def apply_rotation(R: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """
    Apply rotation matrix ``R`` to a batch of points.

    Args:
        R:   ``(3, 3)`` rotation matrix.
        pts: Array of shape ``(..., 3)``.

    Returns:
        Array of the same shape as ``pts`` with each point rotated by ``R``.
        Computed as ``pts @ R.T`` (equivalent to ``R @ p`` per column).
    """
    return pts @ R.T
