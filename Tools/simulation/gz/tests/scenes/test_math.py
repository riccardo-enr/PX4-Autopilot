import math
import numpy as np
from px4_gz_scenes._math import apply_rotation, quat_to_rotation_matrix
from px4_gz_scenes._types import euler_to_quat, IDENTITY_QUAT


def test_identity_quat_gives_identity_matrix():
    R = quat_to_rotation_matrix(IDENTITY_QUAT)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_90_deg_yaw():
    # 90° yaw: x → y, y → -x, z → z
    q = euler_to_quat(0.0, 0.0, math.pi / 2)
    R = quat_to_rotation_matrix(q)
    expected = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    np.testing.assert_allclose(R, expected, atol=1e-12)


def test_90_deg_pitch():
    # 90° pitch (about Y): x → -z, z → x
    q = euler_to_quat(0.0, math.pi / 2, 0.0)
    R = quat_to_rotation_matrix(q)
    expected = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )
    np.testing.assert_allclose(R, expected, atol=1e-12)


def test_apply_rotation_identity():
    R = np.eye(3)
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    out = apply_rotation(R, pts)
    np.testing.assert_array_equal(out, pts)


def test_apply_rotation_batch():
    # 90° yaw rotates (1,0,0) → (0,1,0)
    q = euler_to_quat(0.0, 0.0, math.pi / 2)
    R = quat_to_rotation_matrix(q)
    pts = np.array([[1.0, 0.0, 0.0]])
    out = apply_rotation(R, pts)
    np.testing.assert_allclose(out, [[0.0, 1.0, 0.0]], atol=1e-12)
