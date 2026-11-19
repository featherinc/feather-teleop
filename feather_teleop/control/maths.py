# Small maths helpers used every tick, faster than numpy's equivalents on 3-vectors.
import math

import numpy as np

TINY_ANGLE = 1e-9       # rad, below this a rotation is treated as infinitesimal
NEAR_HALF_TURN = 1e-6   # rad, this close to pi the axis is read off the diagonal


def norm(v: np.ndarray) -> float:
    # Return the length of a 1-D vector.
    return math.sqrt(v @ v)


def clamp(x: float, low: float, high: float) -> float:
    # Return `x` limited to [low, high]. NaN stays NaN.
    return low if x < low else high if x > high else x


def smoothing_factor(cutoff_hz: float, dt: float) -> float:
    # Blend factor of a first-order low-pass: `state += factor * (input - state)` every `dt` seconds.
    return 1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)


def skew(v) -> np.ndarray:
    # Return the matrix that takes the cross product with `v`.
    x, y, z = v.tolist() if isinstance(v, np.ndarray) else v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def to_vector(R: np.ndarray) -> np.ndarray:
    # Rotation matrix to rotation vector (axis times angle, rad).
    (xx, xy, xz), (yx, yy, yz), (zx, zy, zz) = R.tolist()
    angle = math.acos(clamp((xx + yy + zz - 1.0) / 2.0, -1.0, 1.0))
    x, y, z = zy - yz, xz - zx, yx - xy
    if angle < TINY_ANGLE:
        return np.array([x / 2.0, y / 2.0, z / 2.0])
    if math.pi - angle < NEAR_HALF_TURN:
        # Near half a turn the off-diagonal differences vanish.
        axis = np.sqrt(np.clip((np.diag(R) + 1.0) / 2.0, 0.0, None))
        k = int(np.argmax(axis))
        signs = np.sign([R[k, 0] + R[0, k], R[k, 1] + R[1, k], R[k, 2] + R[2, k]])
        signs[k] = 1.0
        return axis * signs * angle
    scale = angle / (2.0 * math.sin(angle))
    return np.array([x * scale, y * scale, z * scale])


def to_matrix(vector) -> np.ndarray:
    # Rotation vector to rotation matrix (Rodrigues' formula).
    vector = np.asarray(vector, float)
    angle = norm(vector)
    if angle < TINY_ANGLE:
        return np.eye(3) + skew(vector)
    K = skew(vector / angle)
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * K @ K

