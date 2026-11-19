"""What every actuator system shares for ros2_control: a joint's description, reading one feedback frame per Robstride
motor, and the check of a commandable range against the URDF. Pure where it can be (tested without hardware); the
URDF helpers take a Pinocchio model.
"""
from typing import List, Optional, Sequence, Tuple, NamedTuple, Any

import numpy as np

from feathersdk.robot.urdf_loader import get_pin_joint_id


class JointDescription(NamedTuple):
    """One joint's name and limits, as plain values (str and Python float), for generating ROS config. Positions and
    velocities are in the motor's frame; `urdf_direction` says how the URDF joint's value relates to them."""
    name: str
    """Motor name (`shoulder_pitch_left`): the key of every SDK dict."""
    urdf_joint: str
    """The URDF joint this motor drives (`SpL`), from the motor's config."""
    urdf_direction: int
    """+1 when the URDF joint's value equals the motor's, -1 when it is the negative (from the motor's config)."""
    position: Optional[Tuple[float, float]]
    """Commandable range (rad, or m): the overlap of the config's range and the calibrated one. None for a continuous
    joint (a drive wheel)."""
    velocity: float
    """`max_velocity` (rad/s, or m/s)."""
    acceleration: float
    """`max_acceleration` (rad/s², or m/s²)."""
    jerk: Optional[float]
    """`max_jerk` (rad/s³), None where the system has no jerk limit."""
    effort: Optional[float]
    """`max_torque` (N·m), None where the system has no torque limit in those units."""


def float32_step(values: np.ndarray) -> np.ndarray:
    """One float32 step at each value, always positive (`np.spacing()` is signed). Commands reach the Teensy as
    float32, so a value within one step of a limit is that limit on the wire."""
    return np.abs(np.spacing(values.astype(np.float32)))


FEEDBACK_READ_ATTEMPTS = 3
"""`feedback_frames()` rereads a motor whose feedback changed while it was being read: a new frame every few
milliseconds against microseconds per read, so a third change in a row means telemetry is broken."""


def feedback_frames(motors: Sequence[Any]) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Each Robstride motor's measured position, velocity and effort, all three from one feedback frame, and that
    frame's stamp (the Teensy's clock). The comms thread can store a new frame between two reads; the frame's stamp
    is stored after its values, so an unchanged stamp means every value is from that frame (a seqlock on the stamp).
    A hot path (once per ros2_control cycle): one loop, no helper calls per motor.

    Raises:
        ValueError: A motor has sent no feedback yet.
        RuntimeError: A motor's feedback changed on every attempt to read it (telemetry is broken).
    """
    n = len(motors)
    position, velocity, effort, stamp = np.empty(n), np.empty(n), np.empty(n), np.empty(n)
    for i, motor in enumerate(motors):
        telemetry, params = motor.telemetry, motor.PARAMS
        try:
            for _ in range(FEEDBACK_READ_ATTEMPTS):
                frame = telemetry[params.FEEDBACK_TS, -1]
                position[i] = telemetry[params.FB_POSITION, -1]
                velocity[i] = telemetry[params.FB_VELOCITY, -1]
                effort[i] = telemetry[params.FB_TORQUE, -1]
                if telemetry[params.FEEDBACK_TS, -1] == frame:
                    break
            else:
                raise RuntimeError(f"{motor.name}: feedback changed during each of {FEEDBACK_READ_ATTEMPTS} reads")
        except IndexError:  # an empty telemetry ring
            raise ValueError(f"{motor.name} has sent no feedback yet; initialize its system first") from None
        stamp[i] = frame
    return position, velocity, effort, stamp


URDF_RANGE_TOLERANCE = 1e-3
"""rad (or m) a commandable range may reach outside the URDF's before a system's `describe()` raises (the URDF's
rounding)."""


def urdf_range_mismatches(
    names: Sequence[str], low: np.ndarray, high: np.ndarray, urdf_low: np.ndarray, urdf_high: np.ndarray,
) -> List[str]:
    """One line per joint whose range [low, high] reaches more than `URDF_RANGE_TOLERANCE` outside the URDF's, or is
    empty; a range inside the URDF's is fine (the SDK is stricter). Written as "not within", so NaN is a mismatch."""
    tol = URDF_RANGE_TOLERANCE
    bad = ~((urdf_low - tol <= low) & (low < high) & (high <= urdf_high + tol))
    return [f"{names[i]}: [{low[i]:.4f}, {high[i]:.4f}], URDF [{urdf_low[i]:.4f}, {urdf_high[i]:.4f}]"
            for i in np.flatnonzero(bad)]


def urdf_limits(model: Any, joint_name: str) -> Optional[Tuple[float, float]]:
    """The URDF joint's position limits from a Pinocchio model, or None for a continuous joint (two configuration
    values: cos and sin, no limits). Raises ValueError if the joint isn't in the URDF or has another shape."""
    joint = model.joints[get_pin_joint_id(model, joint_name)]
    if joint.nq == 2 and joint.nv == 1:
        return None
    if joint.nq != 1:
        raise ValueError(f"URDF joint {joint_name!r} has {joint.nq} configuration values, not one")
    return float(model.lowerPositionLimit[joint.idx_q]), float(model.upperPositionLimit[joint.idx_q])


def check_against_urdf(module_name: str, joints: Sequence[JointDescription], model: Any) -> None:
    """Raise ValueError naming every joint whose commandable range, in the URDF's frame (`urdf_direction` applied),
    reaches outside the URDF's limits. Continuous URDF joints and joints without a range are not checked."""
    names, low, high, urdf_low, urdf_high = [], [], [], [], []
    for j in joints:
        limits = urdf_limits(model, j.urdf_joint)
        if j.position is None or limits is None:
            continue
        lo, hi = j.position if j.urdf_direction == 1 else (-j.position[1], -j.position[0])
        names.append(f"{j.name} ({j.urdf_joint})"); low.append(lo); high.append(hi)
        urdf_low.append(limits[0]); urdf_high.append(limits[1])
    mismatches = urdf_range_mismatches(names, np.array(low), np.array(high), np.array(urdf_low), np.array(urdf_high))
    if mismatches:
        raise ValueError(f"{module_name}: commandable range (the config's overlapping the calibrated one), in the "
                         "URDF's frame, outside the URDF's:\n  " + "\n  ".join(mismatches))
