"""Timed arm trajectories: making, checking, sampling and splicing them.

Pure functions with no threads or hardware, so each part can be unit tested on its own. The arm's trajectory mode
(`Robstride7MArm.set_trajectory()`) is built on them.
"""
import numpy as np

from feathersdk.utils.feathertypes import NamedTuple, Optional, Protocol, Sequence, Tuple


def read_only(values: Sequence) -> np.ndarray:
    """A read-only float64 copy, so a value shared between threads can't change after it's handed over."""
    array = np.array(values, dtype=np.float64)
    array.flags.writeable = False
    return array


class Segment(Protocol):
    """Anything the arm can play: a start, an end, and the state at any time. `ArmTrajectory` (timed points) and a
    planner's own result (e.g. `planner.RuckigSegment`) both meet it, so a planner keeps its exact representation."""

    start: float
    """Host time (`currtime()`, seconds) the segment starts at."""

    @property
    def end(self) -> float:
        """Host time it ends at."""
        ...

    def sample(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Position, velocity, acceleration and effort per motor at host time `t`: the start state before the start,
        and the end state, still, after the end. Must not change the segment (the arm thread calls it every tick)."""
        ...


class ArmTrajectory(NamedTuple):
    """Timed points for every motor of an arm (motors in `motor_order`). Build with `make()`."""
    start: float
    """Host time (`currtime()`, seconds) of the first point."""
    times: np.ndarray
    """Seconds after `start` of each point, shape (points,), strictly increasing from 0."""
    positions: np.ndarray
    """Shape (points, motors), rad."""
    velocities: Optional[np.ndarray]
    """Shape (points, motors), rad/s. Without them, points are joined by straight lines, as ros2_control's trajectory
    controller does with positions only."""
    efforts: np.ndarray
    """Feed-forward torque on top of gravity compensation, shape (points, motors), N·m, linearly interpolated."""
    accelerations: Optional[np.ndarray] = None
    """Shape (points, motors), rad/s², with `velocities` only. With them, points are joined by curves that match the
    position, velocity and acceleration at both ends (quintic), so a plan started mid-motion joins without a jerk."""

    @classmethod
    def make(
        cls, start: float, times: Sequence[float], positions: Sequence, velocities: Optional[Sequence],
        efforts: Sequence, accelerations: Optional[Sequence] = None,
    ) -> 'ArmTrajectory':
        """A trajectory holding read-only copies. Raises ValueError if the shapes don't match, or accelerations are
        given without velocities."""
        t, q, e = read_only(times), read_only(positions), read_only(efforts)
        qd = None if velocities is None else read_only(velocities)
        qdd = None if accelerations is None else read_only(accelerations)
        shapes_match = q.ndim == 2 and len(q) == len(t) == t.size and e.shape == q.shape
        velocities_match = qd is None or qd.shape == q.shape
        accelerations_match = qdd is None or (qd is not None and qdd.shape == q.shape)
        if not (shapes_match and velocities_match and accelerations_match):
            raise ValueError(f"Trajectory shapes don't match: times {t.shape}, positions {q.shape}, efforts {e.shape}, "
                f"velocities {None if qd is None else qd.shape}, accelerations {None if qdd is None else qdd.shape}")
        return cls(float(start), t, q, qd, e, qdd)

    @property
    def end(self) -> float:
        """Host time of the last point."""
        return self.start + float(self.times[-1])

    def sample(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Position, velocity, acceleration and effort per motor at host time `t`. Before the start it gives the first
        point; after the end, the last point, still."""
        times, q, effort = self.times, self.positions, self.efforts
        s = max(t - self.start, 0.0)
        if s >= times[-1]:
            still = np.zeros(q.shape[1])
            return q[-1], still, still, effort[-1]
        k = int(np.searchsorted(times, s, side="right"))  # times[k - 1] <= s < times[k]
        h, u = times[k] - times[k - 1], (s - times[k - 1]) / (times[k] - times[k - 1])
        p0, p1, e = q[k - 1], q[k], effort[k - 1] + u * (effort[k] - effort[k - 1])
        if self.velocities is None:
            return p0 + u * (p1 - p0), (p1 - p0) / h, np.zeros(q.shape[1]), e
        # Hermite curves: float weights times the points' values, in one matrix product. A hot path (every tick).
        u, h = float(u), float(h)
        u2, u3 = u * u, u * u * u
        qd = self.velocities
        if self.accelerations is None:
            # Cubic: matches the positions and velocities at both points. Rows: position, velocity, acceleration.
            weights = np.array([
                [2 * u3 - 3 * u2 + 1, (u3 - 2 * u2 + u) * h, 3 * u2 - 2 * u3, (u3 - u2) * h],
                [(6 * u2 - 6 * u) / h, 3 * u2 - 4 * u + 1, (6 * u - 6 * u2) / h, 3 * u2 - 2 * u],
                [(12 * u - 6) / (h * h), (6 * u - 4) / h, (6 - 12 * u) / (h * h), (6 * u - 2) / h],
            ])
            position, velocity, acceleration = weights @ np.stack((p0, qd[k - 1], p1, qd[k]))
            return position, velocity, acceleration, e
        # Quintic: also matches the accelerations at both points.
        u4, u5 = u3 * u, u3 * u2
        qdd, hh = self.accelerations, h * h
        weights = np.array([
            [1 - 10 * u3 + 15 * u4 - 6 * u5, (u - 6 * u3 + 8 * u4 - 3 * u5) * h, 0.5 * (u2 - 3 * u3 + 3 * u4 - u5) * hh,
             10 * u3 - 15 * u4 + 6 * u5, (-4 * u3 + 7 * u4 - 3 * u5) * h, 0.5 * (u3 - 2 * u4 + u5) * hh],
            [(-30 * u2 + 60 * u3 - 30 * u4) / h, 1 - 18 * u2 + 32 * u3 - 15 * u4,
             0.5 * (2 * u - 9 * u2 + 12 * u3 - 5 * u4) * h,
             (30 * u2 - 60 * u3 + 30 * u4) / h, -12 * u2 + 28 * u3 - 15 * u4, 0.5 * (3 * u2 - 8 * u3 + 5 * u4) * h],
            [(-60 * u + 180 * u2 - 120 * u3) / hh, (-36 * u + 96 * u2 - 60 * u3) / h,
             0.5 * (2 - 18 * u + 36 * u2 - 20 * u3),
             (60 * u - 180 * u2 + 120 * u3) / hh, (-24 * u + 84 * u2 - 60 * u3) / h, 0.5 * (6 * u - 24 * u2 + 20 * u3)],
        ])
        position, velocity, acceleration = weights @ np.stack((p0, qd[k - 1], qdd[k - 1], p1, qd[k], qdd[k]))
        return position, velocity, acceleration, e


def check(
    trajectory: ArmTrajectory, low: np.ndarray, high: np.ndarray, max_velocity: np.ndarray, max_effort: np.ndarray,
) -> None:
    """Raise ValueError unless the arm can follow `trajectory`: finite times strictly increasing from 0, every position
    within [`low`, `high`], every velocity and the average speed between points within `max_velocity`, and every effort
    within `max_effort` (per motor). Each check is "all within limits", so NaN and inf fail it."""
    t, q, qd, e = trajectory.times, trajectory.positions, trajectory.velocities, trajectory.efforts
    if not (t[0] == 0.0 and np.isfinite(t[-1]) and (np.diff(t) > 0.0).all()):
        raise ValueError(f"Times must be finite, start at 0 and strictly increase: {t}")
    if not ((low <= q) & (q <= high)).all():
        raise ValueError("Positions outside angle_range (or NaN)")
    if qd is not None and not (np.abs(qd) <= max_velocity).all():
        raise ValueError("Velocities above max_velocity (or NaN)")
    if not (np.abs(e) <= max_effort).all():
        raise ValueError("Efforts above max_torque (or NaN)")
    if len(t) > 1 and not (np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None] <= max_velocity).all():
        raise ValueError("Points too far apart for their times: moving between them needs more than max_velocity")


def playing(segments: Tuple[Segment, ...], t: float) -> Segment:
    """The segment playing at host time `t`: the latest one that has started, or the first if none has. Runs every tick,
    so it builds no list."""
    for segment in reversed(segments):
        if segment.start <= t:
            return segment
    return segments[0]


def splice(segments: Tuple[Segment, ...], new: Segment, now: float) -> Tuple[Segment, ...]:
    """The segments after adding `new`, which replaces everything from its start on. Segments that can't play again
    after `now` are dropped."""
    kept = [s for s in segments if s.start < new.start]
    started = [i for i, s in enumerate(kept) if s.start <= now]
    return tuple(kept[started[-1] if started else 0:]) + (new,)
