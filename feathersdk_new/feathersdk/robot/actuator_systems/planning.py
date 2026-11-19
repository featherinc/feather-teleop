"""Planning a move, in two stages that each have their own interface:

- `PathPlanner`: **where** to go. Joint-space waypoints from the start to the goal, with no timing. Default
  `StraightLinePath`; OMPL or your own later.
- `MotionGenerator`: **how fast**. Times a path within the velocity, acceleration and jerk limits and returns a
  `Segment` (see `trajectory.py`) for the arm to play. Default `RuckigGenerator`.

`Planning` holds one of each and runs them in order. An IK stage for pose goals can join later as a third field. Pure,
no threads or hardware, so each stage can be unit tested on its own. Planning happens outside the arm's control loop;
the loop only samples the result.

`step_toward()` is the streaming counterpart: one control period toward a goal that may change every period (a
teleoperated arm in direct mode), within the velocity and acceleration limits. `OnlineFollower` does the same with
jerk limits too and a target velocity, using Ruckig's online mode: the arms' follow mode steps one each tick.
"""
import numpy as np

from feathersdk.utils.feathertypes import Any, NamedTuple, Optional, Protocol, Sequence, Tuple
from .trajectory import Segment, playing, read_only


class MotionState(NamedTuple):
    """Position, velocity and acceleration per motor (rad, rad/s, rad/s²)."""
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


class MotionLimits(NamedTuple):
    """Maximum velocity, acceleration and jerk per motor (rad/s, rad/s², rad/s³)."""
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray


class PathPlanner(Protocol):
    """Where to go: joint-space waypoints from `start` to `goal` (one position per motor each), with no timing."""

    def plan_path(self, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """Shape (points, motors), at least 2 points: the first is `start`, the last is `goal`."""
        ...


class MotionGenerator(Protocol):
    """How fast: a timed motion through `waypoints` (from a `PathPlanner`) within `limits`, leaving the `start` state at
    host time `start_time` (`currtime()`) and ending in the `goal` state."""

    def generate(
        self, start: MotionState, waypoints: np.ndarray, goal: MotionState, limits: MotionLimits, start_time: float,
    ) -> Segment:
        ...


class StraightLinePath:
    """The default path planner: straight to the goal in joint space. It doesn't consider obstacles."""

    def plan_path(self, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
        return np.vstack((start, goal))


class RuckigSegment:
    """A Ruckig trajectory as a `Segment`: sampled exactly with Ruckig's own `at_time()`, so planning builds nothing per
    tick. Never changed after it's made, so the arm thread can sample it while other threads plan."""

    __slots__ = ("start", "end", "_trajectory", "_goal", "_still")

    def __init__(self, start: float, trajectory: Any, dofs: int) -> None:
        self.start, self.end, self._trajectory = start, start + trajectory.duration, trajectory
        self._goal, self._still = read_only(trajectory.at_time(trajectory.duration)[0]), read_only(np.zeros(dofs))

    def sample(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Position, velocity, acceleration and effort (always 0) at host time `t`. A hot path (every tick)."""
        if t >= self.end:
            return self._goal, self._still, self._still, self._still
        position, velocity, acceleration = self._trajectory.at_time(max(t - self.start, 0.0))
        return np.array(position), np.array(velocity), np.array(acceleration), self._still


class SegmentChain:
    """Segments played one after another (each starting when the one before ends), as one `Segment`."""

    __slots__ = ("start", "end", "_segments")

    def __init__(self, segments: Sequence[Segment]) -> None:
        self._segments = tuple(segments)
        self.start, self.end = self._segments[0].start, self._segments[-1].end

    def sample(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return playing(self._segments, t).sample(t)


class RuckigGenerator:
    """The default motion generator: time-optimal, jerk-limited Ruckig motion, sampled exactly. Free Ruckig reaches one
    target at a time, so it stops at each intermediate waypoint (passing through them is a Ruckig Pro feature). With
    the default straight-line path there are none, and the motion equals stepping Ruckig every tick."""

    def generate(
        self, start: MotionState, waypoints: np.ndarray, goal: MotionState, limits: MotionLimits, start_time: float,
    ) -> Segment:
        """Raises ValueError if the arrays don't all have one value per motor, `start_time` isn't finite, or Ruckig
        can't plan (NaN or infinite values, a limit that isn't positive, a goal velocity above its limit)."""
        dofs = len(start.position)
        # Ruckig silently plans with a too-short list (a missing motor keeps a stale value), and never sees start_time.
        if any(len(values) != dofs for values in (*start, *goal, *limits)) or np.shape(waypoints)[1:] != (dofs,):
            raise ValueError(f"Every start, goal, limit and waypoint array needs one value per motor ({dofs})")
        if not np.isfinite(start_time):
            raise ValueError(f"start_time must be finite, got {start_time}")
        still = np.zeros(dofs)
        targets = [MotionState(waypoint, still, still) for waypoint in waypoints[1:-1]] + [goal]
        segments = []
        for target in targets:  # each leg starts where the one before ended, at rest
            segments.append(_ruckig_segment(start, target, limits, start_time, dofs))
            start, start_time = target, segments[-1].end
        return segments[0] if len(segments) == 1 else SegmentChain(segments)



def _ruckig_segment(
    start: MotionState, goal: Optional[MotionState], limits: MotionLimits, start_time: float, dofs: int,
) -> RuckigSegment:
    """One Ruckig motion from `start` to `goal`, or with `goal` None a stop: to rest, wherever the limits allow. Raises
    ValueError if Ruckig can't plan it."""
    from ruckig import ControlInterface, InputParameter, Result, Ruckig, RuckigError, Trajectory  # lazy: works
    # without ruckig too

    params = InputParameter(dofs)
    params.current_position, params.current_velocity, params.current_acceleration = map(list, start)
    if goal is None:  # the velocity interface ignores target_position and max_velocity: it only brakes
        params.control_interface = ControlInterface.Velocity
        params.target_velocity = params.target_acceleration = [0.0] * dofs
    else:
        params.target_position, params.target_velocity, params.target_acceleration = map(list, goal)
    params.max_velocity, params.max_acceleration, params.max_jerk = map(list, limits)
    trajectory = Trajectory(dofs)
    try:
        result = Ruckig(dofs).calculate(params, trajectory)
    except RuckigError as err:  # invalid input, e.g. a goal velocity above the limit
        raise ValueError(f"Ruckig could not plan this motion: {err}") from err
    if result not in (Result.Working, Result.Finished):
        raise ValueError(f"Ruckig could not plan this motion: {result}")
    return RuckigSegment(start_time, trajectory, dofs)


def brake_to_rest(start: MotionState, limits: MotionLimits, start_time: float) -> Segment:
    """Brake from `start` to rest as fast as the acceleration and jerk limits allow, and stay wherever that ends
    (Ruckig's velocity interface: the goal is zero velocity, the position is free). Raises ValueError if Ruckig can't
    plan it (e.g. NaN)."""
    return _ruckig_segment(start, None, limits, start_time, len(start.position))


def step_toward(
    position: np.ndarray, velocity: np.ndarray, goal: np.ndarray, limits: MotionLimits, dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """One step of `dt` seconds from `position`, moving at `velocity`, toward `goal`, per motor: the velocity changes by
    at most `limits.acceleration * dt`, stays within `limits.velocity`, and drops in time to stop at the goal instead of
    overshooting it. A goal that moves closer than the stopping distance is braked into harder than the acceleration
    limit, never passed. The motor arrives on `goal` exactly and at rest. Returns the new position and the step's
    velocity, which go into the next call (start from the arm's `read_state().command.position` and zero velocity).

    For direct mode, call it once per arm tick with `dt` the tick (`1 / describe().rate`) and `limits` from
    `describe().limits()`: each step then moves at most `max_velocity * dt`, which `set_command()` always accepts.
    Jerk is not limited. A NaN goal gives a NaN position, which `set_command()` refuses.

    Raises:
        ValueError: `dt` is not positive.
    """
    if not dt > 0.0:  # written so NaN fails
        raise ValueError(f"dt must be positive, got {dt}")
    error = goal - position
    dv = limits.acceleration * dt
    # The fastest speed from which braking at `acceleration` still stops at the goal, one step of reaction included.
    speed = np.minimum(limits.velocity, np.sqrt(dv * dv + 2.0 * limits.acceleration * np.abs(error)) - dv)
    exact = error / dt  # the velocity that lands on the goal this step
    limited = np.clip(np.clip(exact, velocity - dv, velocity + dv), -speed, speed)
    # Arrived: the limited step would pass the goal, or the goal is a step away and the motor can take that step and
    # stop after it (the braking speed alone only approaches the goal, and rounding can leave it ulps short for good).
    arrived = (np.abs(error) <= np.abs(limited * dt)) | ((np.abs(exact) <= dv) & (np.abs(exact - velocity) <= dv))
    return np.where(arrived, goal, position + limited * dt), np.where(arrived, exact, limited)


class OnlineFollower:
    """Steps toward a target that may change at any time, within velocity, acceleration and jerk limits (Ruckig's
    online mode). Each `step()` is one control period of `dt` seconds and starts from where the previous step ended,
    never from a measurement, so the motion stays smooth however unevenly the target changes. With no new target the
    motion ends on the last one, at rest (or moving at its target velocity).

    Not thread-safe: one thread calls `step()`; give it targets and limits from elsewhere through a `LatestWins`.
    """

    __slots__ = ("_ruckig", "_input", "_output", "_finished", "_working")

    def __init__(self, start: np.ndarray, limits: MotionLimits, dt: float, velocity: Optional[np.ndarray] = None) -> None:
        """Start at `start` moving at `velocity` (default at rest), and come to rest there until `set_target()`.

        Raises:
            ValueError: `dt` isn't positive, or the arrays don't have one value per motor.
        """
        from ruckig import InputParameter, OutputParameter, Result, Ruckig  # lazy: the SDK works without ruckig

        if not dt > 0.0:  # written so NaN fails
            raise ValueError(f"dt must be positive, got {dt}")
        dofs = len(start)
        if any(len(values) != dofs for values in limits):
            raise ValueError(f"Every limit array needs one value per motor ({dofs})")
        self._ruckig = Ruckig(dofs, dt)
        self._input = InputParameter(dofs)
        self._output = OutputParameter(dofs)
        self._working, self._finished = Result.Working, Result.Finished
        still = [0.0] * dofs
        self._input.current_position = list(start)
        self._input.current_velocity = still if velocity is None else list(velocity)
        self._input.current_acceleration = still
        self._input.target_position = list(start)
        self._input.target_velocity = still
        self._input.target_acceleration = still
        self.set_limits(limits)

    def set_target(self, position: np.ndarray, velocity: np.ndarray) -> None:
        """Move toward `position`, arriving with `velocity` (zeros: arrive at rest)."""
        self._input.target_position = position.tolist()
        self._input.target_velocity = velocity.tolist()

    def set_limits(self, limits: MotionLimits) -> None:
        self._input.max_velocity = limits.velocity.tolist()
        self._input.max_acceleration = limits.acceleration.tolist()
        self._input.max_jerk = limits.jerk.tolist()

    def step(self) -> Tuple[np.ndarray, np.ndarray]:
        """Advance one period. Returns the new position and velocity. A hot path (every tick): one Ruckig update.

        Raises:
            ValueError: Ruckig can't step (a target velocity above its limit, NaN).
        """
        output = self._output
        result = self._ruckig.update(self._input, output)
        if result != self._working and result != self._finished:
            raise ValueError(f"Ruckig could not follow the target: {result}")
        output.pass_to_input(self._input)
        return np.array(output.new_position), np.array(output.new_velocity)


class Planning(NamedTuple):
    """How the arm plans a move: where to go (`path_planner`), then how fast (`motion_generator`)."""
    path_planner: PathPlanner = StraightLinePath()
    motion_generator: MotionGenerator = RuckigGenerator()

    def plan(self, start: MotionState, goal: MotionState, limits: MotionLimits, start_time: float) -> Segment:
        """Run both stages. Raises ValueError if the path isn't at least 2 waypoints of one value per motor running
        from `start` to `goal`, or the motion generator can't time it."""
        waypoints = np.asarray(self.path_planner.plan_path(start.position, goal.position), dtype=np.float64)
        dofs = len(start.position)
        if waypoints.ndim != 2 or waypoints.shape[1] != dofs or len(waypoints) < 2:
            raise ValueError(f"A path needs at least 2 waypoints of {dofs} values, got shape {waypoints.shape}")
        if not (np.allclose(waypoints[0], start.position) and np.allclose(waypoints[-1], goal.position)):
            raise ValueError("A path must start at the start position and end at the goal position")
        return self.motion_generator.generate(start, waypoints, goal, limits, start_time)
