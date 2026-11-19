import math
import threading
import time
from traceback import format_exc
from collections import deque
from typing import Deque
from feathersdk.robot.actuators.actuator import ActuatorCalibrationError
from feathersdk.robot.actuators.damiao.damiao_motor import DamiaoFeedback, DamiaoMotor, DamiaoMotorConfig
from feathersdk.configs import PyDField, PyDFrozenBaseModel, pyd_model_validator
from feathersdk.utils import currtime, try_log_exc
from feathersdk.utils.feathertypes import TYPE_CHECKING, Any, NamedTuple, Optional, Self, Tuple
from feathersdk.logging import debug, error, info
from feathersdk.quill_defs.python_qdefs.damiao_qdef import DamiaoStatus

if TYPE_CHECKING:
    from feathersdk.robot.actuator_systems.actuator_system import ActuatorSystem, ActuatorSystemConfig


class I2RTGripperConfig(DamiaoMotorConfig):
    """Config for an I2RT linear gripper driven by a Damiao DM4310 motor.

    Positive torque closes the jaws. The jaws are always position controlled, and a force limiter (ported from I2RT's
    `GripperForceLimiter`) caps the grip force once they are blocked by an object. Defaults come from I2RT's
    `linear_4310.yml` and from bench tests on the robot's grippers.
    """
    requires_calibration: bool = PyDField(default=True)
    """The gripper finds its end stops by sweeping at every startup."""

    # Position control
    position_kp: float = PyDField(gt=0.0, le=50.0, default=20.0)
    """Position gain. The force limiter also uses it to turn a torque into a position offset."""
    position_kd: float = PyDField(ge=0.0, le=5.0, default=0.5)
    """Velocity gain."""
    end_margin: float = PyDField(ge=0.0, lt=0.5, default=0.01)
    """Fraction of the stroke kept away from each end stop, so position control never pushes into one."""
    speed: float = PyDField(gt=0.0, default=2.0)
    """How fast the jaws move, in strokes per second: the whole stroke takes `1 / speed` s. Every motion obeys it, the
       open after calibration too; the position target moves toward the goal at this speed and the jaws follow it."""

    # Force limiting
    grip_force: float = PyDField(gt=0.0, default=50.0)
    """Default grip force in Newtons at the jaws, used when a command does not give one."""
    max_grip_force: float = PyDField(gt=0.0, default=50.0)
    """Largest grip force in Newtons that any command may ask for. Raise it in the config for heavy or rigid objects,
       up to what `max_torque` allows."""
    jaw_stroke: float = PyDField(gt=0.0, default=0.096)
    """Jaw travel in meters between the open and closed end stops. Used to convert Newtons to motor torque."""
    clog_torque: float = PyDField(gt=0.0, default=0.5)
    """Average motor torque in Nm above which slow jaws count as blocked by an object (clogged)."""
    clog_velocity: float = PyDField(gt=0.0, default=0.3)
    """Motor speed in rad/s below which jaws pushing with more than `clog_torque` count as blocked."""
    unclog_torque: float = PyDField(ge=0.0, default=0.2)
    """Average motor torque in Nm below which blocked jaws count as free again."""
    friction_torque: float = PyDField(ge=0.0, default=0.3)
    """Torque in Nm added to the grip torque to make up for the lead screw's friction."""
    torque_window: float = PyDField(gt=0.0, default=0.1)
    """Seconds of torque feedback averaged to decide whether the jaws are blocked."""
    setpoint_time_constant: float = PyDField(gt=0.0, default=0.04)
    """Time constant in seconds for easing the position target to the force-limited one while blocked. I2RT smooths
       by 0.1 per step of a ~250 Hz loop, which is about 0.04 s."""

    # Calibration
    calibration_timeout: float = PyDField(gt=1.0, le=120.0, default=15.0)
    """Seconds allowed for the whole calibration: both sweeps, plus enabling and homing. The manipulation system waits
       this long for every gripper to finish."""
    calibration_torque: float = PyDField(gt=0.0, le=1.5, default=0.5)
    """Torque used to drive the jaws into each end stop while calibrating, in Nm."""
    calibration_kd: float = PyDField(ge=0.0, le=5.0, default=0.2)
    """Damping while calibrating, so the jaws don't slam into the end stops."""
    stall_velocity: float = PyDField(gt=0.0, default=0.05)
    """Below this speed in rad/s the jaws count as stopped."""
    stall_time: float = PyDField(gt=0.0, default=0.5)
    """How long in seconds the jaws must stay stopped to count as stalled at an end stop."""
    expected_stroke: float = PyDField(gt=0.0, default=5.06)
    """Expected motor travel in radians between the open and closed end stops. Measured on the robot's grippers."""
    stroke_tolerance: float = PyDField(gt=0.0, default=0.3)
    """Calibration fails if the measured stroke differs from `expected_stroke` by more than this many radians."""
    command_period: float = PyDField(gt=0.0, le=0.1, default=0.01)
    """Seconds between commands while calibrating. Must stay well under the motor's watchdog timeout."""

    @pyd_model_validator(mode='after')
    def validate_forces(self) -> Self:
        if self.grip_force > self.max_grip_force:
            raise ValueError(f"grip_force ({self.grip_force}) must not exceed max_grip_force ({self.max_grip_force})")
        if self.expected_stroke <= self.stroke_tolerance:
            raise ValueError(f"expected_stroke ({self.expected_stroke}) must exceed stroke_tolerance "
                             f"({self.stroke_tolerance})")
        # The shortest stroke calibration accepts needs the most torque for a given force.
        worst_torque = _grip_torque(self, self.max_grip_force, self.expected_stroke - self.stroke_tolerance)
        if worst_torque > self.max_torque:
            raise ValueError(f"max_grip_force ({self.max_grip_force} N) needs up to {worst_torque:.2f} Nm, more than "
                             f"max_torque ({self.max_torque} Nm)")
        return self


def _toward(current: float, goal: float, max_step: float) -> float:
    """`current` moved toward `goal` by at most `max_step`."""
    return current + max(-max_step, min(max_step, goal - current))


def _grip_torque(config: I2RTGripperConfig, force: float, stroke: float) -> float:
    """Motor torque in Nm that squeezes the jaws with `force` Newtons, given the motor `stroke` in radians.

    The lead screw is linear: force * jaw_stroke = torque * motor_stroke. Friction is added on top.
    """
    return force * config.jaw_stroke / stroke + config.friction_torque


class I2RTGripperCalibrationState(PyDFrozenBaseModel):
    """The end stops found by the last calibration sweep, in motor radians."""
    calibrated: bool = PyDField(default=True)
    """Whether the gripper is calibrated."""
    open_position: float = PyDField(default=0.0)
    """Motor position at the fully open end stop."""
    closed_position: float = PyDField(default=0.0)
    """Motor position at the fully closed end stop. Calibration ensures it is greater than `open_position`."""
    calibration_time: float = PyDField(init=False, default_factory=currtime)
    """The system time when the calibration was performed."""

    @property
    def stroke(self) -> float:
        """Motor travel in radians from the open to the closed end stop."""
        return self.closed_position - self.open_position


class _Goal(NamedTuple):
    """What the user asked the jaws to do."""
    position: float
    """Target motor position in radians, already kept `end_margin` away from the end stops."""
    grip_torque: float
    """Motor torque in Nm to squeeze with once the jaws are blocked by an object."""


class _GripperForceLimiter:
    """Caps the grip force of position-controlled jaws. Port of I2RT's `GripperForceLimiter`.

    Position control alone pushes harder the further an object keeps the jaws from their target. Once the jaws are
    blocked (high average torque, low speed) the limiter moves the position target so that kp * (target - position) is
    the grip torque. It works in motor radians, where closing is the positive direction.
    """

    def __init__(self, config: I2RTGripperConfig):
        self._config = config
        self._torques: Deque[Tuple[float, float]] = deque()
        """(time, torque) feedback samples within the last `torque_window` seconds."""
        self._clogged = False
        """Whether the jaws are blocked by an object."""
        self._adjusted_position: Optional[float] = None
        """The eased force-limited target while clogged."""
        self._last_update: Optional[float] = None
        """When `update()` last ran."""

    @property
    def clogged(self) -> bool:
        """Whether the jaws are currently blocked by an object and the force limit is active."""
        return self._clogged

    def reset(self) -> None:
        """Forget all history, e.g. after the jaws went limp or were recalibrated."""
        self._torques.clear()
        self._clogged = False
        self._adjusted_position = None
        self._last_update = None

    def update(
        self, now: float, goal: _Goal, position: float, velocity: float, torque: float, last_command: float,
    ) -> float:
        """Return the position target to command this step.

        Args:
            now: The current time in seconds.
            goal: The user's goal.
            position: The latest measured motor position.
            velocity: The latest measured motor velocity.
            torque: The latest measured motor torque.
            last_command: The position target commanded on the previous step.
        """
        cfg = self._config
        dt = cfg.command_period if self._last_update is None else max(now - self._last_update, 0.0)
        self._last_update = now

        self._torques.append((now, torque))
        while self._torques and now - self._torques[0][0] > cfg.torque_window:
            self._torques.popleft()
        average_torque = abs(sum(t for _, t in self._torques) / len(self._torques))

        if self._clogged:
            wants_to_open = goal.position < position
            if wants_to_open or average_torque < cfg.unclog_torque:
                self._clogged = False
        elif average_torque > cfg.clog_torque and abs(velocity) < cfg.clog_velocity:
            self._clogged = True

        if not self._clogged:
            self._adjusted_position = position
            return goal.position

        # The target where the motor would push with zero torque, then offset from it by the grip torque.
        direction = math.copysign(1.0, goal.position - position) if goal.position != position else 0.0
        zero_torque_position = last_command - direction * abs(torque) / cfg.position_kp
        limited = zero_torque_position + direction * goal.grip_torque / cfg.position_kp

        alpha = 1.0 - math.exp(-dt / cfg.setpoint_time_constant)
        previous = limited if self._adjusted_position is None else self._adjusted_position
        self._adjusted_position = previous + alpha * (limited - previous)
        return self._adjusted_position


_CALIBRATION_HOME_MARGIN = 0.5
"""Seconds of `calibration_timeout` kept back from the sweeps, so homing finishes before the system stops waiting."""


class I2RTGripperFault(NamedTuple):
    """A fault that stopped the gripper. Latched until the gripper is recalibrated."""
    reason: str
    """What went wrong, e.g. `LOSS_OF_COMMUNICATION (0xD)` or the error raised while commanding the gripper."""
    status: Optional[DamiaoStatus]
    """The status the motor reported instead of `ENABLED`, or None if commanding the gripper raised an error."""
    time: float
    """When the fault was detected (host time, seconds)."""
    feedback: Optional[DamiaoFeedback]
    """The feedback frame that reported the fault, or None if commanding the gripper raised an error."""


class I2RTGripper(DamiaoMotor[I2RTGripperConfig]):
    """An I2RT linear gripper with a limited grip force.

    Commands only set the gripper's goal. The owning system re-sends it every step (`_send_current_command`), which
    runs the force limiter and keeps the motor's communication watchdog from disabling it.

    Raw motor commands (`mit_command`, `save_zero`, `write_param`, `save_params`) are blocked, since they would bypass
    the force limit and the end stop margins or cut the gripper off. Use `go_to`, `open`, `close` and `release`.

    Fails fast on faults, like I2RT's driver: if the motor reports any status other than `ENABLED` while the gripper is
    commanded, or commanding it raises an error, the gripper stops commanding it, disables it, and logs the fault once.
    Commands then raise until the gripper is recalibrated (`manipulation_system.recalibrate()`), which clears the fault
    and re-enables it. A fault only stops its own gripper; the system keeps commanding the others.
    """

    calibration_state = I2RTGripperCalibrationState(calibrated=False)
    """The end stops found by the last calibration."""

    def __init__(self, config: I2RTGripperConfig, system: 'ActuatorSystem[ActuatorSystemConfig, I2RTGripper]'):
        super().__init__(config, system)
        self._lock = threading.RLock()
        """Guards the goal and limiter, which both the user's thread and the system thread use."""
        self._sending = False
        """Whether the system thread should send commands. False until the first command after calibration."""
        self._goal: Optional[_Goal] = None
        """The user's goal, or None for limp jaws."""
        self._last_command_position: Optional[float] = None
        """The position target sent on the previous step, or None if the jaws were limp."""
        self._last_step_time: Optional[float] = None
        """When the previous step sent a target, or None if the jaws were limp: sizes the next step of the ramp."""
        self._limiter = _GripperForceLimiter(config)
        """Caps the grip force once the jaws are blocked."""
        self._calibrating = False
        """While True, the calibration loop owns the motor and the system thread must not send commands."""
        self._fault: Optional[I2RTGripperFault] = None
        """The latched motor fault, or None. Cleared by calibration."""
        self._disabled = False
        """True after `disable()`, until `enable()` or calibration. Commands raise meanwhile."""

    def mit_command(
        self, position: float = 0.0, velocity: float = 0.0, kp: float = 0.0, kd: float = 0.0, torque: float = 0.0,
        clamp: bool = False,
    ) -> None:
        """Blocked: raw commands would bypass the force limit. Use `go_to`, `open`, `close` or `release`."""
        raise RuntimeError(f"[{self.config.name}] Raw MIT commands are disabled on the gripper; use go_to(), open(), "
                           f"close() or release()")

    def save_zero(self, block: bool = False) -> None:
        """Blocked: moving the zero would invalidate the calibrated end stops."""
        raise RuntimeError(f"[{self.config.name}] save_zero() is disabled on the gripper")

    def write_param(self, param: Any, value: Any, block: bool = True) -> None:
        """Blocked: changing the registers of a running gripper would cut it off. Use `feathercli set-motor-id`."""
        raise RuntimeError(f"[{self.config.name}] write_param() is disabled on the gripper; "
                           f"use feathercli set-motor-id")

    def save_params(self) -> None:
        """Blocked: see `write_param`."""
        raise RuntimeError(f"[{self.config.name}] save_params() is disabled on the gripper; "
                           f"use feathercli set-motor-id")

    def disable(self, block: bool = False, **kwargs: Any) -> None:
        """Stop commanding the gripper, then disable the motor. Call `enable()` before commanding it again."""
        with self._lock:
            self._disabled = True
            self._sending = False
            self._goal = None
            self._last_command_position = None
            self._last_step_time = None
            self._limiter.reset()
        super().disable(block=block, **kwargs)

    def enable(self, block: bool = False, **kwargs: Any) -> None:
        """Enable the motor so the gripper can be commanded again. Always waits for the motor's reply, so the next
        command sees it enabled. The jaws stay limp until the next command."""
        super().enable(block=True, **kwargs)
        with self._lock:
            self._disabled = False

    @property
    def fault(self) -> Optional[I2RTGripperFault]:
        """The motor fault that stopped the gripper, or None if it is healthy."""
        return self._fault

    def _calibrate(self, load_calibration: Optional[bool] = None, only_load: bool = False, home: bool = True) -> None:
        """Find both end stops by driving into each with `calibration_torque`, then open the gripper if `home`.

        Args:
            load_calibration: Unused. The gripper always sweeps; there is no saved calibration to load.
            only_load: Must be False, since there is nothing to load.
            home: If True, open the gripper after calibrating.
        """
        if only_load:
            raise ValueError(f"[{self.config.name}] The gripper calibrates by sweeping; there is no state to load")

        deadline = currtime() + self.config.calibration_timeout - _CALIBRATION_HOME_MARGIN
        with self._lock:
            self._fault = None  # calibration clears the motor's error and re-enables it
            self._calibrating = True
            self._sending = False
            self._goal = None
            self._last_command_position = None
            self._last_step_time = None
            self._limiter.reset()
        try:
            self.clear_error(block=True)
            self.enable(block=True)
            open_position = self._drive_to_stall(-self.config.calibration_torque, deadline)
            closed_position = self._drive_to_stall(self.config.calibration_torque, deadline)
            self._mit_command()

            stroke = closed_position - open_position
            if abs(stroke - self.config.expected_stroke) > self.config.stroke_tolerance:
                raise ActuatorCalibrationError(
                    f"[{self.config.name}] Measured stroke {stroke:.3f} rad, expected {self.config.expected_stroke:.3f}"
                    f" +/- {self.config.stroke_tolerance:.3f}. Is something blocking the jaws?"
                )

            self.calibration_state = I2RTGripperCalibrationState(
                open_position=open_position, closed_position=closed_position,
            )
            info(f"[{self.config.name}] Calibrated: open {open_position:.3f} rad, closed {closed_position:.3f} rad")
        except BaseException:
            with self._lock:  # anything commanded meanwhile must not run on a gripper that is no longer calibrated
                self.calibration_state = I2RTGripperCalibrationState(calibrated=False)
                self._sending = False
                self._goal = None
            self._mit_command()
            raise
        finally:
            self._calibrating = False

        if home:
            self.open()

    def _drive_to_stall(self, torque: float, deadline: float) -> float:
        """Apply `torque` until the jaws stop moving, and return the furthest position they reached that way.

        The furthest position, rather than where they stopped, ignores any bounce back off the end stop.

        Args:
            torque: The torque to drive with in Nm. Its sign picks the end stop.
            deadline: When the calibration's time budget runs out (host time).
        """
        low_velocity_since: Optional[float] = None
        furthest: Optional[float] = None
        while currtime() < deadline:
            self._mit_command(kd=self.config.calibration_kd, torque=torque)
            time.sleep(self.config.command_period)

            feedback = self.read_feedback()
            if feedback.status != DamiaoStatus.ENABLED:
                raise ActuatorCalibrationError(f"[{self.config.name}] Motor left enabled state: {feedback.status.name}")

            position = feedback.position
            furthest = position if furthest is None else (max if torque > 0 else min)(furthest, position)

            if abs(feedback.velocity) >= self.config.stall_velocity:
                low_velocity_since = None
            elif low_velocity_since is None:
                low_velocity_since = currtime()
            elif currtime() - low_velocity_since >= self.config.stall_time:
                return furthest

        raise ActuatorCalibrationError(
            f"[{self.config.name}] Jaws did not reach an end stop with {torque:+.2f} Nm within the "
            f"{self.config.calibration_timeout} s calibration budget"
        )

    def _send_current_command(self) -> None:
        """Run the force limiter and send the resulting command. Called every step by the owning system."""
        with self._lock:
            if not self._sending or self._calibrating or self._fault is not None:
                return
            try:
                self._send_locked()
            except Exception as exc:  # fail fast on this gripper only; never let it stop the system thread
                self._fail(f"{type(exc).__name__}: {exc}", error_text=format_exc())

    def _send_locked(self) -> None:
        """One step of `_send_current_command`. Call with the lock held."""
        feedback = self.read_feedback()  # status, position, velocity and torque from one frame
        if feedback.status != DamiaoStatus.ENABLED:
            self._fail(f"{feedback.status.name} (0x{int(feedback.status):X})", feedback=feedback)
            return

        goal = self._goal
        if goal is None:
            self._mit_command()
            return

        position = feedback.position
        last_command = position if self._last_command_position is None else self._last_command_position
        now = currtime()
        # The target moves toward the goal at `speed`, never jumps to it; the force limiter works on the moving target.
        dt = self.config.command_period if self._last_step_time is None else max(now - self._last_step_time, 0.0)
        self._last_step_time = now
        step = self.config.speed * self.calibration_state.stroke * dt
        target = self._limiter.update(
            now, _Goal(_toward(last_command, goal.position, step), goal.grip_torque), position, feedback.velocity,
            feedback.torque, last_command,
        )
        # Never target past the end stop margins. Calibration ensures open < closed.
        target = min(max(target, self._position_for(1.0)), self._position_for(0.0))
        self._mit_command(target, kp=self.config.position_kp, kd=self.config.position_kd)
        self._last_command_position = target

    def _fail(
        self, reason: str, feedback: Optional[DamiaoFeedback] = None, error_text: Optional[str] = None,
    ) -> None:
        """Latch a fault: stop commanding, disable the motor, and log it once. Call with the lock held.

        Args:
            reason: What went wrong, for the fault record and the log.
            feedback: The feedback frame that reported a non-`ENABLED` status, if that is the cause.
            error_text: The traceback, if commanding the gripper raised an error.
        """
        goal, blocked = self._goal, self._limiter.clogged
        status = None if feedback is None else feedback.status
        self._fault = I2RTGripperFault(reason=reason, status=status, time=currtime(), feedback=feedback)
        self._sending = False
        self._goal = None
        self._last_command_position = None
        self._last_step_time = None
        self._limiter.reset()

        if feedback is not None:
            details = (f"Position {feedback.position:+.3f} rad, velocity {feedback.velocity:+.2f} rad/s, torque "
                       f"{feedback.torque:+.2f} Nm, temperature MOS {feedback.temperature_mos} C / rotor "
                       f"{feedback.temperature_rotor} C, frame {(self._fault.time - feedback.timestamp) * 1000:.0f} ms "
                       f"old.")
        else:
            details = f"Error while commanding it:\n{error_text}"
        if goal is None:
            intent = "Goal: none (limp)."
        else:
            intent = (f"Goal {goal.position:+.3f} rad with grip torque {goal.grip_torque:.2f} Nm, "
                      f"blocked by an object: {blocked}.")
        error(f"[{self.config.name}] Fault {reason}: stopped commanding and disabled the gripper. {intent} {details} "
              f"Recalibrate to recover.")
        with try_log_exc(f"[{self.config.name}] Failed to disable after a fault"):
            DamiaoMotor.disable(self)

    def _raise_if_unavailable(self) -> None:
        """Raise if the gripper can't take commands: faulted, disabled, or calibrating. Call with the lock held."""
        fault = self._fault
        if fault is not None:
            raise RuntimeError(
                f"[{self.config.name}] Gripper stopped on fault {fault.reason} {currtime() - fault.time:.1f} s ago and "
                f"is disabled. Fix the cause, then recalibrate to recover."
            )
        if self._disabled:
            raise RuntimeError(f"[{self.config.name}] Gripper is disabled; call enable() before commanding it")
        if self._calibrating:
            raise RuntimeError(f"[{self.config.name}] Gripper is calibrating; wait for it to finish")

    def _require_calibrated(self) -> I2RTGripperCalibrationState:
        if not self.calibration_state.calibrated:
            raise RuntimeError(f"[{self.config.name}] Gripper is not calibrated")
        return self.calibration_state

    def _position_for(self, opening: float) -> float:
        """Motor position for an opening between 0.0 (closed) and 1.0 (open), kept `end_margin` from the end stops."""
        state = self._require_calibrated()
        margin = self.config.end_margin * state.stroke
        closed = state.closed_position - margin
        opened = state.open_position + margin
        return closed + opening * (opened - closed)

    def go_to(self, opening: float, force: Optional[float] = None) -> None:
        """Move the jaws to an opening and hold there, squeezing with at most `force` if an object blocks them.

        Args:
            opening: Between 0.0 (closed) and 1.0 (fully open). The jaws stop short of each end stop by `end_margin`.
            force: Grip force limit in Newtons. Defaults to the config's `grip_force`; must not exceed
                `max_grip_force`.
        """
        if not 0.0 <= opening <= 1.0:
            raise ValueError(f"[{self.config.name}] opening must be between 0.0 and 1.0, got {opening}")
        force = self.config.grip_force if force is None else force
        if not 0.0 < force <= self.config.max_grip_force:
            raise ValueError(f"[{self.config.name}] force must be in (0, {self.config.max_grip_force}] N, got {force}")
        state = self._require_calibrated()

        with self._lock:
            self._raise_if_unavailable()
            self._goal = _Goal(self._position_for(opening), _grip_torque(self.config, force, state.stroke))
            self._sending = True
            self._send_current_command()
            self._raise_if_unavailable()

    def open(self) -> None:
        """Fully open the jaws and hold them open."""
        self.go_to(1.0)

    def close(self, force: Optional[float] = None) -> None:
        """Close the jaws and keep squeezing a grasped object with at most `force` Newtons until told otherwise.

        Args:
            force: Grip force limit in Newtons. Defaults to the config's `grip_force`; must not exceed
                `max_grip_force`.
        """
        self.go_to(0.0, force)

    def release(self) -> None:
        """Stop applying any force. The jaws go limp but the motor stays enabled.

        Does nothing if the gripper is faulted, disabled or calibrating, so shutdown code can always call it.
        """
        with self._lock:
            try:
                self._raise_if_unavailable()
            except RuntimeError as exc:
                debug(f"release() ignored: {exc}")
                return
            self._goal = None
            self._last_command_position = None
            self._last_step_time = None
            self._limiter.reset()
            self._sending = True
            self._send_current_command()

    def is_blocked(self) -> bool:
        """Whether an object is blocking the jaws, so the force limit is active. True while holding a grasped object."""
        return self._limiter.clogged

    def get_opening(self) -> float:
        """The current opening between 0.0 (closed) and 1.0 (fully open), from the latest feedback."""
        state = self._require_calibrated()
        opening = (state.closed_position - self.get_position()) / state.stroke
        return min(max(opening, 0.0), 1.0)
