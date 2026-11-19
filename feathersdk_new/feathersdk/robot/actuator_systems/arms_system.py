import time

import numpy as np

from .actuator_system import ActuatorSystem, ActuatorSystemConfig
from .collision import BodyCheck, CollisionError, Contact
from .joints import JointDescription, check_against_urdf, feedback_frames, float32_step
from .planning import MotionLimits, MotionState, OnlineFollower, Planning, brake_to_rest
from .trajectory import ArmTrajectory, Segment, check as check_trajectory, playing, read_only, splice
from feathersdk.configs import PyDField, PyDBaseModel, PyDConfigDict, TEST_DEFAULT
from feathersdk.robot.actuators import RobstrideMotor, RobstrideMotorConfig, calibrate_actuator_group
from feathersdk.utils.feathertypes import (
    Optional, Literal, Union, TYPE_CHECKING, Tuple, List, Dict, ensure, Any, ModuleType, NamedTuple, Sequence, Callable,
)
from feathersdk.logging import notice, info, debug, flush_logs, error, warning, log_event
from feathersdk.robot.actuators.robstride import RobRunMode
from feathersdk.quill_defs.python_qdefs.robstride_qdef import rob_kd_lim, rob_kp_lim
from feathersdk.robot.urdf_loader import get_pin_joint_id, _PinocchioModel
from feathersdk.utils import (
    deg2rad, format_exc, exponential_timeout, UniqueIntEnum, FeatherThreadLock, UniqueEnum, OwnedSlot, NotOwnerError,
    LatestWins, currtime, try_log_exc,
)
from feathersdk.utils.exceptions import UnreachableError


if TYPE_CHECKING:
    import pinocchio as pin
    from feathersdk.robot.robot import FeatherRobot


def get_arm_pin_config_indices(model: _PinocchioModel, joint_names: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    """TODO: make better docstring explaining what's going on here. """
    # NOTE: q (configuration) and the generalized-force/velocity vectors live in
    # DIFFERENT index spaces. The continuous wheel joints (BpL/BpR/BpC) take 2
    # configuration values (cos, sin) but only 1 velocity DOF, so idx_q != idx_v.
    # Use q_indices to write into q, and v_indices to read tau (e.g. gravity).
    q_indices: List[int] = []
    v_indices: List[int] = []
    for name in joint_names:
        j = model.joints[get_pin_joint_id(model, name)]
        q_indices.extend(range(j.idx_q, j.idx_q + j.nq))
        v_indices.extend(range(j.idx_v, j.idx_v + j.nv))
    return np.array(q_indices), np.array(v_indices)


class Robstride7MArmConfig(ActuatorSystemConfig):
    """Configuration for a single Robstride arm."""
    requires_urdf: bool = PyDField(default=True)
    """Whether this system requires the robot URDF to be loaded."""
    operating_frequency: float = PyDField(default=50.0)
    """The operating frequency of the arm system."""
    module_name: str = PyDField(default="arm")
    """The name of the module."""
    side: Literal["left", "right"] = PyDField(json_schema_extra={TEST_DEFAULT: "left"})
    """The side of the robot that the arm is on."""
    hand_joint_name: str = PyDField(json_schema_extra={TEST_DEFAULT: "WrL"})
    """The URDF name of the joint that the hand is attached to. Payloads set with `set_payload()` are added here."""
    shoulder_pitch_motor: RobstrideMotorConfig
    """Configuration for the shoulder pitch motor."""
    shoulder_roll_motor: RobstrideMotorConfig
    """Configuration for the shoulder roll motor."""
    shoulder_yaw_motor: RobstrideMotorConfig
    """Configuration for the shoulder yaw motor."""
    elbow_motor: RobstrideMotorConfig
    """Configuration for the elbow motor."""
    wrist_yaw_motor: RobstrideMotorConfig
    """Configuration for the wrist yaw motor."""
    wrist_pitch_motor: RobstrideMotorConfig
    """Configuration for the wrist pitch motor."""
    wrist_roll_motor: RobstrideMotorConfig
    """Configuration for the wrist roll motor."""

    require_approval_for_calibration: bool = PyDField(default=True)
    """Whether to require approval for calibration by prompting the user for permission. Should always be True for 
       production systems for safety!"""
    on_abort_action: Literal["disable", "freeze"] = PyDField(default="disable")
    """What to do when the arm is aborted. "disable" to home/disable motors, "freeze" to freeze the arm in place."""
    apply_gravity_compensation: bool = PyDField(default=True)
    """Whether to apply gravity compensation to the arm. If True, will add gravity torque to the motor when applying 
       torque."""
    command_timeout: float = PyDField(default=0.2, gt=0.0)
    """Seconds the newest command may age before the arm takes a safety hold (see `Robstride7MArm.hold()`). Commands
       made by the arm's own control laws are always fresh; this guards commands written from other threads."""


class RobstrideArmMotorTargets(PyDBaseModel):
    """Targets for a single Robstride arm motor."""
    model_config = PyDConfigDict(frozen=False, arbitrary_types_allowed=True)

    motor: RobstrideMotor
    """The motor to set targets for."""
    kp: float = 0.0
    """The proportional gain to use."""
    kd: float = 0.0
    """The derivative gain to use."""
    position: float = 0.0
    """The target position angle to reach."""
    additional_torque: float = 0.0
    """The additional torque to apply."""

    target_velocity: float = 0.0
    """The target velocity to have when reaching the target position."""
    target_acceleration: float = 0.0
    """The target acceleration to have when reaching the target position."""
    max_velocity: float = 0.0
    """The maximum velocity to reach."""
    max_acceleration: float = 0.0
    """The maximum acceleration to reach."""
    max_jerk: float = 0.0
    """The maximum jerk to reach."""

    curr_position: float = 0.0
    """The current position of the motor."""
    curr_velocity: float = 0.0
    """The current velocity of the motor."""
    curr_acceleration: float = 0.0
    """The current acceleration of the motor."""


class _ManualTargetIndices(UniqueIntEnum):
    """Indices for the motion data arrays."""
    Kp = 0
    Kd = 1
    AdditionalTorque = 2


_HOLD_SETTLE_TIMEOUT = 2.0
_GO_TO_ACCELERATION = 5.0
"""1/s: `go_to()`'s acceleration limit per unit of its speed limit (home's 5 rad/s^2 at 1 rad/s)."""
_GO_TO_TOLERANCE = 0.05
"""rad: every joint this close to its goal counts as arrived."""
_GO_TO_SLACK, _GO_TO_SETTLE = 2.0, 5.0
"""`go_to()`'s default timeout: this many times the time at full speed, plus this many seconds to settle."""
_GO_TO_POLL = 0.05
"""Seconds between `go_to()`'s checks."""
"""Seconds `Robstride7MArm.hold(block=True)` waits for the arm to stop."""


class ArmCommand(NamedTuple):
    """One tick's command for every motor of an arm, in `motor_order`. Gravity compensation is not included:
    `_send_command` adds it, whoever wrote the command. Build with `make()`."""
    position: np.ndarray
    """Target angle per motor (rad)."""
    velocity: np.ndarray
    """Target velocity per motor (rad/s)."""
    effort: np.ndarray
    """Feed-forward torque per motor (N·m), on top of gravity compensation."""
    kp: np.ndarray
    """Position gain per motor."""
    kd: np.ndarray
    """Velocity gain per motor."""
    stamp: float
    """When the command was made (`currtime()`, seconds)."""

    @classmethod
    def make(
        cls, position: Sequence[float], velocity: Sequence[float], effort: Sequence[float], kp: Sequence[float],
        kd: Sequence[float],
    ) -> 'ArmCommand':
        """A command stamped now, holding read-only copies of the values (so it can't change after it's written to the
        command slot). Raises ValueError unless all five are 1D with the same length."""
        arrays = [read_only(v) for v in (position, velocity, effort, kp, kd)]
        if arrays[0].ndim != 1 or any(a.shape != arrays[0].shape for a in arrays):
            raise ValueError(f"ArmCommand fields must be equal-length 1D arrays, got {[a.shape for a in arrays]}")
        return cls(*arrays, stamp=currtime())


class ArmClipping(NamedTuple):
    """Per motor, whether the Teensy clamped a value of the last command sent (it clamps silently): the value was
    outside the motor's limit, or NaN (then the Teensy sent nothing)."""
    position: np.ndarray
    """Outside the range the Teensy clamps to (`RobstrideMotor.command_angle_range()`)."""
    velocity: np.ndarray
    """Above `max_velocity`."""
    torque: np.ndarray
    """Effort plus gravity compensation above `max_torque`."""
    kp: np.ndarray
    """Outside 0 to the motor version's kp limit."""
    kd: np.ndarray
    """Outside 0 to the motor version's kd limit."""


class ArmState(NamedTuple):
    """One arm at one moment: what each motor measured and what was last sent to it, per motor in `names` order.
    Returned by `Robstride7MArm.read_state()`."""
    names: Tuple[str, ...]
    """Motor names, in `motor_order`."""
    position: np.ndarray
    """Measured angle (rad)."""
    velocity: np.ndarray
    """Measured velocity (rad/s)."""
    effort: np.ndarray
    """Measured torque (N·m)."""
    feedback_stamp: np.ndarray
    """When each motor's newest feedback frame arrived at the Teensy (s, the Teensy's clock, kept in sync with the
    host's). Position, velocity and effort are all from that frame. A stamp that stops changing means stale feedback."""
    command: Optional[ArmCommand]
    """The command last sent to the motors (without gravity compensation), or None before the first."""
    torque: Optional[np.ndarray]
    """The torque last sent: the command's effort plus that tick's gravity compensation (N·m)."""
    clipped: Optional[ArmClipping]
    """What the Teensy clamped in the last command sent."""


class ArmDescription(NamedTuple):
    """What a ROS wrapper needs to know about one arm, as plain values. Returned by `Robstride7MArm.describe()`."""
    name: str
    """The arm's `module_name` (`left_arm`)."""
    joints: Tuple[JointDescription, ...]
    """In motor order: the order of `read_state()`'s arrays."""
    control_modes: Tuple[str, ...]
    """Every value `set_control_mode()` takes."""
    rate: float
    """The arm loop's rate (Hz): each command is sent on the next tick."""
    command_timeout: float
    """Seconds the newest command may age before the arm takes a safety hold: stream faster than this."""

    def limits(self) -> MotionLimits:
        """The joints' velocity, acceleration and jerk limits as arrays in motor order, for `planning.step_toward()`."""
        return MotionLimits(*(np.array([getattr(j, field) for j in self.joints]) for field in
                              ("velocity", "acceleration", "jerk")))


class ArmsControlMode(UniqueEnum):
    """The control mode for the arm."""
    Targets = "targets"
    """Arms will move to specified target positions."""
    Impedance = "impedance"
    """Arms will attempt to hold their current position unless moved around by external (non-gravity) forces."""
    Hold = "hold"
    """Safety hold: the arm stops smoothly where it is and stays there until `set_control_mode()` releases it. Entered
    through `Robstride7MArm.hold()`."""
    Direct = "direct"
    """The motors follow setpoints from `Robstride7MArm.set_command()` on the next tick, with no motion planning. Send
    them steadily: if the newest one gets older than `command_timeout`, the arm takes a safety hold."""
    Trajectory = "trajectory"
    """The motors follow timed points from `Robstride7MArm.set_trajectory()`, sampled at each tick's time."""
    Follow = "follow"
    """The motors move toward the newest target from `Robstride7MArm.set_follow_target()`: each tick the arm thread
    takes one step toward it within the follow limits (Ruckig's online mode), so targets can come at any rate."""


_ArmsControlModeStr = Literal["targets", "impedance", "hold", "direct", "trajectory", "follow"]


class Robstride7MArm(ActuatorSystem[Robstride7MArmConfig, RobstrideMotor]):
    """A single Robstride arm with 7 motor design."""

    def __init__(self, config: Robstride7MArmConfig, robot: 'FeatherRobot'):
        super().__init__(config, robot)

        self._enabled = False

        # Create the motors
        self.shoulder_pitch_motor = RobstrideMotor(self.config.shoulder_pitch_motor, self)
        self.shoulder_roll_motor = RobstrideMotor(self.config.shoulder_roll_motor, self)
        self.shoulder_yaw_motor = RobstrideMotor(self.config.shoulder_yaw_motor, self)
        self.elbow_motor = RobstrideMotor(self.config.elbow_motor, self)
        self.wrist_yaw_motor = RobstrideMotor(self.config.wrist_yaw_motor, self)
        self.wrist_pitch_motor = RobstrideMotor(self.config.wrist_pitch_motor, self)
        self.wrist_roll_motor = RobstrideMotor(self.config.wrist_roll_motor, self)

        self.motor_order = [self.shoulder_pitch_motor, self.shoulder_roll_motor, self.shoulder_yaw_motor, 
            self.elbow_motor, self.wrist_yaw_motor, self.wrist_pitch_motor, self.wrist_roll_motor]
        self.motor_gravity_mults = np.array([m.config.gravity_multiplier for m in self.motor_order])
        self._cache_motor_values()

        # The command slot: the owner (the control mode) writes each tick's command, and the newest one is sent. Only
        # `set_control_mode()` and the safety takeover `hold()` change the owner.
        self._command: OwnedSlot[ArmsControlMode, ArmCommand] = OwnedSlot(ArmsControlMode.Targets)
        # The command last sent and the torque it was sent with (effort plus gravity), for read_state(). Not the
        # slot's newest command: a tick can skip sending it (watchdog, owner switch). Only the arm thread writes it.
        self._sent: Optional[Tuple[ArmCommand, np.ndarray]] = None
        self._hold_reason: Optional[str] = None
        # The plan the targets, hold and trajectory owners play, labelled with the owner it was made for. The arm
        # thread only plays a plan made for the current owner.
        self._trajectory: LatestWins[Tuple[ArmsControlMode, Tuple[Segment, ...]]] = LatestWins()

        # Follow mode: the newest target and limits, written from any thread, and the online trajectory generator the
        # arm thread steps toward them each tick (made when follow mode starts). The versions are the ones it applied.
        self._follow_target: LatestWins[Tuple[np.ndarray, np.ndarray]] = LatestWins()
        self._follow_limits: LatestWins[MotionLimits] = LatestWins()
        self._follow_limits.write(MotionLimits(
            read_only([m.config.max_velocity for m in self.motor_order]),
            read_only([m.config.max_acceleration for m in self.motor_order]),
            read_only([m.config.max_jerk for m in self.motor_order])))
        self._follower: Optional[OnlineFollower] = None
        self._follow_target_version = 0
        self._follow_limits_version = 0

        # Plans are made outside the arm thread. `set_planning()` swaps how set_targets() and home() plan; stops
        # (freeze(), the safety hold) brake with `brake_to_rest()`, which never depends on a custom planning.
        self._planning = Planning()

        # The goal and limits set_targets() plans toward. Sticky: each call changes only what it names.
        self._ruckig_target_positions = np.zeros(len(self.motor_order))
        self._ruckig_max_velocity = np.array([motor.config.max_velocity for motor in self.motor_order])
        self._ruckig_max_acceleration = np.array([motor.config.max_acceleration for motor in self.motor_order])
        self._ruckig_max_jerk = np.array([motor.config.max_jerk for motor in self.motor_order])
        self._ruckig_target_velocities = np.zeros(len(self.motor_order))
        self._ruckig_target_accelerations = np.zeros(len(self.motor_order))

        self._ruckig_lock = FeatherThreadLock("arms_ruckig_lock")

        # Plans are timed by the arm's clock: wall time minus the time the arm thread was stalled, so a stall
        # delays the motion instead of making the next tick jump along the plan. Only the arm thread adds to it.
        self._clock_offset = 0.0
        self._last_tick: Optional[float] = None

        # Stores the non-ruckig targets for each motor.
        self._manual_targets = np.zeros((len(self.motor_order), len(_ManualTargetIndices)))

        # URDF model is set in _initialize()
        self._gravity = np.zeros(len(self.motor_order))
        self.pin_model: Optional[_PinocchioModel] = None
        self.pin_data: Any = None
        self.pin_arm_q_inds: Any = None
        self.pin_arm_v_inds: Any = None
        self.pin_import: Optional[ModuleType] = None

        # The collision check set_command() and set_trajectory() run while on (set_collision_check()), and where the
        # torso puts the column. Swapped whole, so a command call sees one or the other.
        self._collision: Optional[Tuple[BodyCheck, Callable[[], float]]] = None

        # Payload (held object) data. Set in _initialize() and set_payload()
        self._hand_joint_id: Optional[int] = None
        self._hand_base_inertia: Any = None
        self._payload_mass = 0.0

        # Impedance control data
        self._motor_impedance_positions = np.zeros(len(self.motor_order))
        self._motor_impedance_filt_torques = np.zeros(len(self.motor_order))
        self._impedance_kp = np.array([m.config.impedance_kp for m in self.motor_order])
        self._impedance_kd = np.array([m.config.impedance_kd for m in self.motor_order])
    
    def _segment_command(
        self, owner: ArmsControlMode, kp: np.ndarray, kd: np.ndarray, extra_effort: np.ndarray,
    ) -> Optional[ArmCommand]:
        """The owner's plan sampled at this tick's time on the arm's clock, or None if the stored plan was made for
        another owner (just after a switch, before the new owner has planned): then nothing is sent that tick. A hot
        path."""
        stored = self._trajectory.read()
        if stored is None or stored[0] != owner:
            return None
        now = currtime() - self._clock_offset
        position, velocity, _, effort = playing(stored[1], now).sample(now)
        return ArmCommand.make(position, velocity, effort + extra_effort, kp, kd)

    def _follow_command(self) -> ArmCommand:
        """Follow mode: one step toward the newest target, within the newest limits. A hot path (every tick): the
        target and limits are handed to the generator only when they changed."""
        follower = self._follower
        target_version, target = self._follow_target.read_versioned()
        if target_version != self._follow_target_version:
            follower.set_target(*target)
            self._follow_target_version = target_version
        limits_version, limits = self._follow_limits.read_versioned()
        if limits_version != self._follow_limits_version:
            follower.set_limits(limits)
            self._follow_limits_version = limits_version
        position, velocity = follower.step()
        return ArmCommand.make(position, velocity, self._zeros, self._default_kp, self._default_kd)

    def _impedance_command(self) -> ArmCommand:
        """Impedance mode: move each motor's hold position along the external (non-gravity) torque, and return the
        command that holds it there."""
        current_positions = self.capture_position()

        for i, motor in enumerate(self.motor_order):
            curr_pos = current_positions[motor.name]
            imp_pos = self._motor_impedance_positions[i]
            filt_torque = self._motor_impedance_filt_torques[i]
            filt_torque_alpha = motor.config.impedance_position_filt_torque_alpha

            torque_diff = self._gravity[i] - motor.telemetry[motor.PARAMS.FB_TORQUE, -1]
            filt_torque = filt_torque * filt_torque_alpha + torque_diff * (1 - filt_torque_alpha)

            if abs(imp_pos - curr_pos) > motor.config.impedance_position_delta_update:
                imp_pos = curr_pos

            new_torque = np.sign(filt_torque) * max(0.0, abs(filt_torque) - motor.config.impedance_position_min_torque)
            imp_pos += (new_torque / motor.config.impedance_position_d_filter) * (1.0 / self.config.operating_frequency)
            max_offset = motor.config.impedance_position_max_offset
            imp_pos = float(np.clip(imp_pos, curr_pos - max_offset, curr_pos + max_offset))

            self._motor_impedance_filt_torques[i] = filt_torque
            self._motor_impedance_positions[i] = imp_pos

        return ArmCommand.make(
            position=self._motor_impedance_positions,
            velocity=np.zeros(len(self.motor_order)),
            effort=self._manual_targets[:, _ManualTargetIndices.AdditionalTorque],
            kp=self._impedance_kp,
            kd=self._impedance_kd,
        )

    def _send_command(self, command: ArmCommand) -> None:
        """Send `command` to every motor, adding this tick's gravity compensation to its feed-forward torque, and keep
        what was sent for `read_state()`."""
        torque = command.effort + self._gravity  # one array add, not one per motor; kept as what was sent
        for i, motor in enumerate(self.motor_order):
            motor.operation_command(
                torque=torque[i],
                angle=command.position[i],
                velocity=command.velocity[i],
                kp=command.kp[i],
                kd=command.kd[i],
                clamp=True,
            )
        # After the sends: a send that raised leaves the previous one reported. One tuple store, atomic for readers.
        # Not made read-only here (a cost on the tick); read_state() hands out a read-only copy.
        self._sent = (command, torque)

    def _thread_step(self) -> None:
        """Step function for the arm handler thread."""
        # A tick a period or more late means the thread stalled: pause the arm's clock by all but one period, so the
        # plan continues one tick on (less late is normal jitter, and the plan stays on wall time).
        now = currtime()
        if self._last_tick is not None and now - self._last_tick >= 2.0 * self._tick:
            self._clock_offset += now - self._last_tick - self._tick
        self._last_tick = now
        # Still poll each motor for its telemetry in case users want to record while arm is disabled.
        for motor in self.actuators_by_name.values():
            if motor.initialized:
                motor.trigger_feedback_frame(update_hz=self.config.operating_frequency, block=False)
        
        if not self._enabled:
            return
        if self.pin_import is None:
            raise UnreachableError("Pinocchio model not initialized")
        
        # Check if any of the motors are having feedback delay, and log warning if so
        for motor in self.actuators_by_name.values():
            if motor.initialized:
                motor.check_warn_on_feedback_delay(motor.PARAMS.FB_POSITION)
        
        # Compute the new gravity torques for each motor, if using.
        if self.config.apply_gravity_compensation:
            q = self.pin_import.neutral(self.pin_model)
            joint_pos = self.capture_position()
            q[self.pin_arm_q_inds] = [joint_pos[m.config.name] for m in self.motor_order]
            grav = self.pin_import.computeGeneralizedGravity(self.pin_model, self.pin_data, q)[self.pin_arm_v_inds]
            self._gravity = grav * self.motor_gravity_mults
        
        # The slot's owner computes this tick's command and writes it, then the newest command is sent with gravity.
        owner = self._command.owner
        command: Optional[ArmCommand] = None
        manual = self._manual_targets
        if owner in (ArmsControlMode.Targets, ArmsControlMode.Hold):  # a hold is a planned stop at the held position
            command = self._segment_command(owner, manual[:, _ManualTargetIndices.Kp],
                                            manual[:, _ManualTargetIndices.Kd],
                                            manual[:, _ManualTargetIndices.AdditionalTorque])
        elif owner == ArmsControlMode.Impedance:
            command = self._impedance_command()
        elif owner == ArmsControlMode.Trajectory:
            command = self._segment_command(owner, self._default_kp, self._default_kd, self._zeros)
        elif owner == ArmsControlMode.Follow:
            command = self._follow_command()
        elif owner != ArmsControlMode.Direct:  # direct setpoints are written by set_command(), from the caller's thread
            raise ValueError(f"Invalid control mode: {owner}")
        if command is not None:
            try:
                self._command.write(owner, command)
            except NotOwnerError:
                return  # the owner switched during this tick; motors keep their last command until the new owner writes
        elif owner != ArmsControlMode.Direct:
            return  # no plan for this owner yet (it was just switched to): motors keep their last command
        latest = self._command.read()
        if latest is None:
            raise UnreachableError("Command slot is empty after the owner wrote to it")
        # Watchdog: a command older than `command_timeout` means its writer stopped. Hold instead of sending it.
        age = currtime() - latest.stamp
        if age > self.config.command_timeout and self.control_mode != ArmsControlMode.Hold:
            self.hold(f"watchdog: newest command is {age:.3f} s old (limit {self.config.command_timeout} s)")
            return
        self._send_command(latest)
    
    def set_targets(
        self,
        positions: Optional[Dict[str, float]] = None,
        max_velocities: Optional[Dict[str, float]] = None,
        max_accelerations: Optional[Dict[str, float]] = None,
        max_jerks: Optional[Dict[str, float]] = None,
        kps: Optional[Dict[str, float]] = None,
        kds: Optional[Dict[str, float]] = None,
        target_velocities: Optional[Dict[str, float]] = None,
        target_accelerations: Optional[Dict[str, float]] = None,
        additional_torques: Optional[Dict[str, float]] = None,
    ) -> None:
        """Sets the target values for one or more motors in the arm.

        Motion is controlled using Robstride's Operation Command mode. Each arm motor will apply torque according to 
        the following formula:

        `torque = kp * (target_position - current_position) + kd * (target_velocity - current_velocity) + offset`

        Where:

        - `target_position` is the target position angle to reach.
        - `current_position` is the last-received position angle of the motor.
        - `target_velocity` is the target angular velocity to reach. (This is computed automatically. See below.)
        - `current_velocity` is the last-received angular velocity of the motor.
        - `kp` is the proportional gain, applied to the position error.
        - `kd` is the derivative gain, applied to the velocity error.
        - `offset` is extra torque applied as needed. It will consist of:
          * Gravity compensation if `config.apply_gravity_compensation` is True.
          * Plus any additional torque to apply in the `offset` parameter to this function call.
        
        Motor telemetry should update at around `config.operating_frequency` Hz.

        The target velocity is computed automatically by the arm's planning (a straight line timed by Ruckig unless
        `set_planning()` changed it).

        NOTE: Values will be clamped to the motor's limits before sending commands to the motor. EG: using a
        `max_velocity` higher than the motor's `config.max_velocity` will be clamped to the motor's
        `config.max_velocity`.

        NOTE: All parameters are optional. If None, then none of the motors' parameters for that value will be changed.
        Otherwise it should be a dictionary with keys being either 1) the string name of the motor or 2) the
        RobstrideMotor object itself. Values are the new target values for that parameter. Missing motor keys will
        not have their parameters changed. Parameter values that are `None` will not be changed.

        NOTE: If the arm is not in target control mode, then there may be restrictions on which parameters can be set.
        If in Impedance control mode, then only `kps`, `kds`, and `additional_torques` can be set.
        
        Args:
            positions: New target position angles. See Note above for format details.
            max_velocities: New target maximum velocities. See Note above for format details.
            max_accelerations: New target maximum accelerations. See Note above for format details.
            max_jerks: New target maximum jerks. See Note above for format details.
            kps: New proportional gains. See Note above for format details.
            kds: New derivative gains. See Note above for format details.
            target_velocities: New target velocities. These are the velocities we want the motors to have when reaching
                the target position. Set to 0 to have motor stop at target position. See Note above for format details.
            target_accelerations: New target accelerations. These are the accelerations we want the motors to have when 
                reaching the target position. Set to 0 not have instantaneous acceleration change upon reaching target. 
                See Note above for format details.
            additional_torques: New additional torques to apply. See Note above for format details.
        """        
        self._check_thread()
        with self._ruckig_lock:
            if self.control_mode == ArmsControlMode.Targets:
                goal_and_limits = (self._ruckig_target_positions, self._ruckig_max_velocity,
                                   self._ruckig_max_acceleration, self._ruckig_max_jerk, self._ruckig_target_velocities,
                                   self._ruckig_target_accelerations, self._manual_targets)
                saved = [array.copy() for array in goal_and_limits]
                self._set_targets_helper(
                    positions=positions, 
                    max_velocities=max_velocities, 
                    max_accelerations=max_accelerations,
                    max_jerks=max_jerks,
                    kps=kps,
                    kds=kds,
                    target_velocities=target_velocities,
                    target_accelerations=target_accelerations,
                    additional_torques=additional_torques,
                )
                # Only a new goal or limit needs a new plan; gains and extra torque apply on the next tick.
                replanned = bool(positions or max_velocities or max_accelerations or max_jerks or target_velocities
                                 or target_accelerations)
                if replanned:
                    try:
                        self._replan(ArmsControlMode.Targets, self._planning, from_measured=False)
                    except ValueError:  # unplannable: keep the previous goal, limits and plan
                        for array, old in zip(goal_and_limits, saved):
                            array[...] = old
                        raise
                log_event("targets", self.config.module_name, replanned=replanned, positions=positions,
                          max_velocities=max_velocities, max_accelerations=max_accelerations, max_jerks=max_jerks,
                          kps=kps, kds=kds, target_velocities=target_velocities,
                          target_accelerations=target_accelerations, additional_torques=additional_torques)
            elif self.control_mode == ArmsControlMode.Impedance:
                if positions is not None or max_velocities is not None or max_accelerations is not None or \
                    max_jerks is not None or target_velocities is not None or target_accelerations is not None:
                    raise ValueError("Can only set `kps`, `kds`, and `additional_torques` in impedance control mode.")
                self._set_targets_helper(kps=kps, kds=kds, additional_torques=additional_torques)
            elif self.control_mode == ArmsControlMode.Hold:
                raise NotOwnerError(f"{self.config.module_name} is in a safety hold ({self._hold_reason}). Call "
                    "set_control_mode() to release it before setting targets.")
            elif self.control_mode in (ArmsControlMode.Direct, ArmsControlMode.Trajectory):
                raise NotOwnerError(f"{self.config.module_name} is in {self.control_mode.value} mode: call "
                    "set_control_mode('targets') first.")
            else:
                raise ValueError(f"Invalid control mode: {self.control_mode}")
    
    def _set_targets_helper(
        self,
        positions: Optional[Dict[str, float]] = None,
        max_velocities: Optional[Dict[str, float]] = None,
        max_accelerations: Optional[Dict[str, float]] = None,
        max_jerks: Optional[Dict[str, float]] = None,
        kps: Optional[Dict[str, float]] = None,
        kds: Optional[Dict[str, float]] = None,
        target_velocities: Optional[Dict[str, float]] = None,
        target_accelerations: Optional[Dict[str, float]] = None,
        additional_torques: Optional[Dict[str, float]] = None,
    ) -> None:
        for i, motor in enumerate(self.motor_order):
            if positions is not None and motor.name in positions:
                self._ruckig_target_positions[i] = positions[motor.name]
            if max_velocities is not None and motor.name in max_velocities:
                self._ruckig_max_velocity[i] = max_velocities[motor.name]
            if max_accelerations is not None and motor.name in max_accelerations:
                self._ruckig_max_acceleration[i] = max_accelerations[motor.name]
            if max_jerks is not None and motor.name in max_jerks:
                self._ruckig_max_jerk[i] = max_jerks[motor.name]
            if kps is not None and motor.name in kps:
                if self.control_mode == ArmsControlMode.Impedance:
                    self._impedance_kp[i] = kps[motor.name]
                else:
                    self._manual_targets[i, _ManualTargetIndices.Kp] = kps[motor.name]
            if kds is not None and motor.name in kds:
                if self.control_mode == ArmsControlMode.Impedance:
                    self._impedance_kd[i] = kds[motor.name]
                else:
                    self._manual_targets[i, _ManualTargetIndices.Kd] = kds[motor.name]
            if target_velocities is not None and motor.name in target_velocities:
                self._ruckig_target_velocities[i] = target_velocities[motor.name]
            if target_accelerations is not None and motor.name in target_accelerations:
                self._ruckig_target_accelerations[i] = target_accelerations[motor.name]
            if additional_torques is not None and motor.name in additional_torques:
                self._manual_targets[i, _ManualTargetIndices.AdditionalTorque] = additional_torques[motor.name]
    
    def _get_current_motor_values(self) -> Tuple[List[float], List[float], List[float]]:
        """Returns the current motor values for the arm (position, velocity, acceleration)."""
        motor_telems = [m.telemetry for m in self.motor_order]
        positions = [t[t.PARAMS.FB_POSITION, -1] for t in motor_telems]
        velocities = [t[t.PARAMS.FB_VELOCITY, -1] for t in motor_telems]
        accelerations = [0.0 for _ in motor_telems]
        
        return positions, velocities, accelerations
    
    # One builder for both "stay here" plans (the hold's fallback, trajectory mode's start), so they can't drift apart.
    def _still_at(self, position: Sequence[float]) -> ArmTrajectory:
        """A plan that stays at `position` (one per motor) from now on, with no feed-forward torque."""
        return ArmTrajectory.make(currtime() - self._clock_offset, [0.0], [position], None, [self._zeros])

    def _replan(self, owner: ArmsControlMode, planning: Planning, from_measured: bool) -> None:
        """Plan with `planning` from now to the stored goal within the stored limits, and play it as `owner`'s plan.
        Starts from where the current plan has the arm now (so a new goal mid-motion joins without a jump in position,
        velocity or acceleration), or from the measured state if `from_measured` or `owner` has no plan yet. Raises
        ValueError if `planning` can't plan or returns something the arm can't play; nothing changes then. Call with
        `_ruckig_lock` held."""
        now = currtime() - self._clock_offset  # plans are on the arm's clock
        stored = self._trajectory.read()
        segments = stored[1] if stored is not None and stored[0] == owner else ()
        if from_measured or not segments:
            position, velocity, _ = self._get_current_motor_values()
            start = MotionState(np.array(position), np.array(velocity), self._zeros)
        else:
            start = MotionState(*playing(segments, now).sample(now)[:3])
        goal = MotionState(self._ruckig_target_positions, self._ruckig_target_velocities,
                           self._ruckig_target_accelerations)
        limits = MotionLimits(self._ruckig_max_velocity, self._ruckig_max_acceleration, self._ruckig_max_jerk)
        segment = planning.plan(start, goal, limits, now)
        # Sample it once here, so a plan the arm can't play fails in the caller's thread, not the arm's.
        sampled = segment.sample(segment.start)
        if len(sampled) != 4 or any(np.shape(values) != (len(self.motor_order),) for values in sampled):
            raise ValueError(f"{planning!r} returned a plan the arm can't play: sample() must give 4 arrays of one "
                             f"value per motor ({len(self.motor_order)})")
        self._trajectory.write((owner, splice(segments, segment, now)))
    
    def capture_position(self, trigger_feedback: bool = False) -> Dict[str, float]:
        """Returns a dictionary mapping motor name to last-received angular position.
        
        Args:
            trigger_feedback: Whether to trigger a feedback frame for the motors. You should only need to set this to
                True if you want to get telemetry from an arm that is currently disabled. This will send a request to
                the motor to respond with current telemetry.
        """
        if trigger_feedback:
            for motor in self.actuators_by_name.values():
                motor.trigger_feedback_frame(block=False)
        # Python floats: the telemetry stores float32, and a caller's arithmetic on float32 stays float32.
        return {m.name: float(m.telemetry[m.PARAMS.FB_POSITION, -1]) for m in self.actuators_by_name.values()}

    def describe(self) -> ArmDescription:
        """The arm's joints (motor names, with their URDF joints), their limits and the control modes, as plain
        values: what a ROS wrapper generates its config from, so it restates no value. Each joint's commandable range
        is checked against the URDF loaded for gravity compensation; the URDF's velocity and effort limits aren't
        (Feather's URDFs have placeholders there), so take those from here.

        Raises:
            ValueError: No URDF is loaded, or a joint's commandable range reaches outside the URDF's (each such joint
                is listed with both ranges). A range inside the URDF's is fine.
        """
        if self.pin_model is None:
            raise ValueError(f"{self.config.module_name}: describe() checks the joints against the URDF, but none is "
                             "loaded (initialize the robot with a urdf_path)")
        motors = self.motor_order
        commandable = np.array([m.command_angle_range() for m in motors])  # calibrated, else configured
        low = np.maximum(self._angle_low, commandable[:, 0])
        high = np.minimum(self._angle_high, commandable[:, 1])
        # Python floats throughout: ROS parameters typed double reject ints and numpy scalars.
        joints = tuple(JointDescription(
            m.name, m.config.urdf_joint_name, m.config.urdf_direction, (float(lo), float(hi)),
            float(m.config.max_velocity), float(m.config.max_acceleration), float(m.config.max_jerk),
            float(m.config.max_torque),
        ) for m, lo, hi in zip(motors, low, high))
        check_against_urdf(self.config.module_name, joints, self.pin_model)
        return ArmDescription(self.config.module_name, joints, tuple(mode.value for mode in ArmsControlMode),
                              float(self.config.operating_frequency), float(self.config.command_timeout))

    def read_state(self) -> ArmState:
        """A snapshot of the arm: each motor's measured position, velocity and effort, all from one feedback frame, with
        that frame's time; the command last sent, the torque it was sent with (gravity compensation included), and what
        the Teensy clamped in it. Called once per control cycle by a ros2_control hardware interface.

        Raises:
            ValueError: A motor has sent no feedback yet (the arm isn't initialized).
            RuntimeError: A motor's feedback changed on every attempt to read it (telemetry is broken).
        """
        position, velocity, effort, stamp = feedback_frames(self.motor_order)  # one frame per motor (a seqlock)
        sent = self._sent  # read once: the arm thread replaces it each tick
        if sent is None:
            command = torque = clipped = None
        else:
            command, torque = sent[0], read_only(sent[1])  # a copy: a caller can't change what the next read reports
            # Clipping is worked out here, from what was sent, not in the tick: a read costs the tick nothing. Written
            # as "not within limits", so NaN counts as clipped. The angle range is read each time (recalibrating
            # moves it).
            limits = np.array([m.command_angle_range() for m in self.motor_order])
            step = float32_step(limits)  # the Teensy clamps in float32 too
            angle_low, angle_high = (limits - step)[:, 0], (limits + step)[:, 1]
            q, qd, kp, kd = command.position, command.velocity, command.kp, command.kd
            clipped = ArmClipping(
                position=~((angle_low <= q) & (q <= angle_high)),
                velocity=~(np.abs(qd) <= self._max_velocity),
                torque=~(np.abs(torque) <= self._max_effort),
                kp=~((0.0 <= kp) & (kp <= self._max_kp)),
                kd=~((0.0 <= kd) & (kd <= self._max_kd)),
            )
        return ArmState(tuple(self._motor_names), position, velocity, effort, stamp, command, torque, clipped)

    def enable(self) -> None:
        """Enable the arm."""
        if self._enabled:
            return
        
        for motor in self.actuators_by_name.values():
            motor.set_run_mode(RobRunMode.OPERATION, block=True, check=True)
            motor.enable(block=True)
        
        self._enabled = True
        self.freeze()
    
    def disable(self) -> None:
        """Disable the arm. Moves to home position first."""
        if not self._enabled:
            return
        if self._thread_dead():  # the home is a plan the thread plays; the motors keep holding where they are
            error(f"{self.config.module_name}: its thread died, so the arm can't home before disabling: the motors "
                  "keep holding their last command until power-off or a restart")
            return

        self.home(block=True)

        self._enabled = False
        for motor in self.actuators_by_name.values():
            motor.disable(block=True)
    
    def home(self, clean_parameters: bool = False, block: bool = True) -> None:
        """Home the arm.
        
        Will switch to target control mode before homing.

        Args:
            clean_parameters: If True, will overwrite all movement parameters with defaults. This way, we will still
                home nicely even if bad parameters are set.
            block: Whether to block the current thread until the home is complete.
        """
        self.set_control_mode(ArmsControlMode.Targets)

        if clean_parameters:
            self.clean_parameters()
        # One plan, from where the arm actually is (calling set_targets() here would plan twice: from the old plan, then
        # again from the measured state).
        with self._ruckig_lock:
            self._set_targets_helper(
                positions={m.name: 0.0 for m in self.actuators_by_name.values()},
                max_velocities={m.name: 1.0 for m in self.actuators_by_name.values()},
                max_accelerations={m.name: 5.0 for m in self.actuators_by_name.values()},
            )
            self._replan(ArmsControlMode.Targets, self._planning, from_measured=True)
        log_event("home", self.config.module_name, clean_parameters=clean_parameters)

        if block:
            def block_func() -> bool:
                all_homed = True
                for motor in self.actuators_by_name.values():
                    if abs(motor.telemetry[motor.PARAMS.FB_POSITION, -1]) > 0.05:
                        all_homed = False
                        break
                return all_homed
            
            exponential_timeout(30.0, await_true=block_func, start_delay=0.01, max_delay=0.1)
    
    def go_to(self, positions: Dict[str, float], max_velocity: float, block: bool = True,
              tolerance: float = _GO_TO_TOLERANCE, timeout: Optional[float] = None) -> None:
        """Targets mode: move to `positions` ({motor name: rad}; motors left out keep their target), no joint faster than
        `max_velocity` rad/s. Not collision checked.

        Args:
            block: Wait until every joint named is within `tolerance` (rad) of its goal.
            timeout: Seconds to wait; by default twice the time the furthest joint needs at `max_velocity`, plus 5 s.

        Raises:
            TimeoutError: (with `block`) The arm did not get there in time. It is frozen where it is, as it is on any
                exception while waiting (Ctrl+C included).
        """
        self.set_control_mode(ArmsControlMode.Targets)
        self.set_targets(positions=positions, max_velocities=dict.fromkeys(positions, max_velocity),
                         max_accelerations=dict.fromkeys(positions, _GO_TO_ACCELERATION * max_velocity))
        if block:
            _wait_until_at([(self, positions)], max_velocity, tolerance, timeout)

    def _distance_to(self, positions: Dict[str, float]) -> float:
        """The largest distance (rad) of a joint named in `positions` from its goal, as measured."""
        measured = self.capture_position()
        return max(abs(goal - measured[name]) for name, goal in positions.items())

    def clean_parameters(self) -> None:
        """Clean the parameters of the arm. 
        
        This will overwrite all movement parameters with defaults except for the position.
        """
        self.set_targets(**self._default_parameters())

    def _cache_motor_values(self) -> None:
        """Per-motor values from the (frozen) config in `motor_order`, computed once at startup for the hot paths
        (`set_command()`, the tick). Read-only, so nothing can change them by accident."""
        motors = self.motor_order
        self._motor_names = [m.name for m in motors]
        self._motor_name_set = frozenset(self._motor_names)
        self._zeros = read_only(np.zeros(len(motors)))
        self._default_kp = read_only([m.config.default_op_kp for m in motors])
        self._default_kd = read_only([m.config.default_op_kd for m in motors])
        self._angle_low = read_only([m.config.angle_range[0] for m in motors])
        self._angle_high = read_only([m.config.angle_range[1] for m in motors])
        # What set_command() checks positions against: the range widened by one float32 step, since that is how
        # positions reach the motor (a position computed in float32 was refused for 1.6e-8 rad).
        self._angle_low_check = read_only(self._angle_low - float32_step(self._angle_low))
        self._angle_high_check = read_only(self._angle_high + float32_step(self._angle_high))
        self._max_velocity = read_only([m.config.max_velocity for m in motors])
        self._max_effort = read_only([m.config.max_torque for m in motors])
        self._max_kp = read_only([rob_kp_lim[m.config.version] for m in motors])
        self._max_kd = read_only([rob_kd_lim[m.config.version] for m in motors])
        self._tick = 1.0 / self.config.operating_frequency

    def _default_parameters(self) -> Dict[str, Dict[str, float]]:
        """Every movement parameter except position at its configured default, as `set_targets()` keyword arguments."""
        motors = self.actuators_by_name.values()
        return dict(
            kps=dict(zip(self._motor_names, self._default_kp)),
            kds=dict(zip(self._motor_names, self._default_kd)),
            max_velocities={m.name: m.config.max_velocity for m in motors},
            max_accelerations={m.name: m.config.max_acceleration for m in motors},
            max_jerks={m.name: m.config.max_jerk for m in motors},
            target_velocities={m.name: 0.0 for m in motors},
            target_accelerations={m.name: 0.0 for m in motors},
            additional_torques={m.name: 0.0 for m in motors},
        )

    def _brake(self, owner: ArmsControlMode) -> Segment:
        """Stop the arm: plan a brake to rest from the measured state, as fast as the stored acceleration and jerk
        limits allow, ending wherever that is (not back at the position it had: that overshoots and returns),
        play it as `owner`'s plan and make its rest position the stored goal, so a later `set_targets()` that names
        no position keeps the arm there. Returns the plan: its `end` is when the arm is commanded at rest, equal to its
        start when the arm already is. Raises ValueError if it can't be planned (e.g. NaN telemetry); nothing changes
        then. Call with `_ruckig_lock` held."""
        position, velocity, _ = self._get_current_motor_values()
        start = MotionState(np.array(position), np.array(velocity), self._zeros)
        limits = MotionLimits(self._ruckig_max_velocity, self._ruckig_max_acceleration, self._ruckig_max_jerk)
        brake = brake_to_rest(start, limits, currtime() - self._clock_offset)
        # The goal is the rest position (where the brake ends), at rest: one source for what the arm holds.
        self._ruckig_target_positions[:] = brake.sample(brake.end)[0]
        self._ruckig_target_velocities[:] = 0.0
        self._ruckig_target_accelerations[:] = 0.0
        # Only the brake: an earlier plan of this owner is over (the brake starts from the measured state, not from it).
        self._trajectory.write((owner, (brake,)))
        return brake

    def freeze(self, clean_parameters: bool = True) -> None:
        """Stop the arm in targets mode: brake to rest as fast as the limits allow and stay there (wherever the brake
        ends). Takes control from any owner, releasing a safety hold. If gravity compensation is disabled, then
        this may not freeze well!

        Args:
            clean_parameters: If True, reset gains, limits and extra torques to their defaults first, so the stop is
                clean even if bad parameters are set.
        """
        self._check_thread()  # a stop is a plan the thread plays
        with self._ruckig_lock:
            if self.control_mode != ArmsControlMode.Targets:
                self._switch_owner(ArmsControlMode.Targets)  # not set_control_mode(): that calls freeze() itself
            if clean_parameters:
                self._set_targets_helper(**self._default_parameters())  # not clean_parameters(): that plans a move
                info(f"{self.config.module_name}: freeze reset gains, limits and extra torques to their defaults")
            brake = self._brake(ArmsControlMode.Targets)
        log_event("freeze", self.config.module_name, clean_parameters=clean_parameters,
                  stop_s=brake.end - brake.start)

    def hold(self, reason: str, block: bool = False) -> None:
        """Safety takeover: take control from any owner, brake to rest with the default gains and limits, and stay
        there until `set_control_mode()` releases it (`set_targets()` raises NotOwnerError until then). Logged as a
        warning. Called by the command watchdog and by `abort()` with `on_abort_action="freeze"`.

        Args:
            reason: Why the arm is held, shown in the log and in `set_targets()` errors.
            block: If True and the arm is enabled, wait until the stop has finished (commanded velocity zero). Raises
                TimeoutError `_HOLD_SETTLE_TIMEOUT` seconds after the brake should have ended.
        """
        self._check_thread()  # the abort routes a dead arm to disable(); the watchdog runs in a live thread
        with self._ruckig_lock:  # until the hold's own plan is stored, ticks send nothing (it's labelled for hold)
            previous = self._command.switch_owner(ArmsControlMode.Hold)
            self._hold_reason = reason
            self._set_targets_helper(**self._default_parameters())
            planned = True
            try:
                brake = self._brake(ArmsControlMode.Hold)
                stop_end, stop_s = brake.end, brake.end - brake.start
            except ValueError:  # e.g. NaN telemetry: a safety hold must never fail, so hold where the arm is, unplanned
                planned, stop_s = False, 0.0
                error(f"{self.config.module_name}: couldn't plan the hold's smooth stop; holding in place:\n"
                      f"{format_exc()}")
                self._ruckig_target_positions[:] = self._get_current_motor_values()[0]
                self._trajectory.write((ArmsControlMode.Hold, (self._still_at(self._ruckig_target_positions),)))
                stop_end = currtime() - self._clock_offset
        warning(f"{self.config.module_name}: safety hold ({reason}); took control from {previous.value} and reset "
            "gains and limits to their defaults")
        log_event("hold", self.config.module_name, reason=reason, taken_from=previous.value, planned=planned,
                  stop_s=stop_s)

        if block and self._enabled:
            def stopped() -> bool:  # command stamps are wall time; stop_end is on the arm's clock
                command = self._command.read()
                return command is not None and command.stamp >= stop_end + self._clock_offset and \
                    bool(np.allclose(command.velocity, 0.0))
            exponential_timeout(max(stop_end + self._clock_offset - currtime(), 0.0) + _HOLD_SETTLE_TIMEOUT,
                                await_true=stopped, start_delay=0.01, max_delay=0.05)

    def set_control_mode(self, control_mode: Union[ArmsControlMode, _ArmsControlModeStr]) -> None:
        """Sets the control mode for the arm.
        
        Switching to targets, trajectory or hold brakes the arm to rest first (as `freeze()` does). Follow continues
        from the arm's motion: its first steps start from the last setpoint's position and velocity. Direct and
        impedance modes can't play a brake, so switching to them raises ValueError while the arm is moving: stop it
        first (`hold(reason, block=True)`, or `freeze()` and wait). Does nothing if `control_mode` is already the
        current control mode.

        In impedance control mode, if external (non-gravity) forces are applied, the arms will move around.

        The control mode owns the arm's command slot, so switching it is the only way to change which control law
        commands the motors. Each switch is logged. Switching to hold calls `hold()`; switching away from hold releases
        a safety hold.
        
        Args:
            control_mode: The new control mode to set. Can be a string or an `ArmsControlMode` enum.
        """
        self._check_thread()
        if isinstance(control_mode, str):
            control_mode = ArmsControlMode(control_mode)
        if control_mode == self.control_mode:
            return
        
        if control_mode == ArmsControlMode.Targets:
            self.freeze()  # takes the slot and brakes to rest, played in targets mode
        elif control_mode == ArmsControlMode.Impedance:
            if not self.config.apply_gravity_compensation:
                raise ValueError("Impedance control mode requires gravity compensation to be enabled")
            self._check_at_rest(control_mode)
            self._motor_impedance_positions = np.array([m.telemetry[m.PARAMS.FB_POSITION, -1] for m in self.motor_order])
            self._switch_owner(control_mode)
        elif control_mode == ArmsControlMode.Hold:
            self.hold("requested through set_control_mode()")
        elif control_mode == ArmsControlMode.Direct:
            self._check_at_rest(control_mode)
            self._switch_owner(control_mode)
            # Hold at rest until the first set_command(), at the held position, so the motors' setpoint doesn't move
            # and read_state().command.position is where a caller's steps start. The watchdog holds the arm if no
            # setpoint comes in time.
            self._command.write(control_mode, ArmCommand.make(
                self._held_position(), self._zeros, self._zeros, self._default_kp, self._default_kd))
        elif control_mode == ArmsControlMode.Follow:
            # Continue from the last setpoint, moving as it was, and come to rest at the held position until the first
            # set_follow_target(). Made before the switch, so a tick never finds follow mode without its generator.
            start = read_only(self._held_position())
            sent = self._sent
            moving = sent[0].velocity if self._enabled and sent is not None else None
            self._follower = OnlineFollower(start, self._follow_limits.read(), self._tick, moving)
            self._follow_target_version = self._follow_target.write((start, self._zeros))
            self._follow_limits_version = self._follow_limits.read_versioned()[0]
            self._switch_owner(control_mode)
        elif control_mode == ArmsControlMode.Trajectory:
            # The brake is the first trajectory, played until set_trajectory() splices onto it. Written before the
            # switch, so a tick never finds trajectory mode without a trajectory.
            with self._ruckig_lock:
                self._brake(control_mode)
                self._switch_owner(control_mode)
        else:
            raise ValueError(f"Invalid control mode: {control_mode}")

    def _held_position(self) -> np.ndarray:
        """Where the motors are held: the position last sent. Disabled or nothing sent yet (the last send may be stale;
        the limp arm may have been moved): where the arm is."""
        sent = self._sent
        if self._enabled and sent is not None:
            return sent[0].position
        measured = self.capture_position()
        return np.array([measured[n] for n in self._motor_names])

    def _check_at_rest(self, control_mode: ArmsControlMode) -> None:
        """Raises ValueError if the newest command still moves the arm: direct, follow and impedance modes start from
        rest."""
        command = self._command.read()
        if command is not None and not np.allclose(command.velocity, 0.0):
            raise ValueError(f"{self.config.module_name} is moving (commanded velocity {command.velocity}); stop it "
                             f"before switching to {control_mode.value} mode: hold(reason, block=True)")

    def set_command(
        self,
        positions: Dict[str, float],
        velocities: Optional[Dict[str, float]] = None,
        efforts: Optional[Dict[str, float]] = None,
        kps: Optional[Dict[str, float]] = None,
        kds: Optional[Dict[str, float]] = None,
    ) -> None:
        """Direct mode: send a setpoint to every motor on the next tick, with no motion planning.

        Call it at a steady rate: if the newest setpoint gets older than `config.command_timeout`, the arm takes a
        safety hold. Gravity compensation is added on top of `efforts`. Each dict is keyed by motor name and must have
        every motor of the arm. `velocities` and `efforts` default to 0; `kps` and `kds` to each motor's `default_op_kp`
        and `default_op_kd`.

        Raises:
            NotOwnerError: The arm is not in direct mode.
            RuntimeError: The arm's thread died.
            ValueError: A motor is missing or unknown, a position is outside the motor's `angle_range` by more than
                a float32 step (positions reach the motor as float32), or a velocity is above its `max_velocity`.
                The previous setpoint stays in effect. How far a setpoint is from the previous one isn't checked: the
                caller paces its setpoints (`planning.step_toward()` steps within the limits).
            CollisionError: (a ValueError) With `set_collision_check()` on, the setpoint brings the arm into the torso
                or the base. The previous setpoint stays in effect: send it again to hold there.
        """
        self._check_thread()
        if self.control_mode != ArmsControlMode.Direct:
            raise NotOwnerError(f"{self.config.module_name} is in {self.control_mode.value} mode, not direct")
        self._command.write(ArmsControlMode.Direct, self._direct_command(positions, velocities, efforts, kps, kds))

    def _direct_command(
        self,
        positions: Dict[str, float],
        velocities: Optional[Dict[str, float]],
        efforts: Optional[Dict[str, float]],
        kps: Optional[Dict[str, float]],
        kds: Optional[Dict[str, float]],
    ) -> ArmCommand:
        """Check a direct setpoint and build its command, without sending it. Raises ValueError as `set_command()`
        describes. A hot path (called for every streamed setpoint): inline checks, values precomputed in `__init__`."""
        names = self._motor_names
        for values in (positions, velocities, efforts, kps, kds):
            if values is not None and values.keys() != self._motor_name_set:
                raise ValueError(f"Need exactly the motors {names}, got {sorted(values)}")
        # Each check is written as "all within limits": NaN fails every comparison and inf fails the bounds, so neither
        # can slip through. Defaults are valid already, so only values the caller gave are checked.
        q = np.array([positions[n] for n in names], dtype=np.float64)
        if not ((self._angle_low_check <= q) & (q <= self._angle_high_check)).all():
            raise ValueError(f"Positions outside angle_range (or NaN): {dict(zip(names, q))}")
        qd, effort, kp, kd = self._zeros, self._zeros, self._default_kp, self._default_kd
        if velocities is not None:
            qd = np.array([velocities[n] for n in names], dtype=np.float64)
            if not (np.abs(qd) <= self._max_velocity).all():
                raise ValueError(f"Velocities above max_velocity (or NaN): {dict(zip(names, qd))}")
        if efforts is not None:  # the Teensy would clamp effort plus gravity silently
            effort = np.array([efforts[n] for n in names], dtype=np.float64)
            if not (np.abs(effort) <= self._max_effort).all():
                raise ValueError(f"Efforts above max_torque (or NaN): {dict(zip(names, effort))}")
        if kps is not None:  # the Teensy would clamp a negative kp to 0: a limp joint
            kp = np.array([kps[n] for n in names], dtype=np.float64)
            if not ((0.0 <= kp) & (kp <= self._max_kp)).all():
                raise ValueError(f"kp outside the motors' limits (or NaN): {dict(zip(names, kp))}")
        if kds is not None:
            kd = np.array([kds[n] for n in names], dtype=np.float64)
            if not ((0.0 <= kd) & (kd <= self._max_kd)).all():
                raise ValueError(f"kd outside the motors' limits (or NaN): {dict(zip(names, kd))}")
        collision = self._collision
        if collision is not None:  # last: the costliest check (2 ms on the robot)
            check, torso = collision
            previous = self._command.read()  # the sweep from the previous setpoint
            contact = check.refused(None if previous is None else previous.position, (q,), torso())
            if contact is not None:
                raise self._collision_error("setpoint", contact, check.margin)
        return ArmCommand.make(q, qd, effort, kp, kd)

    def set_follow_target(self, positions: Dict[str, float], velocities: Optional[Dict[str, float]] = None) -> None:
        """Follow mode: move toward `positions`, arriving with `velocities` (default: at rest).

        Call it at any rate and from any thread. Every tick the arm thread takes one step toward the newest target,
        within the follow limits (`set_follow_limits()`) and from its own previous step, never from a measurement, so
        uneven or late targets don't make the motion jerk. With no new target the arm ends on the last one; there is
        no watchdog hold. Each dict is keyed by motor name and must have every motor of the arm.

        Raises:
            NotOwnerError: The arm is not in follow mode.
            RuntimeError: The arm's thread died.
            ValueError: A motor is missing or unknown, a position is outside the motor's `angle_range`, or a velocity
                is above its follow limit. The previous target stays.
            CollisionError: (a ValueError) With `set_collision_check()` on, the way from the previous target to this
                one brings the arm into the torso or the base. The previous target stays.
        """
        self._check_thread()
        if self.control_mode != ArmsControlMode.Follow:
            raise NotOwnerError(f"{self.config.module_name} is in {self.control_mode.value} mode, not follow")
        self._follow_target.write(self._follow_target_arrays(positions, velocities))

    def _follow_target_arrays(
        self, positions: Dict[str, float], velocities: Optional[Dict[str, float]],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Check a follow target and return it as read-only arrays in motor order, without setting it. Raises as
        `set_follow_target()` describes. A hot path (called for every streamed target): inline checks."""
        names = self._motor_names
        for values in (positions, velocities):
            if values is not None and values.keys() != self._motor_name_set:
                raise ValueError(f"Need exactly the motors {names}, got {sorted(values)}")
        # Written as "all within limits", so NaN fails every comparison.
        q = np.array([positions[n] for n in names], dtype=np.float64)
        if not ((self._angle_low_check <= q) & (q <= self._angle_high_check)).all():
            raise ValueError(f"Positions outside angle_range (or NaN): {dict(zip(names, q))}")
        qd = self._zeros
        if velocities is not None:
            qd = np.array([velocities[n] for n in names], dtype=np.float64)
            if not (np.abs(qd) <= self._follow_limits.read().velocity).all():
                raise ValueError(f"Velocities above the follow limits (or NaN): {dict(zip(names, qd))}")
        collision = self._collision
        if collision is not None:  # last: the costliest check
            check, torso = collision
            previous = self._follow_target.read()
            contact = check.refused(None if previous is None else previous[0], (q,), torso())
            if contact is not None:
                raise self._collision_error("follow target", contact, check.margin)
        return read_only(q), read_only(qd)

    def set_follow_limits(
        self,
        max_velocities: Optional[Dict[str, float]] = None,
        max_accelerations: Optional[Dict[str, float]] = None,
        max_jerks: Optional[Dict[str, float]] = None,
    ) -> None:
        """The velocity, acceleration and jerk limits follow mode moves within, per motor name. Partial dicts change
        only the motors named, and the limits stay until changed. They start at each motor's `max_velocity`,
        `max_acceleration` and `max_jerk`, which they can't exceed. Takes effect on the next tick, in any mode.

        Raises:
            ValueError: A motor is unknown, or a limit isn't positive or exceeds the motor's own.
        """
        current = self._follow_limits.read()
        arrays = [np.array(values) for values in current]
        motors = self.actuators_by_name
        for i, (given, field) in enumerate(((max_velocities, "max_velocity"), (max_accelerations, "max_acceleration"),
                                            (max_jerks, "max_jerk"))):
            for name, value in (given or {}).items():
                if name not in motors:
                    raise ValueError(f"{name} is not a motor of {self.config.module_name}")
                ceiling = getattr(motors[name].config, field)
                if not 0.0 < value <= ceiling:  # written so NaN fails
                    raise ValueError(f"{name}: {field} {value} must be above 0 and at most the motor's {ceiling}")
                arrays[i][self._motor_names.index(name)] = value
        self._follow_limits.write(MotionLimits(*(read_only(a) for a in arrays)))

    def set_trajectory(
        self,
        times: Sequence[float],
        positions: Dict[str, Sequence[float]],
        velocities: Optional[Dict[str, Sequence[float]]] = None,
        efforts: Optional[Dict[str, Sequence[float]]] = None,
        start_time: Optional[float] = None,
    ) -> float:
        """Trajectory mode: follow timed points, replacing whatever was planned from `start_time` on.

        The arm moves from where it is planned to be at `start_time` (so nothing jumps) to the first point, then through
        the rest. With `velocities`, points are joined by smooth curves that match them; without, by straight lines.
        After the last point the arm holds there until the next trajectory. Gravity compensation is added on top of
        `efforts`; gains are each motor's `default_op_kp` and `default_op_kd`. Call it from one thread at a time.
        `start_time` and the returned end are wall time; a stall of the arm thread delays the trajectory by the stall.

        Args:
            times: Seconds after `start_time` of each point, strictly increasing and above 0. At least one point.
            positions: Per motor name, one position per point (rad). Must have every motor of the arm.
            velocities: Per motor name, one velocity per point (rad/s), or None.
            efforts: Per motor name, one feed-forward torque per point (N·m), or None for 0.
            start_time: Host time (`currtime()`) the trajectory starts at; None for now. Not in the past.

        Returns:
            The host time at which the last point is reached.

        Raises:
            NotOwnerError: The arm is not in trajectory mode.
            ValueError: A motor is missing or unknown, there are no points, or the points can't be followed: NaN or
                infinite values, positions outside `angle_range`, velocities above `max_velocity`, efforts above
                `max_torque`, or points too far apart for their times. The current plan stays.
            CollisionError: (a ValueError) With `set_collision_check()` on, the plan brings the arm into the torso or
                the base. The current plan stays.
        """
        self._check_thread()
        if self.control_mode != ArmsControlMode.Trajectory:
            raise NotOwnerError(f"{self.config.module_name} is in {self.control_mode.value} mode, not trajectory")
        now = currtime()
        start = now if start_time is None else start_time
        if not start >= now:  # also rejects NaN
            raise ValueError(f"start_time {start} is in the past (now is {now})")
        if len(times) == 0:
            raise ValueError("A trajectory needs at least one point")
        names = self._motor_names
        for values in (positions, velocities, efforts):
            if values is not None and values.keys() != self._motor_name_set:
                raise ValueError(f"Need exactly the motors {names}, got {sorted(values)}")
        # Shape (points, motors); ragged per-motor sequences make numpy raise ValueError.
        q = np.array([positions[n] for n in names], dtype=np.float64).T
        qd = None if velocities is None else np.array([velocities[n] for n in names], dtype=np.float64).T
        e = np.zeros_like(q) if efforts is None else np.array([efforts[n] for n in names], dtype=np.float64).T

        # Begin at the planned state at `start`, as ros2_control's trajectory controller does with the current state.
        owner, segments = ensure(self._trajectory.read())
        if owner != ArmsControlMode.Trajectory:
            raise UnreachableError(f"Trajectory mode is playing a plan made for {owner.value}")
        offset = self._clock_offset  # read once: the arm thread may add to it meanwhile
        start -= offset  # onto the arm's clock, where plans are timed
        q0, qd0, _, e0 = playing(segments, start).sample(start)
        new = ArmTrajectory.make(start, np.concatenate([[0.0], times]), np.vstack([q0, q]),
                                 None if qd is None else np.vstack([qd0, qd]), np.vstack([e0, e]))
        # The same float32-widened range as set_command(): trajectory points reach the motor as float32 too.
        check_trajectory(new, self._angle_low_check, self._angle_high_check, self._max_velocity, self._max_effort)
        collision = self._collision
        if collision is not None:  # the plan as the arm will play it, one sample per tick
            check, torso = collision
            ticks = np.append(np.arange(new.start + self._tick, new.end, self._tick), new.end)
            contact = check.refused(q0, (new.sample(t)[0] for t in ticks), torso())
            if contact is not None:
                raise self._collision_error("trajectory", contact, check.margin)
        self._trajectory.write((ArmsControlMode.Trajectory, splice(segments, new, now - offset)))
        log_event("trajectory", self.config.module_name, start=start + offset, end=new.end + offset,
                  times=list(times), positions=positions, velocities=velocities, efforts=efforts)
        return new.end + offset  # back to wall time

    def set_collision_check(self, on: bool, margin: float = 0.03) -> None:
        """Turn on (or off) refusing motions that bring this arm within `margin` (m) of the torso or the base.

        While on, `set_command()` and `set_trajectory()` raise `CollisionError` for a motion that comes within
        `margin` of them, or closer than the arm already is, so an arm already that close can always move away; the
        arm stays where it was sent last, as for any refused command. The torso is guarded too
        (`LinearEZMotionTorsoSystem.set_guard()`): a move down stops where it would bring the arm, where it is
        measured, within `margin` of the base (or closer, or into it), and one that can't go down at all raises
        `CollisionError` from `go_to()` and `update_velocity()`; up always goes.

        The check places the URDF's collision meshes (the arm's as convex hulls, the body's simplified to 2 cm) at the
        torso's newest height, the neck and steering at their zero: about 2 ms an arm on the robot, up to 7 ms within
        the margin. Not checked: arm against arm, the neck, the hands (the URDF has none), `set_targets()` and
        `home()`. Turning it on the first time loads and simplifies the meshes: about 30 s on the robot.

        Raises:
            ValueError: No URDF is loaded, the robot has no torso or its config lacks `urdf_joint_offset` or the lift
                motor's `urdf_joint_name`, or the URDF has no collision geometry for this arm.
        """
        if not on:
            self._collision = None
            torso = getattr(ensure(self.robot), "torso_system", None)
            if torso is not None:
                torso.set_guard(self.config.module_name, None)
            info(f"{self.config.module_name}: collision check off")
            return
        robot = ensure(self.robot)
        loader, torso = robot.urdf_loader, getattr(robot, "torso_system", None)
        if loader is None or torso is None:
            raise ValueError(f"{self.config.module_name}: the collision check needs the URDF and the torso (urdf "
                             f"{'loaded' if loader else 'missing'}, torso {'present' if torso else 'missing'})")
        torso_joint = torso.describe().joint  # checks the torso's URDF mapping
        torso.urdf_position()  # raises now, not in a command, if the torso can't say where it is
        arm = (loader.pin_import, loader.pin_model, loader.pin_collision_hulls, loader.pin_collision_coarse,
               [m.config.urdf_joint_name for m in self.motor_order], [m.config.urdf_direction for m in self.motor_order],
               torso_joint, margin)
        check = BodyCheck(*arm)
        self._collision = (check, torso.urdf_position)
        # The torso's own check, for its thread (a BodyCheck is one thread's): the arm where it is measured, against
        # what the torso lowers it toward.
        lowering = BodyCheck(*arm, fixed_only=True)
        names = self._motor_names

        def guard(torso_q: float, lower_q: float) -> Optional[Tuple[float, CollisionError]]:
            measured = self.capture_position()
            floor = lowering.lowest(np.array([measured[n] for n in names]), torso_q, lower_q)
            return None if floor is None else (floor[0], self._collision_error("torso going down", floor[1], margin))

        torso.set_guard(self.config.module_name, guard)
        info(f"{self.config.module_name}: collision check on, margin {margin} m; never checked (they meet at home): "
             f"{check.skipped}")

    def _collision_error(self, what: str, contact: Contact, margin: float) -> CollisionError:
        return CollisionError(f"{self.config.module_name}: the {what} brings {contact.arm_link} within "
                              f"{contact.distance * 1000:.0f} mm of {contact.body_link} (margin "
                              f"{margin * 1000:.0f} mm)", contact)

    @property
    def planning(self) -> Planning:
        """How `set_targets()`, `home()` and `freeze()` plan: a path planner, then a motion generator. The default is a
        straight line timed by Ruckig. Change it with `set_planning()`."""
        return self._planning

    def set_planning(self, planning: Planning) -> None:
        """Use `planning` for `set_targets()`, `home()` and `freeze()` from their next call, for example
        `Planning(path_planner=my_planner)` (see `planning.py`). The safety hold keeps the default planning."""
        if not isinstance(planning, Planning):
            raise TypeError(f"Expected a Planning, got {planning!r}")
        with self._ruckig_lock:  # never swapped halfway through a set_targets() re-plan
            self._planning = planning

    @property
    def control_mode(self) -> ArmsControlMode:
        """The current control mode. It is also the owner of the arm's command slot: the only control law whose
        commands reach the motors."""
        return self._command.owner

    def _switch_owner(self, control_mode: ArmsControlMode) -> None:
        """Hand the command slot to `control_mode` and log the switch. Leaving a safety hold releases it."""
        previous = self._command.switch_owner(control_mode)
        if previous == ArmsControlMode.Hold:
            info(f"{self.config.module_name}: hold ({self._hold_reason}) released")
            log_event("hold_released", self.config.module_name, reason=self._hold_reason)
            self._hold_reason = None
        info(f"{self.config.module_name}: control switched from {previous.value} to {control_mode.value}")
        log_event("control_mode", self.config.module_name, previous=previous.value, mode=control_mode.value)

    @property
    def payload_mass(self) -> float:
        """The mass (kg) of the object currently set as held in the hand. 0.0 if none."""
        return self._payload_mass

    def set_payload(self, mass: float, com: Optional[List[float]] = None) -> None:
        """Tells the arm it is holding an object so gravity compensation also holds up the object's weight.

        The object is modeled as a point mass rigidly attached to the hand. Takes effect on the next control tick. Call
        `clear_payload()` when the object is released, otherwise the arm will push upward.

        NOTE: The motor's `gravity_multiplier` is also applied to the object's gravity torque.

        Args:
            mass: The mass of the object in kg. Must be >= 0. 0 removes the payload.
            com: The object's center of mass [x, y, z] in meters, in the frame of the `config.hand_joint_name` joint.
                If None, uses the hand's own center of mass from the URDF.
        """
        if self._hand_joint_id is None:
            raise ValueError("Cannot set payload: arm has no URDF model (is it initialized?)")
        if mass < 0.0:
            raise ValueError(f"Payload mass must be >= 0, got {mass}")

        if mass == 0.0:
            new_inertia = ensure(self._hand_base_inertia).copy()
        else:
            com_arr = ensure(self._hand_base_inertia).lever.copy() if com is None else np.asarray(com, dtype=float)
            if com_arr.shape != (3,):
                raise ValueError(f"Payload COM must be [x, y, z], got {com_arr}")
            new_inertia = self._hand_base_inertia + ensure(self.pin_import).Inertia(mass, com_arr, np.zeros((3, 3)))

        ensure(self.pin_model).inertias[self._hand_joint_id] = new_inertia
        self._payload_mass = float(mass)
        info(f"Arm {self.config.module_name} payload set to {mass:.3f} kg")
        log_event("payload", self.config.module_name, mass=float(mass), com=None if com is None else list(com))

    def clear_payload(self) -> None:
        """Sets the payload weight to 0.0."""
        self.set_payload(0.0)

    def _initialize(self, load_calibration: Optional[bool] = None) -> None:
        """Initialize the arm."""
        # Get pinocchio info for gravity compensation if provided
        robot = ensure(self.robot)
        if robot.urdf_loader is not None:
            self.pin_import = robot.urdf_loader.pin_import
            # Copy the model so that payload changes to this arm don't affect other users of the shared URDF model
            self.pin_model = robot.urdf_loader.pin_model.copy()
            self.pin_data = ensure(self.pin_model).createData()

            arm_names = [m.config.urdf_joint_name for m in self.motor_order]
            if any(not name for name in arm_names):
                missing = [m.config.name for m in self.motor_order if not m.config.urdf_joint_name]
                raise ValueError(f"Arm motors missing urdf_joint_name required for gravity compensation: {missing}")

            self.pin_arm_q_inds, self.pin_arm_v_inds = get_arm_pin_config_indices(self.pin_model, arm_names)

            self._hand_joint_id = get_pin_joint_id(self.pin_model, self.config.hand_joint_name)
            self._hand_base_inertia = ensure(self.pin_model).inertias[self._hand_joint_id].copy()
        elif self.config.apply_gravity_compensation:
            raise ValueError("Gravity compensation is enabled, but no URDF model is provided")
        
        self.recalibrate(load_calibration=load_calibration)

        self._start_thread()
    
    def _calibrate_and_pos_motor(self, motor: RobstrideMotor, position: float = 0.0) -> None:
        """Calibrate a motor, then set it to the given position and wait for it to reach the position."""
        motor._calibrate(load_calibration=False, home=False)
        motor.set_run_mode(RobRunMode.POSITION_PP, block=True, check=True)
        speed = motor.config.calibration_config.speed * motor.config.calibration_config.fast_find_speed_mult
        motor.set_target_velocity(speed, block=True, check=True)
        motor.enable(block=True)
        motor.set_target_position(position, block=True, check=True)

        timeout = motor.config.calibration_config.torque_limit_timeout
        debug(f"[{motor.config.name}]: calibration complete, moving to position {position}")
        motor.await_param(motor.PARAMS.FB_POSITION, position, 0.01, check_errors=True, 
                          torque_limit=motor.config.calibration_config.torque_threshold, timeout=timeout)
    
    def recalibrate(self, load_calibration: Optional[bool] = None) -> None:
        """Recalibrate the arm."""
        debug(f"Recalibrating arm {self.config.module_name} with load_calibration={load_calibration}")
        if load_calibration is None or load_calibration:
            try:
                debug(f"Loading full calibration state for arm {self.config.module_name}")
                calibrate_actuator_group(list(self.actuators_by_name.values()), load_calibration=True, only_load=True)
                return
            except Exception:
                info(f"Could not load full calibration state for arm {self.config.module_name}: {format_exc()}")

        log_event("recalibrate", self.config.module_name, load_calibration=load_calibration)
        if self.config.require_approval_for_calibration:
            notice(ARM_RECALIBRATION_NOTICE.format(arm_name=self.config.module_name))
            flush_logs()
            if input().lower() not in ["y", "yes"]:
                raise ValueError("Calibration cancelled by user")

        # Calibrate the motors that don't need a special position and can just return home
        self._calibrate_and_pos_motor(self.shoulder_yaw_motor, position=0.0)
        self._calibrate_and_pos_motor(self.wrist_yaw_motor, position=0.0)
        self._calibrate_and_pos_motor(self.wrist_pitch_motor, position=0.0)

        # Calibrate elbow, then move to wrist-roll calibration position (bend elbow up a little so we don't hit our 
        # torso) and calibrate wrist roll motor
        # For shaft-oriented coordinates, we need to flip the sign on only the left motor.
        elb_min, elb_max = self.elbow_motor.config.angle_range
        elb_mult = -1.0 if abs(elb_min) > abs(elb_max) else 1.0
        self._calibrate_and_pos_motor(self.elbow_motor, position=elb_mult * deg2rad(135))
        self._calibrate_and_pos_motor(self.wrist_roll_motor, position=0.0)

        # Lock the shoulder roll in place before calibrating shoulder pitch. This motor is not homed
        # yet, so FB_POSITION is the raw encoder angle and is sometimes outside the configured joint range.
        # Skip the Teensy LOC_REF range check for this hold. Clamping would command a different angle.
        self.shoulder_roll_motor.set_run_mode(RobRunMode.POSITION_PP, block=True, check=True)
        self.shoulder_roll_motor.enable(block=True)
        pos = self.shoulder_roll_motor.read_param_await(self.shoulder_roll_motor.PARAMS.FB_POSITION)
        self.shoulder_roll_motor.set_skip_write_param_checks(True)
        try:
            self.shoulder_roll_motor.set_target_position(pos, block=True, check=True)
        finally:
            self.shoulder_roll_motor.set_skip_write_param_checks(False)

        # Calibrate pitch motor, then move outwards so we can reach full calibration range of shoulder roll motor
        # We need to flip the sign on the pitch motor if it's on the left side of the robot
        shr_mult = -1.0 if self.config.side == "left" else 1.0
        self._calibrate_and_pos_motor(self.shoulder_pitch_motor, position=shr_mult * deg2rad(-50))
        self._calibrate_and_pos_motor(self.shoulder_roll_motor, position=0.0)

        # Move elbow and shoulder and back to home position
        param_mech_pos = self.elbow_motor.PARAMS.FB_POSITION
        timeout = self.elbow_motor.config.calibration_config.torque_limit_timeout
        self.elbow_motor.set_target_position(0, block=True, check=True)
        self.shoulder_pitch_motor.set_target_position(0, block=True, check=True)
        self.elbow_motor.await_param(param_mech_pos, 0, 0.01, check_errors=True, timeout=timeout)
        self.shoulder_pitch_motor.await_param(param_mech_pos, 0, 0.01, check_errors=True, timeout=timeout)

        # Finally, disable all the motors
        for motor in self.actuators_by_name.values():
            motor.disable(block=True)
    
    def _abort_impl(self) -> None:
        log_event("abort", self.config.module_name, action=self.config.on_abort_action, enabled=self._enabled)
        if self._enabled:
            if self.config.on_abort_action == "disable" or self._thread_dead():  # no brake without the thread either
                self.disable()
            elif self.config.on_abort_action == "freeze":
                # Wait for the stop to finish: the arm thread stops below, and motors keep their last command after it.
                with try_log_exc(f"{self.config.module_name}: safety hold on abort did not finish"):
                    self.hold("abort", block=True)
        super()._abort_impl()


class ArmsSystemConfig(ActuatorSystemConfig):
    """Configuration for a full Robstride arm system."""
    module_name: str = PyDField(default="arms_system")
    """The name of the arms system."""
    requires_urdf: bool = PyDField(default=True)
    """Whether this system requires the robot URDF to be loaded."""
    right_arm: Optional[Robstride7MArmConfig]
    """Configuration for the right arm. Leave None to not include a right arm."""
    left_arm: Optional[Robstride7MArmConfig]
    """Configuration for the left arm. Leave None to not include a left arm."""


class ArmsSystem(ActuatorSystem[ArmsSystemConfig, RobstrideMotor]):
    """A system that controls 1-2 Robstride arms."""

    right_arm: Optional[Robstride7MArm]
    """The right arm. None if not using a right arm."""
    left_arm: Optional[Robstride7MArm]
    """The left arm. None if not using a left arm."""

    def __init__(self, config: ArmsSystemConfig, robot: 'FeatherRobot'):
        super().__init__(config, robot)

        self.right_arm = Robstride7MArm(config.right_arm, robot) if config.right_arm is not None else None
        self.left_arm = Robstride7MArm(config.left_arm, robot) if config.left_arm is not None else None
        self._arms = [arm for arm in (self.right_arm, self.left_arm) if arm is not None]
        self._motor_name_set = frozenset(n for arm in self._arms for n in arm._motor_names)

    def _initialize(self, load_calibration: Optional[bool] = None) -> None:
        """Initialize the arms system."""
        if self.right_arm is not None:
            self.right_arm._initialize(load_calibration=load_calibration)
        if self.left_arm is not None:
            self.left_arm._initialize(load_calibration=load_calibration)
    
    def recalibrate(self, load_calibration: Optional[bool] = None) -> None:
        """Recalibrate the arms system."""
        if self.right_arm is not None:
            self.right_arm.recalibrate(load_calibration=load_calibration)
        if self.left_arm is not None:
            self.left_arm.recalibrate(load_calibration=load_calibration)

    def capture_position(self, trigger_feedback: bool = False) -> Dict[str, float]:
        """Capture the position of the arms.
        
        Args:
            trigger_feedback: Whether to trigger a feedback frame for the motors. You should only need to set this to
                True if you want to get telemetry from an arm that is currently disabled. This will send a request to
                the motor to respond with current telemetry.
        
        Returns:
            Dict[str, float]: Dict mapping each motor name to its current position. Combines both arms together.
        """
        return {
            **(self.right_arm.capture_position(trigger_feedback=trigger_feedback) if self.right_arm is not None else {}),
            **(self.left_arm.capture_position(trigger_feedback=trigger_feedback) if self.left_arm is not None else {}),
        }

    def read_state(self) -> Dict[str, ArmState]:
        """Each arm's `Robstride7MArm.read_state()`, keyed by the arm's `module_name` (`left_arm`, `right_arm`)."""
        return {arm.config.module_name: arm.read_state() for arm in self._arms}

    def describe(self) -> Dict[str, ArmDescription]:
        """Each arm's `Robstride7MArm.describe()`, keyed by the arm's `module_name`. Every arm is checked before
        raising, so one error lists the mismatches of both."""
        descriptions, errors = {}, []
        for arm in self._arms:
            try:
                descriptions[arm.config.module_name] = arm.describe()
            except ValueError as e:
                errors.append(str(e))
        if errors:
            raise ValueError("\n".join(errors))
        return descriptions

    def set_targets(
        self,
        positions: Optional[Dict[str, float]] = None,
        max_velocities: Optional[Dict[str, float]] = None,
        max_accelerations: Optional[Dict[str, float]] = None,
        max_jerks: Optional[Dict[str, float]] = None,
        kps: Optional[Dict[str, float]] = None,
        kds: Optional[Dict[str, float]] = None,
        target_velocities: Optional[Dict[str, float]] = None,
        target_accelerations: Optional[Dict[str, float]] = None,
        additional_torques: Optional[Dict[str, float]] = None,
    ) -> None:
        """Sets the target values for one or more motors in the arm.

        Motion is controlled using Robstride's Operation Command mode. Each arm motor will apply torque according to 
        the following formula:

        `torque = kp * (target_position - current_position) + kd * (target_velocity - current_velocity) + offset`

        Where:

        - `target_position` is the target position angle to reach.
        - `current_position` is the last-received position angle of the motor.
        - `target_velocity` is the target angular velocity to reach. (This is computed automatically. See below.)
        - `current_velocity` is the last-received angular velocity of the motor.
        - `kp` is the proportional gain, applied to the position error.
        - `kd` is the derivative gain, applied to the velocity error.
        - `offset` is extra torque applied as needed. It will consist of:
          * Gravity compensation if `config.apply_gravity_compensation` is True.
          * Plus any additional torque to apply in the `offset` parameter to this function call.
        
        Motor telemetry should update at around `config.operating_frequency` Hz.

        The target velocity is computed automatically by the arm's planning (a straight line timed by Ruckig unless
        `set_planning()` changed it).

        NOTE: Values will be clamped to the motor's limits before sending commands to the motor. EG: using a
        `max_velocity` higher than the motor's `config.max_velocity` will be clamped to the motor's
        `config.max_velocity`.

        NOTE: All parameters are optional. If None, then none of the motors' parameters for that value will be changed.
        Otherwise it should be a dictionary with keys being either 1) the string name of the motor or 2) the
        RobstrideMotor object itself. Values are the new target values for that parameter. Missing motor keys will
        not have their parameters changed. Parameter values that are `None` will not be changed.
        
        Args:
            positions: New target position angles. See Note above for format details.
            max_velocities: New target maximum velocities. See Note above for format details.
            max_accelerations: New target maximum accelerations. See Note above for format details.
            max_jerks: New target maximum jerks. See Note above for format details.
            kps: New proportional gains. See Note above for format details.
            kds: New derivative gains. See Note above for format details.
            target_velocities: New target velocities. These are the velocities we want the motors to have when reaching
                the target position. Set to 0 to have motor stop at target position. See Note above for format details.
            target_accelerations: New target accelerations. These are the accelerations we want the motors to have when 
                reaching the target position. Set to 0 not have instantaneous acceleration change upon reaching target. 
                See Note above for format details.
            additional_torques: New additional torques to apply. See Note above for format details.
        """
        if self.right_arm is not None:
            self.right_arm.set_targets(
                positions=positions,
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                max_jerks=max_jerks,
                kps=kps,
                kds=kds,
                target_velocities=target_velocities,
                target_accelerations=target_accelerations,
                additional_torques=additional_torques,
            )
        if self.left_arm is not None:
            self.left_arm.set_targets(
                positions=positions,
                max_velocities=max_velocities,
                max_accelerations=max_accelerations,
                max_jerks=max_jerks,
                kps=kps,
                kds=kds,
                target_velocities=target_velocities,
                target_accelerations=target_accelerations,
                additional_torques=additional_torques,
            )
    
    def enable(self) -> None:
        """Enable the arms system."""
        if self.right_arm is not None:
            self.right_arm.enable()
        if self.left_arm is not None:
            self.left_arm.enable()
    
    def disable(self) -> None:
        """Disable the arms system. Moves to home position first."""
        # Move both to home at the same time without blocking, then block for both
        if self.right_arm is not None and self.left_arm is not None:
            if self.right_arm._enabled and not self.right_arm._thread_dead():
                self.right_arm.home(block=False)
            if self.left_arm._enabled and not self.left_arm._thread_dead():
                self.left_arm.home(block=False)
        
        if self.right_arm is not None and self.right_arm._enabled:
            self.right_arm.disable()
        if self.left_arm is not None and self.left_arm._enabled:
            self.left_arm.disable()

    def home(self, block: bool = True) -> None:
        """Home the arms system. Will home both arms at the same time.
        
        Args:
            block: Whether to block the current thread until the home is complete. If False, will return immediately.
        """
        if self.right_arm is not None:
            self.right_arm.home(block=block)
        if self.left_arm is not None:
            self.left_arm.home(block=block)
    
    def go_to(self, positions: Dict[str, float], max_velocity: float, block: bool = True,
              tolerance: float = _GO_TO_TOLERANCE, timeout: Optional[float] = None) -> None:
        """Both arms to `positions` ({motor name: rad}, of either arm) together. See `Robstride7MArm.go_to()`."""
        moves = []
        for arm in self._arms:
            own = {name: value for name, value in positions.items() if name in arm.actuators_by_name}
            if own:
                moves.append((arm, own))
        unknown = set(positions) - {name for _, own in moves for name in own}
        if unknown:
            raise ValueError(f"No arm has motors {sorted(unknown)}")
        for arm, own in moves:
            arm.go_to(own, max_velocity, block=False)
        if block:
            _wait_until_at(moves, max_velocity, tolerance, timeout)

    def freeze(self) -> None:
        """Freeze the arms system. The arms will hold their current position at the time of this call."""
        if self.right_arm is not None:
            self.right_arm.freeze()
        if self.left_arm is not None:
            self.left_arm.freeze()
    
    def hold(self, reason: str, block: bool = False) -> None:
        """Safety hold on both arms. See `Robstride7MArm.hold()`."""
        for arm in self._arms:
            arm.hold(reason, block=block)

    def set_command(
        self,
        positions: Dict[str, float],
        velocities: Optional[Dict[str, float]] = None,
        efforts: Optional[Dict[str, float]] = None,
        kps: Optional[Dict[str, float]] = None,
        kds: Optional[Dict[str, float]] = None,
    ) -> None:
        """Direct setpoints for both arms; each dict has every motor of both. See `Robstride7MArm.set_command()`. Both
        arms' setpoints are checked, then written together, so a rejected setpoint or a mode switch never leaves only
        one arm moved."""
        for values in (positions, velocities, efforts, kps, kds):
            if values is not None and not values.keys() <= self._motor_name_set:
                raise ValueError(f"Not a motor of these arms: {sorted(values.keys() - self._motor_name_set)}")
        for arm in self._arms:
            arm._check_thread()  # this door writes the arms' slots itself, past their own set_command()
            if arm.control_mode != ArmsControlMode.Direct:
                raise NotOwnerError(f"{arm.config.module_name} is in {arm.control_mode.value} mode, not direct")
        commands = []
        for arm in self._arms:
            mine = arm._motor_name_set
            commands.append(arm._direct_command(
                {n: v for n, v in positions.items() if n in mine},
                None if velocities is None else {n: v for n, v in velocities.items() if n in mine},
                None if efforts is None else {n: v for n, v in efforts.items() if n in mine},
                None if kps is None else {n: v for n, v in kps.items() if n in mine},
                None if kds is None else {n: v for n, v in kds.items() if n in mine},
            ))
        # Both or neither: a mode switch on either arm since the check above can't leave only one arm moved.
        OwnedSlot.write_together(ArmsControlMode.Direct, [(arm._command, c) for arm, c in zip(self._arms, commands)])

    def set_follow_target(self, positions: Dict[str, float], velocities: Optional[Dict[str, float]] = None) -> None:
        """Follow targets for both arms; each dict has every motor of both. See `Robstride7MArm.set_follow_target()`.
        Both arms' targets are checked before either is set, so a refused target never moves only one arm."""
        for values in (positions, velocities):
            if values is not None and not values.keys() <= self._motor_name_set:
                raise ValueError(f"Not a motor of these arms: {sorted(values.keys() - self._motor_name_set)}")
        targets = []
        for arm in self._arms:
            arm._check_thread()
            if arm.control_mode != ArmsControlMode.Follow:
                raise NotOwnerError(f"{arm.config.module_name} is in {arm.control_mode.value} mode, not follow")
            mine = arm._motor_name_set
            targets.append(arm._follow_target_arrays(
                {n: v for n, v in positions.items() if n in mine},
                None if velocities is None else {n: v for n, v in velocities.items() if n in mine}))
        for arm, target in zip(self._arms, targets):
            arm._follow_target.write(target)

    def set_follow_limits(
        self,
        max_velocities: Optional[Dict[str, float]] = None,
        max_accelerations: Optional[Dict[str, float]] = None,
        max_jerks: Optional[Dict[str, float]] = None,
    ) -> None:
        """Each arm's `Robstride7MArm.set_follow_limits()`, with the motors of that arm."""
        for arm in self._arms:
            mine = arm._motor_name_set
            arm.set_follow_limits(*(None if values is None else {n: v for n, v in values.items() if n in mine}
                                    for values in (max_velocities, max_accelerations, max_jerks)))

    def set_collision_check(self, on: bool, margin: float = 0.03) -> None:
        """Each arm's `Robstride7MArm.set_collision_check()`: refuse motions that bring it into the torso or the
        base."""
        for arm in self._arms:
            arm.set_collision_check(on, margin)

    def set_control_mode(self, control_mode: Union[ArmsControlMode, _ArmsControlModeStr]) -> None:
        """Sets the control mode for the arms system.
        
        Switching to targets, trajectory or hold brakes the arm to rest first (as `freeze()` does). Follow continues
        from the arm's motion: its first steps start from the last setpoint's position and velocity. Direct and
        impedance modes can't play a brake, so switching to them raises ValueError while the arm is moving: stop it
        first (`hold(reason, block=True)`, or `freeze()` and wait). Does nothing if `control_mode` is already the
        current control mode.

        In impedance control mode, if external (non-gravity) forces are applied, the arms will move around.
        
        Args:
            control_mode: The new control mode to set. Can be a string or an `ArmsControlMode` enum.
        """
        for arm in self._arms:
            arm.set_control_mode(control_mode)
    
    def _abort_impl(self) -> None:
        self.disable()
        if self.right_arm is not None:
            self.right_arm._abort_impl()
        if self.left_arm is not None:
            self.left_arm._abort_impl()
        super()._abort_impl()


ARM_RECALIBRATION_NOTICE = """
===============
ARM CALIBRATION
===============

The arm `{arm_name}` requires calibration. Ensure arm is in the home position, and there is nothing within arms reach
of the robot for safety. 'y' to continue, 'n' to cancel.
"""


def _wait_until_at(moves: List[Tuple["Robstride7MArm", Dict[str, float]]], max_velocity: float, tolerance: float,
                   timeout: Optional[float]) -> None:
    """Wait until every arm in `moves` is within `tolerance` of its goal. Freezes them all on a timeout or any exception."""
    if timeout is None:
        furthest = max(arm._distance_to(goal) for arm, goal in moves)
        timeout = _GO_TO_SLACK * furthest / max_velocity + _GO_TO_SETTLE
    deadline = time.monotonic() + timeout
    try:
        while any(arm._distance_to(goal) > tolerance for arm, goal in moves):
            if time.monotonic() > deadline:
                raise TimeoutError(f"{', '.join(arm.config.module_name for arm, _ in moves)} did not reach the goal in "
                                   f"{timeout:.1f} s; frozen where they are")
            time.sleep(_GO_TO_POLL)
    except BaseException:
        for arm, _ in moves:
            arm.freeze()
        raise
