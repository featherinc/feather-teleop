import math
import threading
import time
from feathersdk.robot.actuator_systems.actuator_system import ActuatorSystem, ActuatorSystemConfig
from feathersdk.robot.actuator_systems.joints import JointDescription, check_against_urdf
from feathersdk.robot.actuators.ezmotion import (
    EC_PER_ROT, EZMotionMotor, EZMotionMotorConfig, EZMotionHomingMethod, EZMotionHaltOption, EZMotionParam,
    EZMOTION_PARAMS, EZMotionError,
)
from feathersdk.robot.actuators import TActuator
from feathersdk.utils.feathertypes import (
    Union, Literal, Optional, TYPE_CHECKING, Generic, NamedTuple, Tuple, TypeVar, ensure, Callable, Dict,
)
from feathersdk.utils import exponential_timeout, assert_in_range, try_log_exc, currtime
from feathersdk.logging import debug, error, info, warning, log_event
from feathersdk.configs import PyDField, TEST_DEFAULT, PyDBaseModel
from feathersdk.quill_defs.python_qdefs.ezmotion_qdef import EZ_MOTION_UPDATE_PARAMS_VALUE

if TYPE_CHECKING:
    from feathersdk.robot.actuator_systems.collision import CollisionError
    from feathersdk.robot.robot import FeatherRobot

Guard = Callable[[float, float], Optional[Tuple[float, "CollisionError"]]]
"""Asked how far the torso may go down from one URDF position (m) toward a lower one: None for all the way, or the
lowest position it allows (between the two) and why. See `LinearEZMotionTorsoSystem.set_guard()`."""

_WRONG_WAY_MM = 5.0
"""mm the torso may move away from a move's target beyond what braking from its speed at the move takes, before the
watchdog halts it: the drive resuming a halted move instead of the new one went the wrong way at full speed."""

_FLOOR_MARGIN_MM = 2.0
"""mm above a move's guarded target the tick looks down to: the target was the lowest the guards allowed, so arms that
moved closer to the base since make the tick halt, while the approach to an unchanged floor doesn't."""


_EZMOTION_DISTANCE_PER_ROTATION_MM = 3.973
"""The distance traveled per full rotation of the lift shaft in millimeters. Equal to a lift motor rotation when
   ``reduction_ratio`` is 1.0."""

_HEIGHT_TOLERANCE_MM = 0.5
"""Millimeters a height may sit outside [min, max] and still be accepted. Commanding 0, or reading a lift that settled
   on the stop, can be a fraction of a millimeter past the end; that is clamped into range. Anything further raises."""

_STALE_TARGET_REACHED_S = 0.5
"""Seconds after a move or halt is sent before target reached counts without first reading it cleared. Status words
   read just after a move is sent can still report the previous target as reached, measured for up to ~150 ms; a move
   that starts clears the bit well within this. Only a move with nowhere to go (already at its target) waits it out."""

_PARKED_MOVE_SPEED_MARGIN_MM_S = 1.0
"""How far above a move's velocity the torso must be moving before the move is parked behind a halt, and the speed the
   halt must come down to within before the move is sent. A smaller drop is not worth the detour."""

_PARKED_MOVE_DEADLINE_MARGIN_S = 0.5
"""Seconds past the expected end of the deceleration ramp before a parked move is sent regardless of speed."""

_ABORT_STOPPED_MM_S = 1.0
"""Speed below which an aborting torso counts as stopped once its halt reports target reached."""

_ABORT_HOLD_TIMEOUT_S = 0.5
"""Seconds an abort waits for the hold at the current position to be acknowledged before disabling anyway."""

_ABORT_POLL_S = 0.02
_ARRIVED_MM = 5.0
"""`go_to(wait=True)`: this close to the goal (mm) counts as there."""
_PROGRESS_MM = 1.0
"""`go_to(wait=True)`: the lift must gain this much (mm) toward the goal to count as still moving."""
_STALL_S = 3.0
"""`go_to(wait=True)`: seconds without that progress before the move is given up. Judged by progress, not by a time
budget, because the lift's real speed can be below the one asked for."""
_WAIT_POLL_S = 0.1
"""Seconds between lift motor reads while an abort waits."""

_STATE_READ_CYCLE = (EZMOTION_PARAMS.VELOCITY, EZMOTION_PARAMS.STATUS_WORD, EZMOTION_PARAMS.TORQUE,
                     EZMOTION_PARAMS.ERROR_STATUS)
"""The parameter the torso tick asks the lift motor for beside its position, in turn: what the tick and
`read_state()` use (the status word for target reached and faults, the torque for `torque_limit`)."""

_STATE_READ_TIMEOUT_S = 0.05
"""Seconds the tick waits for a state read's reply before asking for the next: the lift motor's own blocking-read
timeout (`message_block_timeout`)."""


class TorsoState(NamedTuple):
    """The torso at one moment, from the newest replies the lift motor sent. Returned by `read_state()`. Each value is
    the lift motor's newest reply to that parameter (they aren't one frame, as an arm's are): the tick refreshes the
    height every tick and the rest in turn."""
    height: float
    """Height from the bottom (mm)."""
    velocity: float
    """Speed (mm/s), positive up."""
    torque: float
    """Torque, in per-mil of the lift motor's rated torque."""
    stamp: float
    """When the height was read (s, the Teensy's clock, kept in sync with the host's). Unchanging: stale state."""
    errors: Tuple[EZMotionError, ...]
    """The lift motor's fault bits, decoded. Empty when it has none."""
    moving: bool
    """A move or halt was sent and hasn't reached its target."""
    target: float
    """The height (mm) of the last move sent, NaN when none is in effect (after a halt, a fault or a recalibration)."""


class TorsoDescription(NamedTuple):
    """What a ROS wrapper needs to know about the torso, as plain values. Returned by `describe()`."""
    name: str
    """The torso's `module_name`."""
    joint: str
    """URDF joint name (prismatic, `TvC`)."""
    height_range: Tuple[float, float]
    """(min, max) height from the bottom (mm) `go_to()` accepts."""
    urdf_range: Tuple[float, float]
    """The same range as the URDF joint's position (m): height / 1000 + `urdf_joint_offset`."""
    velocity: float
    """The fastest move `go_to()` accepts (mm/s)."""
    acceleration: float
    """A move's acceleration (mm/s²)."""
    deceleration: float
    """A move's or a halt's deceleration (mm/s²)."""
    rate: float
    """The torso loop's rate (Hz): a command is sent on the next tick."""


class _EZMotionActionRecalibrate:
    """An action to recalibrate the EZMotion motor."""
    pass


class _EZMotionActionReadParam(PyDBaseModel):
    """An action to read a parameter from the EZMotion motor."""
    model_config = {"arbitrary_types_allowed": True}
    param: EZMotionParam[int]
    """The parameter to read from the motor."""


class _EZMotionActionSetPosition(PyDBaseModel):
    """An action to set the position of the EZMotion motor."""
    position: int
    """The position to set the motor to in encoder counts."""
    velocity: Optional[int] = None
    """The velocity to set the motor to in encoder counts per second. If None, the velocity will be set to the target
       velocity of the lift motor."""

class _EZMotionActionHalt:
    """An action to halt the EZMotion motor on its deceleration ramp."""
    pass


_EZMotionAction = Union[
    None, _EZMotionActionRecalibrate, _EZMotionActionSetPosition, _EZMotionActionReadParam, _EZMotionActionHalt
]


class TorsoSystemConfig(ActuatorSystemConfig):
    module_name: str = PyDField(default="torso_system")
    """The name of the torso system."""
    operating_frequency: float = PyDField(gt=0.0, lt=1000.0, default=30.0)
    """The operating frequency of the torso system."""

TTorsoSystemConfig = TypeVar("TTorsoSystemConfig", bound=TorsoSystemConfig)
"""The type of the torso system configuration."""


class TorsoSystem(ActuatorSystem[TTorsoSystemConfig, TActuator], Generic[TTorsoSystemConfig, TActuator]):
    """Controls the torso of the robot."""


class LinearEZMotionTorsoSystemConfig(TorsoSystemConfig):
    """Configuration for the torso system."""
    lift_motor: EZMotionMotorConfig
    """The configuration for the torso lift motor."""
    direction: Literal[1, -1] = PyDField(default=1)
    """The direction of the torso lift motor. 1 means encoder counts increase as the height increases, -1 means encoder
       counts increase as the height decreases. Also selects the homing method so the motor always homes to the top."""
    reduction_ratio: float = PyDField(gt=0.0, le=2.0, default=1.0)
    """The gear reduction between the lift motor and the lift shaft, as motor rotations per shaft rotation. 1.0 is
       direct drive, a 4:3 reduction is 4 / 3."""
    max_height_mm: float = PyDField(gt=50, le=2000, json_schema_extra={TEST_DEFAULT: 100.0})
    """The maximum height of the torso in millimeters from the bottom."""
    min_height_mm: float = PyDField(ge=0, le=500, json_schema_extra={TEST_DEFAULT: 20.0})
    """The minimum height of the torso in millimeters from the bottom."""
    decelerate_on_zero_velocity: bool = PyDField(default=True)
    """Whether to decelerate when setting the velocity to 0 in update_velocity()."""
    shutdown_on_target_reached: bool = PyDField(default=False)
    """Whether to de-energize the lift motor once a move reaches its target or a halt comes to a stop. If False, the 
       lift motor stays energized holding its position. The next move re-energizes it."""
    decelerate_before_slower_move: bool = PyDField(default=False)
    """Whether a move slower than the torso is currently moving halts first and is sent once the halt has slowed the
       torso to its velocity. The lift motor treats a move's velocity as a hard cap rather than a ramp target, so a
       slower move sent directly (e.g. update_velocity(-10) while moving at -145 mm/s) cuts the speed in one step,
       spiking torque and current. The halt slows down on the lift motor's deceleration instead."""
    torque_limit: int = PyDField(ge=1, le=3000, default=3000)
    """Torque in per-mil of rated torque (the lift motor's 0x6077 reading) above which a move or halt de-energizes the
       lift motor, e.g. on contact or a stall. Checked every torso step from the torque already read then. The default 
       is the lift motor's 300% ceiling, which it never reads above, so it never trips."""
    abort_stop_timeout_s: float = PyDField(ge=0.0, le=10.0, default=2.0)
    """Seconds an abort waits for a moving torso to halt before de-energizing the lift motor. A stopped torso is then 
       held at its current position first, so the lift motor engages its brake before turning off and the load does not
       drop. 0 de-energizes at once, dropping a moving or halted torso until the brake engages."""
    urdf_joint_offset: Optional[float] = PyDField(default=None)
    """Position (m) of the URDF joint `lift_motor.urdf_joint_name` (prismatic, +z up) when the torso is at 0 mm, so
       URDF position = height / 1000 + urdf_joint_offset. None: no URDF mapping; `describe()` raises."""


class LinearEZMotionTorsoSystem(TorsoSystem[LinearEZMotionTorsoSystemConfig, EZMotionMotor]):
    """Controls the linear torso of the robot using an EZMotion motor."""

    lift_motor: EZMotionMotor
    """The lift motor."""

    def __init__(self, config: LinearEZMotionTorsoSystemConfig, robot: 'FeatherRobot'):
        super().__init__(config, robot)

        self._enabled = False
        self._next_action: _EZMotionAction = None
        self._action_lock = threading.Lock()
        self._height_range = (self.config.min_height_mm, self.config.max_height_mm)

        # Store the last position/velocity so we can decelerate when needed
        self._last_position = 0
        self._last_velocity: Optional[int] = 0

        # Motion state, only written by the torso thread. `_moving` is set when a move or halt is sent to the lift motor
        # and cleared once it reaches its target or comes to a stop.
        self._moving = False
        self._halting = False
        self._holding = False
        self._motion_sent_at = 0.0
        self._target_reached_cleared = False

        # A move parked behind a halt until the torso slows to its velocity, with the speed it is sent below (mm/s) and
        # the time it is sent regardless. See `decelerate_before_slower_move`.
        self._parked_move: Optional[_EZMotionActionSetPosition] = None
        self._parked_move_cutover_mm_s = 0.0
        self._parked_move_deadline = 0.0

        # The last move sent to the lift motor, while it is still what the lift motor is doing or has just done. An
        # identical move is not sent again. Cleared by anything else sent to the lift motor.
        self._last_move: Optional[_EZMotionActionSetPosition] = None

        # Set when the last move, halt or recalibration has finished, cleared when a new one is queued. A latch rather
        # than a height comparison, so a waiter can neither miss an arrival between polls nor see a stale one.
        self._target_reached_event = threading.Event()

        # The state reads in flight: sent when, and each parameter with its telemetry stamp before the request, so the
        # tick can tell its reply has landed. The tick never waits for the lift motor.
        self._state_read: Optional[Tuple[float, Tuple[Tuple[EZMotionParam[int], float], ...]]] = None
        self._state_read_index = 0
        self._faulted = False  # the lift motor's fault bit last tick, so a fault is logged once, not every tick

        # What may stop the torso going down (set_guard()), by name. Replaced whole, never changed in place: the torso
        # thread iterates the one it read.
        self._guards: Dict[str, Guard] = {}
        # Whether the lift motor's control word has its halt bit set: a halt keeps the halted move's target, and
        # clearing the bit resumes it, so the next move latches its target with the bit still set.
        self._halt_latched = False
        # Where the torso was (mm) and how fast it moved (mm/s, + up) when the current move was sent: the wrong-way
        # watchdog's reference. None while no move runs.
        self._move_from: Optional[Tuple[float, float]] = None
        self.stopped_by: Optional[str] = None
        """Why a guard last stopped or refused a move down; cleared by the next move it lets go."""

        self.lift_motor = EZMotionMotor(self.config.lift_motor, self)
    
    def _initialize(self, load_calibration: Optional[bool] = None) -> None:
        self._start_thread()
        self.recalibrate(load_calibration=load_calibration, block=True)
        self._enabled = True

    def _thread_step(self) -> None:
        """Thread step function for the torso system: the checks on the state read last tick, the next action, then
        this tick's state reads. Nothing here waits for the lift motor: seven blocking reads a tick ran this loop at
        6 Hz of 30."""
        with try_log_exc(msg="Failed to perform action in TorsoSystem"):
            if self.lift_motor.calibration_state.calibrated:
                telemetry = self.lift_motor.telemetry
                faulted = bool(int(telemetry.get_latest_value(EZMOTION_PARAMS.STATUS_WORD, default=0)) & (1 << 3))
                if faulted and not self._faulted:
                    errors = int(telemetry.get_latest_value(EZMOTION_PARAMS.ERROR_STATUS, default=0))
                    error(f"Lift motor has errors: {EZMotionError.get_errors(errors)}")
                self._faulted = faulted
                # A torque trip ends the move or halt. Otherwise the halt a parked move rides on hands over to the move
                # instead of completing.
                if self._moving and not self._check_torque_limit():
                    if self._parked_move is not None:
                        self._send_parked_move_if_slowed()
                    else:
                        self._check_target_reached()
                if self._moving and not self._halting and not self._holding:
                    self._check_direction()
                guards = self._guards
                if guards and self._moving and not self._halting and not self._holding:
                    self._check_guards(guards)
                # The halt bit is a level, so sending it again never restarts the ramp; a halt the lift motor dropped
                # costs one tick instead of the move it was meant to stop (two dropped halts once let it run 210 mm).
                if self._halting and not self._holding:
                    self.lift_motor.halt()

            self._run_next_action()

            # Asked last, after everything this tick sent: the lift motor drops a request that arrives while it is still
            # answering another, and these replies land long before the next tick's commands.
            if self.lift_motor.calibration_state.calibrated:
                self._request_state()

    def _run_next_action(self) -> None:
        """Perform the next action. Taken once: callers can replace it while it runs, and it is only cleared below if
        they have not, so a newer action is never lost."""
        with self._action_lock:
            action = self._next_action
        if action is None:
            return
        if isinstance(action, _EZMotionActionRecalibrate):
            # Halt and brake parameters are written before homing, so a failed write stops the recalibration before
            # the torso moves. Halts slow down on the lift motor's deceleration (0x6084). Written every calibration
            # rather than trusting the value stored on the drive.
            self.lift_motor.write_param(
                EZMOTION_PARAMS.HALT_OPTION_CODE, int(EZMotionHaltOption.SLOW_DOWN_RAMP), verify=True,
            )

            # Brake objects are only on the integrated-brake lift motor. A motor without one keeps holding with
            # the windings, and writing the objects would fail calibration.
            lift_config = self.config.lift_motor
            if lift_config.brake_enable:
                for param, value in (
                    (EZMOTION_PARAMS.BRAKE_ENABLE, int(lift_config.brake_enable)),
                    (EZMOTION_PARAMS.BRAKE_STOP_DELAY, lift_config.brake_stop_delay_ms),
                    (EZMOTION_PARAMS.BRAKE_OFF_SPEED, lift_config.brake_off_speed_rpm),
                    (EZMOTION_PARAMS.BRAKE_OFF_DELAY, lift_config.brake_off_delay_ms),
                ):
                    self.lift_motor.write_param(param, value, verify=True)
                # Applies the brake parameters. They read back before this but have no effect until it is written.
                self.lift_motor.write_param(
                    EZMOTION_PARAMS.UPDATE_CONTROL_PARAMETERS, EZ_MOTION_UPDATE_PARAMS_VALUE,
                )
                info(f"[{self.config.module_name}] Lift motor brake params: {{"
                     f"'enable': {self.lift_motor.read_param_await(EZMOTION_PARAMS.BRAKE_ENABLE)}, "
                     f"'stop_delay_ms': {self.lift_motor.read_param_await(EZMOTION_PARAMS.BRAKE_STOP_DELAY)}, "
                     f"'off_speed_rpm': {self.lift_motor.read_param_await(EZMOTION_PARAMS.BRAKE_OFF_SPEED)}, "
                     f"'off_delay_ms': {self.lift_motor.read_param_await(EZMOTION_PARAMS.BRAKE_OFF_DELAY)}}}")

            m = EZMotionHomingMethod.TORQUE_UP if self.config.direction == -1 else EZMotionHomingMethod.TORQUE_DOWN
            self.lift_motor._calibrate(homing_method=m)

            # Set the max and min position limits. Position 0 is the top, where homing backed off to, but the lift
            # motor measures its limits from the hard stop it homed against. Shift them by the homing offset, as
            # written to the lift motor (away from the hard stop), or the range ends the offset short at the bottom.
            limit_shift = -self.config.direction * self.config.lift_motor.homing_offset
            travel = self._distance_to_encoder_counts(self.config.max_height_mm - self.config.min_height_mm)
            if self.config.direction == 1:
                self.lift_motor.write_param(EZMOTION_PARAMS.MAX_POSITION_LIMIT, limit_shift)
                self.lift_motor.write_param(EZMOTION_PARAMS.MIN_POSITION_LIMIT, -travel + limit_shift)
            else:
                self.lift_motor.write_param(EZMOTION_PARAMS.MIN_POSITION_LIMIT, limit_shift)
                self.lift_motor.write_param(EZMOTION_PARAMS.MAX_POSITION_LIMIT, travel + limit_shift)

            self._moving, self._halting, self._holding = False, False, False
            self._parked_move = None
            self._last_move = None

            # Homing leaves the lift motor energized in homing mode, where a disable turns it off before the brake
            # engages. A move to where it stands puts it at a reached target in position mode, which de-energizes
            # like any other move, brake first.
            if self.config.shutdown_on_target_reached:
                position = self.lift_motor.read_param_await(EZMOTION_PARAMS.POSITION)
                self._send_move(_EZMotionActionSetPosition(position=position))
                self._holding = True
            else:
                self._target_reached_event.set()
        
        elif isinstance(action, _EZMotionActionSetPosition):
            self._on_move_action(action)
        elif isinstance(action, _EZMotionActionReadParam):
            self.lift_motor.read_param(action.param)
        elif isinstance(action, _EZMotionActionHalt):
            # A stop cancels a parked move; the halt it rides on is already slowing the torso
            self._parked_move = None
            # Only halt a moving torso. Starting a halt that is in progress again, or halting a hold, would restart
            # its completion, so a caller repeating update_velocity(0) would keep it from ever finishing (the tick
            # re-sends the halt bit itself). A stopped torso has nothing to halt, and halting one de-energized would
            # re-energize it.
            if self._moving and not self._halting and not self._holding:
                self._send_halt()
            elif not self._moving:
                self._target_reached_event.set()
        else:
            self._clear_action(action)
            raise ValueError(f"Invalid action: {action}")
        
        self._clear_action(action)

    def _request_state(self) -> None:
        """Ask the lift motor for its state without waiting: its position and, in turn, one of `_STATE_READ_CYCLE`,
        once the previous pair's replies have landed (their telemetry stamps advanced) or `_STATE_READ_TIMEOUT_S` has
        passed. Two reads outstanding at most: a `go_to()` already sends the lift motor six writes 5 ms apart."""
        telemetry = self.lift_motor.telemetry
        if self._state_read is not None:
            sent_at, requested = self._state_read
            if currtime() - sent_at < _STATE_READ_TIMEOUT_S and any(
                    telemetry.get_latest_timestamp(param, default=0.0) <= before for param, before in requested):
                return
        pair = (EZMOTION_PARAMS.POSITION, _STATE_READ_CYCLE[self._state_read_index])
        self._state_read_index = (self._state_read_index + 1) % len(_STATE_READ_CYCLE)
        self._state_read = (currtime(), tuple((p, telemetry.get_latest_timestamp(p, default=0.0)) for p in pair))
        for param in pair:
            self.lift_motor.request_param(param)

    def read_state(self) -> TorsoState:
        """A snapshot of the torso from the lift motor's newest replies: height, speed, torque, when the height was
        read, its faults, whether it is moving and the last move's target. Nothing blocks: what a ros2_control hardware
        interface reads once per cycle (`get_height(fresh_value=True)` queues a blocking read instead: 179 ms on the
        robot).

        Raises:
            ValueError: The lift motor has sent no state yet (the torso isn't initialized).
        """
        telemetry = self.lift_motor.telemetry
        try:
            position = int(telemetry[EZMOTION_PARAMS.POSITION, -1])
            stamp = telemetry.get_latest_timestamp(EZMOTION_PARAMS.POSITION)
            velocity = telemetry[EZMOTION_PARAMS.VELOCITY, -1]
            torque = telemetry[EZMOTION_PARAMS.TORQUE, -1]
            errors = int(telemetry[EZMOTION_PARAMS.ERROR_STATUS, -1])
        except IndexError:  # an empty telemetry ring
            raise ValueError(f"[{self.config.module_name}] the lift motor has sent no state yet; initialize the torso "
                             "first") from None
        last = self._last_move  # read once: the torso thread replaces it
        return TorsoState(
            # Not _pos_to_height(): a move can settle just outside the range, and a reading must never raise for it.
            height=self.config.max_height_mm - abs(self._encoder_counts_to_distance(position)),
            velocity=self._encoder_counts_to_velocity(velocity),
            torque=float(torque),
            stamp=stamp,
            errors=tuple(EZMotionError.get_errors(errors)),
            moving=self._moving,
            target=math.nan if last is None
            else self.config.max_height_mm - abs(self._encoder_counts_to_distance(last.position)),
        )

    def describe(self) -> TorsoDescription:
        """The torso's URDF joint, its range in both units, its speed limits and the loop rate, as plain values: what a
        ROS wrapper generates its config from, so it restates no value. The height range, mapped with
        `urdf_joint_offset`, is checked against the loaded URDF's limits for the joint.

        Raises:
            ValueError: No URDF is loaded, the config lacks `lift_motor.urdf_joint_name` or `urdf_joint_offset`, the
                joint isn't in the URDF, or the mapped range reaches outside the URDF's (both ranges named).
        """
        joint, offset = self.config.lift_motor.urdf_joint_name, self.config.urdf_joint_offset
        if not joint or offset is None:
            raise ValueError(f"[{self.config.module_name}] describe() needs lift_motor.urdf_joint_name and "
                             f"urdf_joint_offset in the config (got {joint!r}, {offset!r})")
        loader = ensure(self.robot).urdf_loader
        if loader is None:
            raise ValueError(f"[{self.config.module_name}] describe() checks the joint against the URDF, but none is "
                             "loaded (initialize the robot with a urdf_path)")
        low, high = self._height_range
        urdf_range = (self._urdf_height(low), self._urdf_height(high))
        lift = self.config.lift_motor
        description = JointDescription(
            lift.name, joint, 1, (float(urdf_range[0]), float(urdf_range[1])),  # the URDF's frame: metres, +z up
            float(self.max_velocity_mm_s) / 1000.0,
            float(self._encoder_counts_to_distance(lift.target_acceleration)) / 1000.0, None, None,
        )
        check_against_urdf(self.config.module_name, (description,), loader.pin_model)
        # Python floats throughout: ROS parameters typed double reject ints and numpy scalars.
        return TorsoDescription(
            self.config.module_name, joint, (float(low), float(high)), (float(urdf_range[0]), float(urdf_range[1])),
            float(self.max_velocity_mm_s), float(self._encoder_counts_to_distance(lift.target_acceleration)),
            float(self._encoder_counts_to_distance(lift.target_deceleration)), float(self.config.operating_frequency),
        )

    def urdf_position(self) -> float:
        """The URDF joint's position (m, +z up) at the newest height read: what the arms' collision check places the
        column at.

        Raises:
            ValueError: The config has no `urdf_joint_offset`, or the lift motor has sent no state yet.
        """
        if self.config.urdf_joint_offset is None:
            raise ValueError(f"[{self.config.module_name}] the URDF position needs urdf_joint_offset in the config")
        return self._urdf_height(self.read_state().height)

    def _urdf_height(self, height: float) -> float:
        """A height (mm from the bottom) as the URDF joint's position (m)."""
        return height / 1000.0 + ensure(self.config.urdf_joint_offset)

    def set_guard(self, name: str, guard: Optional[Guard]) -> None:
        """Let `guard` limit how far the torso goes down (None removes the guard named `name`). A move down
        (`go_to()`, `update_velocity()`) is sent to the lowest height every guard allows, so the lift motor's own
        profile stops it there (`stopped_by` says why); one no guard allows a millimetre of is refused. Each tick of a
        move down, the guards are asked about the way from where the torso is to where a halt would stop it, down to
        the move's target: arms that came closer since halt it. Moves up are never asked about. The arms' collision
        check sets one per arm (`set_collision_check()`)."""
        guards = {k: v for k, v in self._guards.items() if k != name}
        if guard is not None:
            guards[name] = guard
        self._guards = guards

    def _guarded(self, guards: Dict[str, Guard], height: float, lower: float
                 ) -> Optional[Tuple[float, "CollisionError"]]:
        """The highest floor (mm) any guard puts between `height` and `lower` (mm), and why; None for all the way."""
        here, there = self._urdf_height(height), self._urdf_height(lower)
        highest: Optional[Tuple[float, "CollisionError"]] = None
        for guard in guards.values():
            floor = guard(here, there)
            if floor is not None and (highest is None or floor[0] > highest[0]):
                highest = floor
        if highest is None:
            return None
        return (highest[0] - ensure(self.config.urdf_joint_offset)) * 1000.0, highest[1]

    def _stopping_mm(self, speed: float) -> float:
        """How far (mm) the torso goes at `speed` (mm/s) before a halt sent now stops it: a tick, then the ramp."""
        deceleration = self._encoder_counts_to_distance(self.config.lift_motor.target_deceleration)
        return speed / self.config.operating_frequency + speed * speed / (2.0 * deceleration)

    def _check_guards(self, guards: Dict[str, Guard]) -> None:
        """Halt a move down a guard no longer allows, from where the torso is to where a halt would stop it (not below
        the move's target, the lowest the guards allowed when it was sent)."""
        state = self.read_state()
        if not state.target < state.height:  # NaN (no move) or not down
            return
        lower = max(state.height - self._stopping_mm(abs(state.velocity)), state.target + _FLOOR_MARGIN_MM)
        if lower >= state.height:
            return
        floor = self._guarded(guards, state.height, lower)
        if floor is None:
            return
        refused = floor[1]
        self._parked_move = None
        self._send_halt()
        self.stopped_by = str(refused)
        warning(f"[{self.config.module_name}] stopped going down at {state.height:.0f} mm: {refused}")
        log_event("guard_halt", self.config.module_name, height_mm=float(state.height), why=str(refused))

    def _set_action(self, action: _EZMotionAction) -> None:
        """Queue the action for the torso thread, replacing any it has not taken yet."""
        with self._action_lock:
            if not isinstance(action, _EZMotionActionReadParam):
                self._target_reached_event.clear()
            self._next_action = action
    
    def _clear_action(self, action: _EZMotionAction) -> None:
        """Clear the queued action if it is still `action`, the one the torso thread just performed."""
        with self._action_lock:
            if self._next_action is action:
                self._next_action = None
    
    def _speed_mm_s(self) -> float:
        """The torso speed read this step, in millimeters per second."""
        velocity = self.lift_motor.telemetry.get_latest_value(EZMOTION_PARAMS.VELOCITY, default=0)
        return abs(self._encoder_counts_to_velocity(velocity))

    def _on_move_action(self, action: _EZMotionActionSetPosition) -> None:
        """Send a move, or with `decelerate_before_slower_move` park it behind a halt if it is slower than the torso."""
        # A repeat of the move the lift motor already has (update_velocity() in a loop) would re-send the whole move,
        # and at a reached target with shutdown_on_target_reached, re-energize the lift motor and release its brake
        last = self._last_move
        if last is not None and last.position == action.position and last.velocity == action.velocity:
            if not self._moving:
                self._target_reached_event.set()
            return

        velocity = self.config.lift_motor.target_velocity if action.velocity is None else action.velocity
        cutover_mm_s = abs(self._encoder_counts_to_velocity(velocity)) + _PARKED_MOVE_SPEED_MARGIN_MM_S
        speed_mm_s = self._speed_mm_s()

        if self.config.decelerate_before_slower_move and speed_mm_s > cutover_mm_s:
            # Halt once; a repeated slower move (update_velocity() in a loop) only replaces the parked one
            if self._parked_move is None:
                deceleration_mm_s2 = self._encoder_counts_to_distance(self.config.lift_motor.target_deceleration)
                self._parked_move_deadline = (currtime() + (speed_mm_s - cutover_mm_s) / max(deceleration_mm_s2, 1.0)
                                              + _PARKED_MOVE_DEADLINE_MARGIN_S)
                self._send_halt()
            self._parked_move, self._parked_move_cutover_mm_s = action, cutover_mm_s
            return

        # A move no slower than the torso replaces a parked one, from the halt it was riding on
        halted = self._parked_move is not None
        self._parked_move = None
        self._send_move(action, halted=halted)

    def _send_parked_move_if_slowed(self) -> None:
        """Send the parked move once the halt has slowed the torso to its velocity, or its deadline has passed."""
        speed_mm_s = self._speed_mm_s()
        if speed_mm_s > self._parked_move_cutover_mm_s:
            if currtime() < self._parked_move_deadline:
                return
            warning(f"[{self.config.module_name}] Sending parked move at {speed_mm_s:.1f} mm/s, above its "
                    f"{self._parked_move_cutover_mm_s:.1f} mm/s cutover; the velocity change may spike torque")
        action, self._parked_move = self._parked_move, None
        self._send_move(ensure(action), halted=True)

    def _send_move(self, action: _EZMotionActionSetPosition, halted: bool = False) -> None:
        """Send a move to the lift motor, latched under the halt if one is set (`halted`, or a halt sent since the last
        move): the lift motor then never resumes the move the halt interrupted, and a lost write leaves it halted."""
        self._last_position, self._last_velocity = action.position, action.velocity
        self.lift_motor.go_to(action.position, velocity=action.velocity, halted=halted or self._halt_latched)
        self._halt_latched = False
        self._on_motion_sent(halting=False)
        self._last_move = action
        telemetry = self.lift_motor.telemetry
        position = telemetry.get_latest_value(EZMOTION_PARAMS.POSITION, default=None)
        velocity = telemetry.get_latest_value(EZMOTION_PARAMS.VELOCITY, default=0)
        self._move_from = None if position is None else (
            self.config.max_height_mm - abs(self._encoder_counts_to_distance(position)),
            self._encoder_counts_to_velocity(velocity))

    def _send_halt(self) -> None:
        """Halt the lift motor on its deceleration ramp; the next move is latched under the halt."""
        self.lift_motor.halt()
        self._halt_latched = True
        self._on_motion_sent(halting=True)
        self._move_from = None

    def _check_direction(self) -> None:
        """Halt a move going away from its target further than braking from its speed when it was sent takes, plus
        `_WRONG_WAY_MM`: whatever the cause (the lift motor once resumed a halted move down instead of going up)."""
        if self._move_from is None or self._last_move is None:
            return
        start, speed = self._move_from
        position = self.lift_motor.telemetry.get_latest_value(EZMOTION_PARAMS.POSITION, default=None)
        if position is None:
            return
        height = self.config.max_height_mm - abs(self._encoder_counts_to_distance(position))
        target = self.config.max_height_mm - abs(self._encoder_counts_to_distance(self._last_move.position))
        if abs(target - start) < 1.0:
            return  # a move to where it stands: no direction
        up = target > start
        away_speed = max(0.0, -speed if up else speed)  # how fast it was already going the other way
        allowed = _WRONG_WAY_MM + self._stopping_mm(away_speed)
        away = (start - height) if up else (height - start)
        if away <= allowed:
            return
        self._parked_move = None
        self._send_halt()
        self.stopped_by = (f"the lift moved {away:.0f} mm {'down' if up else 'up'} while sent {'up' if up else 'down'} "
                           f"to {target:.0f} mm: halted")
        error(f"[{self.config.module_name}] {self.stopped_by} (from {start:.0f} mm at {speed:+.0f} mm/s)")
        log_event("wrong_way", self.config.module_name, height_mm=float(height), target_mm=float(target),
                  from_mm=float(start), speed_mm_s=float(speed))

    def _on_motion_sent(self, halting: bool) -> None:
        """Start tracking a move or halt that was just sent to the lift motor."""
        self._moving, self._halting, self._holding = True, halting, False
        if halting:
            self._last_move = None
        self._motion_sent_at = currtime()
        self._target_reached_cleared = False

    def _check_torque_limit(self) -> bool:
        """De-energize the lift motor if the torque read this step exceeds `torque_limit`. Returns whether it tripped.

        The move or halt in progress ends without reaching its target, so `wait_for_target_reached()` keeps waiting.
        """
        torque = self.lift_motor.telemetry.get_latest_value(EZMOTION_PARAMS.TORQUE, default=0)
        if abs(torque) <= self.config.torque_limit:
            return False

        self.lift_motor.disable()
        self._moving, self._halting, self._holding = False, False, False
        self._parked_move, self._last_move = None, None
        position = self.lift_motor.telemetry.get_latest_value(EZMOTION_PARAMS.POSITION, default=0)
        height = self.config.max_height_mm - abs(self._encoder_counts_to_distance(position))
        error(f"[{self.config.module_name}] Torque {torque} per-mil exceeded the {self.config.torque_limit} per-mil "
              f"limit at {height:.2f} mm, lift motor de-energized")
        log_event("torque_trip", self.config.module_name, torque=int(torque), limit=self.config.torque_limit,
                  height_mm=float(height))
        return True
    
    def _check_target_reached(self) -> None:
        """Check the lift motor state read this step for the current move or halt reaching its target.

        Both complete when a status word read after they were sent reports target reached (bit 10). In profile
        position mode the lift motor sets it once a move settles in its position window, or once a halt brings the
        velocity to 0. The bit must first read cleared, since the first reads after a move can still report the
        previous target; a move or halt that never clears it completes after `_STALE_TARGET_REACHED_S`.
        """
        status_ts = self.lift_motor.telemetry.get_latest_timestamp(EZMOTION_PARAMS.STATUS_WORD, default=0.0)
        if status_ts <= self._motion_sent_at:
            return

        if not int(self.lift_motor.telemetry.get_latest_value(EZMOTION_PARAMS.STATUS_WORD)) & (1 << 10):
            self._target_reached_cleared = True
        elif self._target_reached_cleared or currtime() - self._motion_sent_at >= _STALE_TARGET_REACHED_S:
            self._on_target_reached()
    
    def _on_target_reached(self) -> None:
        """Finish the current move or halt, de-energizing the lift motor if `shutdown_on_target_reached` is set.

        A halt is not de-energized directly: the lift motor only engages its brake before turning off when it is
        disabled at a reached target, and after a halt it turns off at once and the load drops until the brake engages.
        So a halt is replaced with a move to the current position, which de-energizes like any other move once it is
        reached.
        """
        kind = "Halt" if self._halting else "Move"
        # Not _pos_to_height(), which raises if the position settles just outside the height range
        position = self.lift_motor.telemetry.get_latest_value(EZMOTION_PARAMS.POSITION)
        height = self.config.max_height_mm - abs(self._encoder_counts_to_distance(position))

        if self._halting and self.config.shutdown_on_target_reached:
            self.lift_motor.hold_position()
            self._on_motion_sent(halting=False)
            self._last_move, self._holding = None, True
            debug(f"[{self.config.module_name}] Halt reached target at {height:.2f} mm, holding before de-energizing")
            return

        self._moving, self._halting, self._holding = False, False, False
        if self.config.shutdown_on_target_reached:
            self.lift_motor.disable()
        self._target_reached_event.set()
        debug(f"[{self.config.module_name}] {kind} reached target at {height:.2f} mm"
              f"{', lift motor de-energized' if self.config.shutdown_on_target_reached else ''}")
        log_event("reached", self.config.module_name, what=kind.lower(), height_mm=float(height),
                  de_energized=self.config.shutdown_on_target_reached)
    
    def wait_for_target_reached(self, timeout: Optional[float] = None) -> bool:
        """Block until the last move, halt or recalibration has finished: reached its target and, with
        `shutdown_on_target_reached`, de-energized the lift motor.

        A move that stops short of its target never finishes, so pass a timeout unless blocking indefinitely is fine.

        Args:
            timeout: The maximum number of seconds to wait. None waits indefinitely.

        Returns:
            True if it finished, False if the timeout passed first.
        """
        return self._target_reached_event.wait(timeout)
    
    def await_action_complete(self) -> None:
        """Waits until the last sent action is complete."""
        exponential_timeout(self.lift_motor.config.homing_timeout, await_true=lambda: self._next_action is None)
    
    def recalibrate(self, load_calibration: Optional[bool] = None, block: bool = False) -> None:
        """Recalibrate the lift motor.
        
        Args:
            load_calibration: If True, load calibration state from file instead of recalibrating. None uses the lift 
                motor's ``config.load_calibration``.
            block: Whether to block the current thread until the recalibration is complete.
        """
        if not self.lift_motor.config.requires_calibration:
            return
        
        load_calibration = self.lift_motor.config.load_calibration if load_calibration is None else load_calibration
        if load_calibration:
            raise NotImplementedError("Automatic recalibration is not implemented yet")
        
        self._set_action(_EZMotionActionRecalibrate())
        log_event("recalibrate", self.config.module_name)
        if block:
            self.await_action_complete()
    
    def disable(self) -> None:
        """Disable the torso system."""
        self.lift_motor.disable()
        self._enabled = False
    
    def enable(self) -> None:
        """Enable the torso system. A halt sent before stays set: clearing it would resume the halted move."""
        self._last_move = None
        if self._halt_latched:
            self.lift_motor.halt()  # enables with the halt bit set
        else:
            self.lift_motor.enable()
        self._enabled = True
    
    def go_to(self, height: float, velocity: Optional[float] = None, block: bool = False, wait: bool = False) -> None:
        """Go to the given height from the bottom of the torso.
        
        Args:
            height: The height to go to in millimeters from the bottom of the torso.
            velocity: The velocity to go to in millimeters per second. If None, the velocity will be set to the target
                velocity of the lift motor. Must be positive and at most `max_velocity_mm_s`.
            block: Whether to block the current thread until the action has been confirmed sent/received by the motor.
            wait: Whether to wait until the lift is there (within 5 mm). It stops the lift if it stalls or on any
                exception while waiting (Ctrl+C included).

        Raises:
            CollisionError: (a ValueError) A guard (`set_guard()`) refuses going down from here.
            TimeoutError: (with `wait`) The lift made no progress toward the height for 3 s. It is stopped there.
        """
        if not self._enabled:
            raise RuntimeError("Torso system is not enabled")
        if velocity is not None and velocity < 0:
            raise ValueError("Velocity must be positive")
        if velocity is not None and velocity > self.max_velocity_mm_s:
            raise ValueError(f"Velocity {velocity} mm/s exceeds the max torso velocity of "
                             f"{math.floor(self.max_velocity_mm_s * 100) / 100} mm/s")
        
        guards = self._guards
        if guards:
            self.stopped_by = None
            height_now = self.read_state().height
            floor = self._guarded(guards, height_now, height) if height < height_now else None
            if floor is not None:  # down, past where a guard allows: to there, so the lift motor stops on it
                if floor[0] >= height_now - 1.0:
                    self.stopped_by = str(floor[1])
                    raise floor[1]
                height = floor[0]
                self.stopped_by = f"stops at {height:.0f} mm: {floor[1]}"
        velocity_counts = self._distance_to_encoder_counts(velocity) if velocity is not None else None
        self._set_action(_EZMotionActionSetPosition(position=self._height_to_pos(height), velocity=velocity_counts))
        log_event("move", self.config.module_name, height_mm=float(height),
                  velocity_mm_s=None if velocity is None else float(velocity))
        if block:
            self.await_action_complete()
        if wait:
            self._wait_until_at(height)

    def _wait_until_at(self, height: float) -> None:
        """Wait until the lift is within _ARRIVED_MM of `height`, stopping it if it stalls or on any exception."""
        gap = abs(height - self.read_state().height)
        closest, progress_at = gap, time.monotonic()
        try:
            while gap > _ARRIVED_MM:
                if gap < closest - _PROGRESS_MM:
                    closest, progress_at = gap, time.monotonic()
                elif time.monotonic() - progress_at > _STALL_S:
                    raise TimeoutError(f"{self.config.module_name} stopped {gap:.0f} mm short of {height:.0f} mm")
                time.sleep(_WAIT_POLL_S)
                gap = abs(height - self.read_state().height)
        except BaseException:
            self.update_velocity(0.0)
            raise
    
    def update_velocity(self, velocity: float, block: bool = False) -> None:
        """Update the velocity of the torso system.

        Args:
            velocity: The velocity to update the torso system to in millimeters per second. Set negative to move torso
                down, positive to move up. The magnitude must be at most `max_velocity_mm_s`.
            block: Whether to block the current thread until the action has been confirmed sent/received by the motor.
        """
        if velocity == 0 and self.config.decelerate_on_zero_velocity:
            self._halt(block=block)
            return
        
        height = self.config.max_height_mm if velocity >= 0 else self.config.min_height_mm
        log_event("velocity", self.config.module_name, mm_s=float(velocity))
        self.go_to(height, velocity=abs(velocity), block=block)
    
    def _halt(self, block: bool = False) -> None:
        """Stop the torso on the lift motor's deceleration ramp and hold the current height.

        The motor stays enabled. The next `go_to()` or `update_velocity()` replaces the halt. Does nothing if the torso
        system is disabled, since the lift motor is not moving and halting would enable it.

        Args:
            block: Whether to block the current thread until the action has been confirmed sent/received by the motor.
        """
        if not self._enabled:
            return
        
        self._set_action(_EZMotionActionHalt())
        log_event("halt", self.config.module_name)
        if block:
            self.await_action_complete()
    
    def get_height(self, fresh_value: bool = False) -> float:
        """Get the current height from the bottom of the torso.
        
        Args:
            fresh_value: Whether to get the current height from the motor's telemetry. If False, the last value read
                from the motor will be returned (which should update at ~self.config.operating_frequency Hz)
        """
        if fresh_value:
            self._set_action(_EZMotionActionReadParam(param=EZMOTION_PARAMS.POSITION))
            self.await_action_complete()
        return self._pos_to_height(int(self.lift_motor.telemetry[EZMOTION_PARAMS.POSITION, -1]))
    
    def get_velocity(self) -> float:
        """Get the current velocity of the torso in millimeters per second."""
        self._set_action(_EZMotionActionReadParam(param=EZMOTION_PARAMS.VELOCITY))
        self.await_action_complete()
        return self._encoder_counts_to_velocity(
            self.lift_motor.telemetry.get_latest_value(EZMOTION_PARAMS.VELOCITY)
        )
    
    @property
    def max_velocity_mm_s(self) -> float:
        """The maximum torso velocity in millimeters per second, from the lift motor's max speed and reduction ratio."""
        return self._encoder_counts_to_distance(self.config.lift_motor.max_velocity)

    def _distance_to_encoder_counts(self, distance: float) -> int:
        """Convert a distance in millimeters to encoder counts of the lift motor."""
        return int(distance / _EZMOTION_DISTANCE_PER_ROTATION_MM * self.config.reduction_ratio * EC_PER_ROT)

    def _encoder_counts_to_distance(self, encoder_counts: Union[int, float]) -> float:
        """Convert a number of encoder counts to a distance in millimeters."""
        return encoder_counts / EC_PER_ROT / self.config.reduction_ratio * _EZMOTION_DISTANCE_PER_ROTATION_MM
    
    def _encoder_counts_to_velocity(self, encoder_counts_per_second: float) -> float:
        """Convert velocity from encoder counts per second to millimeters per second."""
        return self.config.direction * self._encoder_counts_to_distance(encoder_counts_per_second)
    
    def _accepted_height(self, height: float) -> float:
        """Clamp a height within `_HEIGHT_TOLERANCE_MM` of the range; raise ValueError further out."""
        low, high = self._height_range
        if low - _HEIGHT_TOLERANCE_MM <= height <= high + _HEIGHT_TOLERANCE_MM:
            return min(max(height, low), high)
        assert_in_range(height, self._height_range, obj_name="torso height position")

    def _height_to_pos(self, height: float) -> int:
        """Convert a height from the bottom of the torso to a target position in encoder counts."""
        height = self._accepted_height(height)
        # Need total distance to travel. Must be - if direction is 1, + if direction is -1, so flip operands.
        return self.config.direction * self._distance_to_encoder_counts(height - self.config.max_height_mm)
    
    def _pos_to_height(self, position: int) -> float:
        """Convert a position in encoder counts to a height from the bottom of the torso."""
        # abs(EC) is distance in mm from max height, no matter the direction.
        height = self.config.max_height_mm - abs(self._encoder_counts_to_distance(position))
        return self._accepted_height(height)

    def _abort_impl(self) -> None:
        """Abort the torso system, halting and holding it first if `abort_stop_timeout_s` is set."""
        log_event("abort", self.config.module_name, enabled=self._enabled, moving=self._moving)
        # Stop the torso thread first so it cannot command the lift motor during the abort
        super()._abort_impl()
        if not self.lift_motor.config.has_endpoint():
            return

        if self.config.abort_stop_timeout_s > 0 and self._enabled and self.lift_motor.calibration_state.calibrated:
            with try_log_exc(msg=f"[{self.config.module_name}] Failed to stop torso before disabling, disabling now"):
                self._stop_and_hold_for_abort()
        self.disable()  # the system too: a command after the abort is refused, not queued for a thread that is gone

    def _stop_and_hold_for_abort(self) -> None:
        """Halt a moving torso and wait for it to stop, then hold it at a reached target so the disable that follows
        engages the brake before the lift motor turns off. Returns early, leaving the disable to the caller, on a lift
        motor fault or when the halt does not stop the torso within `abort_stop_timeout_s`."""
        if self.lift_motor.has_error():
            warning(f"[{self.config.module_name}] Lift motor faulted, disabling without stopping first")
            return

        if (self._moving and not self._holding) or self._parked_move is not None:
            self.lift_motor.halt()
            self._halt_latched = True
            deadline = currtime() + self.config.abort_stop_timeout_s
            while True:
                status = self.lift_motor.read_param_await(EZMOTION_PARAMS.STATUS_WORD)
                speed_mm_s = abs(self._encoder_counts_to_velocity(
                    self.lift_motor.read_param_await(EZMOTION_PARAMS.VELOCITY)
                ))
                if status & (1 << 10) and speed_mm_s < _ABORT_STOPPED_MM_S:
                    break
                if currtime() >= deadline:
                    warning(f"[{self.config.module_name}] Torso still moving at {speed_mm_s:.1f} mm/s after "
                            f"{self.config.abort_stop_timeout_s} s, disabling anyway")
                    return
                time.sleep(_ABORT_POLL_S)

        # Energized but not at an acknowledged target (bit 12), e.g. halted: the disable would turn the motor off before
        # the brake engages. Hold at current position first. A reached target, or a de-energized motor, needs nothing.
        status = self.lift_motor.read_param_await(EZMOTION_PARAMS.STATUS_WORD)
        if status & (1 << 2) and not status & (1 << 12):
            self.lift_motor.hold_position()
            deadline = currtime() + _ABORT_HOLD_TIMEOUT_S
            while not self.lift_motor.read_param_await(EZMOTION_PARAMS.STATUS_WORD) & (1 << 12):
                if currtime() >= deadline:
                    warning(f"[{self.config.module_name}] Hold not acknowledged within {_ABORT_HOLD_TIMEOUT_S} s, "
                            f"disabling anyway")
                    return
                time.sleep(_ABORT_POLL_S)
