from feathersdk.robot.actuators.robstride import RobstrideMotor, RobstrideMotorConfig, RobRunMode
from feathersdk.robot.actuator_systems import ActuatorSystemConfig, ActuatorSystem
from feathersdk.robot.actuator_systems.joints import JointDescription, check_against_urdf, feedback_frames
from feathersdk.robot.actuator_systems.trajectory import read_only
from feathersdk.configs import PyDField
from feathersdk.utils.feathertypes import (
    Dict, Optional, TYPE_CHECKING, Generic, NamedTuple, TypeVar, Tuple, Sequence, ensure,
)
from feathersdk.utils.math import clamp_to_range, value_in_range
from feathersdk.robot.actuators import calibrate_actuator_group, TActuator
from feathersdk.robot.urdf_loader import get_pin_joint_id
from feathersdk.logging import log_event
import math
import numpy as np

if TYPE_CHECKING:
    from feathersdk.robot.robot import FeatherRobot


class NeckSystemConfig(ActuatorSystemConfig):
    module_name: str = PyDField(default="neck_system")
    """The name of the neck system."""
    operating_frequency: float = PyDField(gt=0.0, lt=1000.0, default=30.0)
    """The operating frequency of the neck system."""
    multithreaded_calibration: bool = PyDField(default=True)
    """Whether to calibrate the motors in a multi-threaded manner."""


TNeckSystemConfig = TypeVar("TNeckSystemConfig", bound=NeckSystemConfig)
"""The type of the neck system configuration."""


class NeckSystem(ActuatorSystem[TNeckSystemConfig, TActuator], Generic[TNeckSystemConfig, TActuator]):
    """Controls the neck of the robot."""


class NeckState(NamedTuple):
    """The neck at one moment, per motor in `names` order (pitch, yaw). Returned by `read_state()`."""
    names: Tuple[str, ...]
    """Motor names (`neck_pitch`, `neck_yaw`)."""
    position: np.ndarray
    """Measured angle (rad), the motor's frame."""
    velocity: np.ndarray
    """Measured velocity (rad/s)."""
    effort: np.ndarray
    """Measured torque (N·m)."""
    stamp: np.ndarray
    """When each motor's feedback frame arrived at the Teensy (s, its clock, synced to the host's). Position, velocity
    and effort are from that frame. A stamp that stops changing means stale feedback."""
    target: np.ndarray
    """The last target angle given to each motor (rad), NaN before the first."""


class NeckDescription(NamedTuple):
    """What a ROS wrapper needs to know about the neck, as plain values. Returned by `describe()`."""
    name: str
    """The neck's `module_name`."""
    joints: Tuple[JointDescription, ...]
    """In motor order (pitch, yaw): the order of `read_state()`'s arrays."""
    rate: float
    """The neck loop's rate (Hz): feedback is requested at it; targets go to the motors as they are set."""


class TwoDOFRobstrideNeckSystemConfig(NeckSystemConfig):
    pitch_motor: RobstrideMotorConfig
    """The configuration for the pitch motor."""
    yaw_motor: RobstrideMotorConfig
    """The configuration for the yaw motor."""

    camera_offset: Tuple[float, float, float] = PyDField(default=(0.0138, 0.0899, 0.0451))
    """Camera (ZED 2i left lens) position relative to the pitch joint, in meters. Used by `look_at`. This is not in
       the URDF, which only models the camera body. The rest of the neck geometry is read from the URDF."""


def solve_look_at(
    target: Sequence[float], yaw_axis: Sequence[float], pitch_offset: Sequence[float], camera_offset: Sequence[float]
) -> Tuple[float, float]:
    """Return (yaw, pitch) in radians that point the camera through `target`. All positions are in the robot base frame
    (+X forward, +Y left, +Z up). Positive yaw turns left, positive pitch tilts down."""
    tx, ty, tz = (t - a for t, a in zip(target, yaw_axis))
    px, py, pz = pitch_offset
    _, cy, cz = camera_offset

    # Yaw so the target lines up with the camera, which sits (py + cy) to the side of the yaw axis
    r = math.hypot(tx, ty)
    if r <= abs(py + cy):
        raise ValueError(f"look_at target {tuple(target)} is too close to the neck")
    yaw = math.atan2(ty, tx) - math.asin((py + cy) / r)

    # Pitch so the target lines up with the camera, which sits cz above the pitch axis
    qx, qz = math.sqrt(r * r - (py + cy) ** 2) - px, tz - pz
    pitch = math.asin(cz / math.hypot(qx, qz)) - math.atan2(qz, qx)
    return math.remainder(yaw, math.tau), math.remainder(pitch, math.tau)


class TwoDOFRobstrideNeckSystem(NeckSystem[TwoDOFRobstrideNeckSystemConfig, RobstrideMotor]):
    """Controls the neck of the robot using two Robstride motors (pitch and yaw)."""

    def __init__(self, config: TwoDOFRobstrideNeckSystemConfig, robot: 'FeatherRobot'):
        super().__init__(config, robot)
        self.yaw_motor = RobstrideMotor(self.config.yaw_motor, self)
        """The yaw motor."""
        self.pitch_motor = RobstrideMotor(self.config.pitch_motor, self)
        """The pitch motor."""
        self.motor_order = [self.pitch_motor, self.yaw_motor]
        """The motors in the order `read_state()` and `describe()` report them."""
        # Per-motor values from the (frozen) config, computed once for set_targets() (called per streamed target).
        self._motor_names = [m.name for m in self.motor_order]
        self._index = {name: i for i, name in enumerate(self._motor_names)}
        self._angle_low = read_only([m.config.angle_range[0] for m in self.motor_order])
        self._angle_high = read_only([m.config.angle_range[1] for m in self.motor_order])
        self._max_velocity = read_only([m.config.max_velocity for m in self.motor_order])
        self._targets = np.full(len(self.motor_order), np.nan)  # the last target per motor; go_to() writes it

        # Neck geometry for `look_at`, in meters. Set from the URDF in _initialize()
        self._yaw_axis_xyz: Optional[Tuple[float, float, float]] = None
        """Yaw joint position in the robot base frame."""
        self._pitch_axis_offset: Optional[Tuple[float, float, float]] = None
        """Pitch joint position relative to the yaw joint."""

    def _load_urdf_geometry(self) -> None:
        """Reads the neck joint positions used by `look_at` from the URDF, if one is loaded."""
        urdf_loader = ensure(self.robot).urdf_loader
        if urdf_loader is None:
            return

        pin, model = urdf_loader.pin_import, urdf_loader.pin_model
        data = model.createData()
        pin.forwardKinematics(model, data, pin.neutral(model))
        yaw_id = get_pin_joint_id(model, self.yaw_motor.config.urdf_joint_name)
        pitch_id = get_pin_joint_id(model, self.pitch_motor.config.urdf_joint_name)
        self._yaw_axis_xyz = tuple(data.oMi[yaw_id].translation)
        self._pitch_axis_offset = tuple(model.jointPlacements[pitch_id].translation)

    def _initialize(self, load_calibration: Optional[bool] = None) -> None:
        """Initialize the neck platform. Called after all motors have been initialized."""
        try:
            self._load_urdf_geometry()
            self.recalibrate(load_calibration=load_calibration)

            # Write default parameters and enable the motor
            for motor in [self.yaw_motor, self.pitch_motor]:
                motor.set_run_mode(RobRunMode.POSITION_PP, block=True, check=True)
                motor.enable(block=True)
                motor.set_target_velocity(motor.config.max_velocity, block=True, check=True)
                motor.set_target_acceleration(motor.config.max_acceleration, block=True, check=True)
                motor.set_target_position(0, block=True, check=True)
            
            self._start_thread()
        except:  # Disable all motors if initialization fails.
            for motor in [self.yaw_motor, self.pitch_motor]:
                motor.disable()
            raise
    
    def _thread_step(self) -> None:
        """Thread step function for the neck system."""
        for motor in [self.yaw_motor, self.pitch_motor]:
            if motor.initialized:
                motor.trigger_feedback_frame(update_hz=self.config.operating_frequency, block=False)
                motor.check_warn_on_feedback_delay(motor.PARAMS.FB_MODE)
    
    def recalibrate(self, load_calibration: Optional[bool] = None) -> None:
        """Recalibrate the neck system.
        
        Args:
            load_calibration: If True, load calibration state from file instead of recalibrating. None uses each motor's
                ``config.load_calibration``.
        """
        calibrate_actuator_group(
            [self.pitch_motor, self.yaw_motor], 
            load_calibration=load_calibration, 
            multithreaded=self.config.multithreaded_calibration,
        )

    def go_to(self, pitch: Optional[float] = None, yaw: Optional[float] = None, block: bool = False) -> None:
        """Set target position for neck movement.
        
        Args:
            pitch (float, optional): Target pitch angle in radians. None to keep current target.
            yaw (float, optional): Target yaw angle in radians. None to keep current target.
            block (bool, optional): If True, block until the response is received and we've checked that the value
                was successfully written. (Not waiting until the target position is reached)
        """
        self._check_thread()
        if pitch is not None and not math.isnan(pitch):
            self.pitch_motor.set_target_position(pitch, block=block, check=block)
            self._targets[0] = pitch
        if yaw is not None and not math.isnan(yaw):
            self.yaw_motor.set_target_position(yaw, block=block, check=block)
            self._targets[1] = yaw
        log_event("go_to", self.config.module_name, pitch=pitch, yaw=yaw)

    def set_targets(
        self, positions: Optional[Dict[str, float]] = None, max_velocities: Optional[Dict[str, float]] = None,
    ) -> None:
        """Target angles and the speeds to reach them, per motor name, sent to the motors at once (the motor's own
        profile moves there; nothing is planned here). Partial dicts change only the motors named. For streaming from
        a controller: nothing blocks, and a bad value changes nothing.

        Raises:
            ValueError: A key is not a neck motor; a position is outside the motor's commandable range (its
                `angle_range` overlapping the calibrated one) or NaN; a speed is not in (0, `max_velocity`].
        """
        self._check_thread()
        # Every value is checked before anything is sent, with "within" comparisons so NaN fails.
        resolved_positions, resolved_speeds = [], []
        for values, store in ((positions, resolved_positions), (max_velocities, resolved_speeds)):
            if values is None:
                continue
            for key, value in values.items():
                i = self._index.get(key)
                if i is None:
                    raise ValueError(f"Not a motor of {self.config.module_name}: {key!r}; motors {self._motor_names}")
                store.append((i, float(value)))
        for i, value in resolved_positions:
            low, high = self.motor_order[i].command_angle_range()
            low, high = max(low, self._angle_low[i]), min(high, self._angle_high[i])
            if not low <= value <= high:
                raise ValueError(f"{self._motor_names[i]}: position {value} not within [{low:.4f}, {high:.4f}] rad")
        for i, value in resolved_speeds:
            if not 0.0 < value <= self._max_velocity[i]:
                raise ValueError(f"{self._motor_names[i]}: speed {value} not in (0, {self._max_velocity[i]}] rad/s")
        for i, value in resolved_speeds:  # the profile's speed cap (VEL_MAX in position mode)
            self.motor_order[i].set_target_velocity(value)
        for i, value in resolved_positions:
            self.motor_order[i].set_target_position(value)
            self._targets[i] = value

    def read_state(self) -> NeckState:
        """A snapshot of the neck: each motor's measured position, velocity and effort from one feedback frame, that
        frame's time, and the last target. Nothing blocks: what a ros2_control hardware interface reads each cycle.

        Raises:
            ValueError: A motor has sent no feedback yet (the neck isn't initialized).
            RuntimeError: A motor's feedback changed on every attempt to read it (telemetry is broken).
        """
        position, velocity, effort, stamp = feedback_frames(self.motor_order)
        return NeckState(tuple(self._motor_names), position, velocity, effort, stamp, self._targets.copy())

    def describe(self) -> NeckDescription:
        """The neck's joints (motor names, their URDF joints and signs), their limits and the loop rate, as plain
        values: what a ROS wrapper generates its config from. Each commandable range, in the URDF's frame, is checked
        against the loaded URDF.

        Raises:
            ValueError: No URDF is loaded, a joint isn't in it, or a commandable range reaches outside the URDF's.
        """
        loader = ensure(self.robot).urdf_loader
        if loader is None:
            raise ValueError(f"{self.config.module_name}: describe() checks the joints against the URDF, but none is "
                             "loaded (initialize the robot with a urdf_path)")
        joints = []
        for i, m in enumerate(self.motor_order):
            low, high = m.command_angle_range()  # calibrated, else configured
            low, high = max(low, self._angle_low[i]), min(high, self._angle_high[i])
            # Python floats throughout: ROS parameters typed double reject ints and numpy scalars.
            joints.append(JointDescription(
                m.name, m.config.urdf_joint_name, m.config.urdf_direction, (float(low), float(high)),
                float(m.config.max_velocity), float(m.config.max_acceleration), float(m.config.max_jerk),
                float(m.config.max_torque),
            ))
        check_against_urdf(self.config.module_name, joints, loader.pin_model)
        return NeckDescription(self.config.module_name, tuple(joints), float(self.config.operating_frequency))

    def look_at(self, target: Sequence[float], block: bool = False) -> bool:
        """Point the camera at `target`, an (x, y, z) point in meters in the robot base frame (origin on the floor
        between the wheels, +X forward, +Y left, +Z up). Motors are clamped to their angle ranges.

        Returns:
            True if the target is within the neck's range, False if a motor was clamped (camera not on target).
        """
        if self._yaw_axis_xyz is None or self._pitch_axis_offset is None:
            raise ValueError("look_at requires the robot URDF to be loaded")

        yaw, pitch = solve_look_at(target, self._yaw_axis_xyz, self._pitch_axis_offset, self.config.camera_offset)
        yaw = -yaw  # solve_look_at yaw is positive left, but the yaw motor is positive right
        pitch_range, yaw_range = self.config.pitch_motor.angle_range, self.config.yaw_motor.angle_range
        self.go_to(
            pitch=float(clamp_to_range(pitch, pitch_range)), yaw=float(clamp_to_range(yaw, yaw_range)), block=block
        )
        in_range = value_in_range(pitch, pitch_range) and value_in_range(yaw, yaw_range)
        log_event("look_at", self.config.module_name, target=list(target), in_range=in_range)  # go_to() logs the angles
        return in_range

    def set_speed(
        self, pitch_speed: Optional[float] = None, yaw_speed: Optional[float] = None, block: bool = False
    ) -> None:
        """Set the speed for the neck motors.
        
        Args:
            pitch_speed (float, optional): The target speed for the pitch motor in radians per second. None to keep 
                current speed.
            yaw_speed (float, optional): The target speed for the yaw motor in radians per second. None to keep 
                current speed.
            block (bool, optional): If True, block until the response is received and we've checked that the value
                was successfully written. (Not waiting until the target speed is reached)
        """
        self._check_thread()
        if pitch_speed is not None and not math.isnan(pitch_speed):
            self.pitch_motor.set_target_velocity(pitch_speed, block=block, check=block)
        if yaw_speed is not None and not math.isnan(yaw_speed):
            self.yaw_motor.set_target_velocity(yaw_speed, block=block, check=block)
        log_event("speed", self.config.module_name, pitch=pitch_speed, yaw=yaw_speed)

    def _abort_impl(self) -> None:
        """Abort the neck system."""
        # Attempt to move the pitch motor gently to the bottom so it doesn't slam down
        try:
            # Make sure the pitch motor has been fully initialized before attempting to move it gently
            run_mode = self.pitch_motor.telemetry.get_latest_value(self.pitch_motor.PARAMS.RUN_MODE, default=None)
            if self.pitch_motor.config.has_endpoint() and run_mode == RobRunMode.POSITION_PP:
                self.pitch_motor.set_target_velocity(2.0, block=True, check=True)
                self.pitch_motor.set_target_position(0, block=True, check=True)
                self.pitch_motor.await_param(self.pitch_motor.PARAMS.FB_POSITION, 0, 0.01, timeout=2.0)
        finally:
            if self.pitch_motor.config.has_endpoint():
                self.pitch_motor.disable()
            if self.yaw_motor.config.has_endpoint():
                self.yaw_motor.disable()
            super()._abort_impl()
