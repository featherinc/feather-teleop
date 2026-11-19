from feathersdk.configs import PyDField
from feathersdk.robot.actuator_systems import ActuatorSystem, ActuatorSystemConfig
from feathersdk.robot.actuators import TActuator, calibrate_actuator_group
from feathersdk.robot.actuators.damiao import I2RTGripper, I2RTGripperConfig
from feathersdk.robot.actuators.realhand import RealHand, RealHandConfig
from feathersdk.utils import try_log_exc
from feathersdk.utils.feathertypes import Dict, Generic, Optional, Tuple, TypeVar, Union, TYPE_CHECKING


if TYPE_CHECKING:
    from feathersdk.robot.robot import FeatherRobot


_OpenPct = Union[float, Tuple[float, float, float, float, float, float]]
"""One open percent for every finger, or six percents in RealHand finger order."""

_POLL_PARAMS = (
    RealHand.PARAMS.POSITION,
    RealHand.PARAMS.SPEED,
    RealHand.PARAMS.NORMAL_FORCE,
    RealHand.PARAMS.TEMPERATURE,
    RealHand.PARAMS.ERRORS,
)
"""Parameters read from each hand on every thread tick."""


class ManipulationSystemConfig(ActuatorSystemConfig):
    """Configuration for the manipulation system."""
    module_name: str = PyDField(default="manipulation_system")
    """The name of the actuator system."""
    actuators_share_same_endpoint: bool = PyDField(default=False)
    """The two hands are on different arm CAN buses."""


TManipulationSystemConfig = TypeVar("TManipulationSystemConfig", bound=ManipulationSystemConfig)
"""Type variable for the manipulation system configuration."""


class ManipulationSystem(
    ActuatorSystem[TManipulationSystemConfig, TActuator],
    Generic[TManipulationSystemConfig, TActuator],
):
    """Controls the robot's hands."""


class RealHandManipulationSystemConfig(ManipulationSystemConfig):
    """A RealHand on each arm. Leave a side None to omit that hand."""
    right_hand: Optional[RealHandConfig] = None
    """The right hand. Stock RealHand id is 0x27."""
    left_hand: Optional[RealHandConfig] = None
    """The left hand. Stock RealHand id is 0x28."""


class RealHandManipulationSystem(ManipulationSystem[RealHandManipulationSystemConfig, RealHand]):
    """RealHand hands. Each finger is commanded as a percent open."""

    def __init__(self, config: RealHandManipulationSystemConfig, robot: "FeatherRobot") -> None:
        super().__init__(config, robot)
        self.right_hand = RealHand(config.right_hand, self) if config.right_hand is not None else None
        """The right hand."""
        self.left_hand = RealHand(config.left_hand, self) if config.left_hand is not None else None
        """The left hand."""

    def _initialize(self, load_calibration: Optional[bool] = None) -> None:
        """The hand calibrates itself when it powers on."""
        self._start_thread()

    def _abort_impl(self) -> None:
        """Run each configured hand's abort action, then stop the poll thread."""
        for hand in (self.left_hand, self.right_hand):
            if hand is None:
                continue
            with try_log_exc(f"Error in on_abort for {hand.config.name}"):
                hand.on_abort()
        super()._abort_impl()

    def recalibrate(self, load_calibration: Optional[bool] = None) -> None:
        """The hand calibrates itself when it powers on."""
        raise RuntimeError("The Realhand motor recalibrates itself automatically")

    def _thread_step(self) -> None:
        """Read hand state at ``config.operating_frequency``."""
        for hand in (self.left_hand, self.right_hand):
            if hand is None or not hand.config.has_endpoint():
                continue
            version = int(hand.config.version)
            for param in _POLL_PARAMS:
                if param.available_on[version]:
                    hand.read_param(param, block=False)

    def get_open_pcts(self) -> Dict[str, Dict[str, float]]:
        """How open each finger is on each configured hand.

        The outer keys are ``"left"`` and ``"right"``. A side whose hand is ``None`` is omitted. Each inner dict maps
        a finger name to its open percent, 0 closed and 100 fully open.
        """
        return {
            side: hand.get_open_pcts()
            for side, hand in (("left", self.left_hand), ("right", self.right_hand))
            if hand is not None
        }

    def set_open_pcts(self, targets: Dict[str, _OpenPct], block: bool = False) -> None:
        """Set how open the fingers are. 0 is closed, 100 is fully open.

        ``targets`` maps ``"left"`` and ``"right"`` to either one percent for the whole hand, or six percents in finger
        order: thumb flexion, thumb abduction, index, middle, ring, pinky. A missing side is left alone.

        Args:
            targets: Open percent for each hand to command.
            block: Wait until each commanded hand echoes its new position.
        """
        unknown = [key for key in targets if key not in ("left", "right")]
        if unknown:
            raise ValueError(f"Unknown hands {unknown}. Use 'left' and 'right'")
        for side, hand in (("left", self.left_hand), ("right", self.right_hand)):
            if side not in targets:
                continue
            if hand is None:
                raise ValueError(f"No {side} hand is configured")
            hand.set_open_pcts(targets[side], block=block)

class I2RTGripperManipulationSystemConfig(ManipulationSystemConfig):
    """Configuration for a pair of I2RT grippers. Leave a side as None if that arm has no gripper."""
    left_gripper: Optional[I2RTGripperConfig] = PyDField(default=None)
    """The configuration for the left gripper."""
    right_gripper: Optional[I2RTGripperConfig] = PyDField(default=None)
    """The configuration for the right gripper."""
    actuators_share_same_endpoint: bool = PyDField(default=False)
    """Each gripper sits on its own arm's CAN bus."""
    operating_frequency: float = PyDField(gt=5.0, lt=1000.0, default=50.0)
    """How often each gripper's force limiter runs and its command is re-sent, in Hz. Must stay well above 1 / the
       motor's watchdog timeout. The limiter's timings are in seconds, so they hold at any rate."""


class I2RTGripperManipulationSystem(ManipulationSystem[I2RTGripperManipulationSystemConfig, I2RTGripper]):
    """Controls the I2RT grippers.

    The grippers calibrate (sweep open and closed) at every startup. Afterwards a background thread runs each
    gripper's force limiter and re-sends its command, because the motors disable themselves if commands stop.
    """

    left_gripper: Optional[I2RTGripper]
    """The left gripper, or None if not configured."""
    right_gripper: Optional[I2RTGripper]
    """The right gripper, or None if not configured."""

    def __init__(self, config: I2RTGripperManipulationSystemConfig, robot: 'FeatherRobot'):
        super().__init__(config, robot)
        self.left_gripper = None if config.left_gripper is None else I2RTGripper(config.left_gripper, self)
        self.right_gripper = None if config.right_gripper is None else I2RTGripper(config.right_gripper, self)

    def _initialize(self, load_calibration: Optional[bool] = None) -> None:
        """Start re-sending commands, then calibrate the grippers, leaving them open.

        The thread must run before calibrating. The grippers calibrate in parallel, and one that finishes first only gets
        its open command re-sent by the thread. Without it, that motor's watchdog trips while the other still calibrates.
        The thread skips each gripper until it is calibrated.
        """
        try:
            self._start_thread()
            self.recalibrate(load_calibration=load_calibration)
        except:  # Disable all grippers if initialization fails.
            for gripper in self.actuators_by_name.values():
                gripper.disable()
            raise

    def recalibrate(self, load_calibration: Optional[bool] = None) -> None:
        """Sweep every gripper open and closed to find its end stops, then open it. The jaws move.

        Args:
            load_calibration: Unused. The grippers always sweep; there is no saved calibration to load.
        """
        grippers = list(self.actuators_by_name.values())
        if grippers:  # both sides may be None
            calibrate_actuator_group(grippers, load_calibration=False)

    def _thread_step(self) -> None:
        """Run each gripper's force limiter and re-send its command, so the motors' watchdogs never disable them."""
        for gripper in self.actuators_by_name.values():
            gripper._send_current_command()  # a gripper's own errors fail that gripper; they never raise here
            if gripper._sending:  # a faulted, disabled or uncalibrated gripper sends nothing, so gets no feedback
                gripper.check_warn_on_feedback_delay(gripper.PARAMS.FB_STATUS)

    def _abort_impl(self) -> None:
        """Stop re-sending commands, then release and disable the grippers so nothing stays squeezed."""
        super()._abort_impl()
        for gripper in self.actuators_by_name.values():
            if gripper.config.has_endpoint():
                with try_log_exc(f"Failed to release {gripper.config.name}"):
                    gripper._mit_command()
                    gripper.disable()
