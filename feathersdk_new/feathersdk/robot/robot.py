import atexit, signal, os
from feathersdk.utils import format_exc, no_dupe_dict, try_log_exc, Abortable, suppress, ensure
from feathersdk.utils.time import currtime
from feathersdk.utils.constants import _build_directories, get_robot_config_path, get_robot_urdf_path
from feathersdk.utils.featherthreads import _FeatherExecutable
from feathersdk.configs import Configurable, create_from_config, PyDBaseModel, PyDField, load_configs
from feathersdk.logging import error, debug, info, flush_logs, LoggingConfig, init_logging, log_json, NOTICE
from feathersdk.robot.instance_lock import get_robot_instance_lock, release_robot_instance_lock
from feathersdk.utils.feathertypes import (
    Optional, Iterable, Union, Path, TYPE_CHECKING, Dict, Tuple, List, Any
)
from feathersdk.robot.robot_module import RobotModule
from feathersdk.robot.services import DataMonitorService, DataMonitorServiceConfig, perftime, perftime_insert, \
    BatteryService, BatteryServiceConfig, ZedCameraService, ZedCameraServiceConfig, \
    WheelOdometryService, WheelOdometryServiceConfig
from feathersdk.robot.actuator_systems import TwoDOFRobstrideNeckSystem, TwoDOFRobstrideNeckSystemConfig, \
    LinearEZMotionTorsoSystem, LinearEZMotionTorsoSystemConfig, ThreeWheelHolonomicRobstrideNavigationSystem, \
    ThreeWheelHolonomicRobstrideNavigationSystemConfig, ActuatorSystem, ArmsSystem, \
    ArmsSystemConfig, ManipulationSystem, ManipulationSystemConfig, ActuatorSystemConfig
from feathersdk.robot.robot_module import RobotModuleConfig
from feathersdk.robot.actuators import Actuator, ActuatorConfig, RobstrideMotor
from feathersdk.robot.comms import CommsManager, CommsManagerConfig
from feathersdk.robot.urdf_loader import URDFLoader
from feathersdk.robot.actuators.telemetry import FeatherTelemetry, _TelemetryParameters
from feathersdk.robot.actuators.telemetry import compile_kernels as compile_telemetry_kernels


_ROSNodeType = Any
"""Type hint for the ROS node type. Currently just Any, might try to make more specific later."""


if TYPE_CHECKING:
    from feathersdk.ipc.client import FeatherClient


class ActuatorNotFoundError(Exception):
    """Raised when accessing a non-existent actuator, or when an actuator cannot be found on any CAN endpoint."""


class FeatherRobotConfig(PyDBaseModel):
    """Configuration for a full FeatherRobot."""
    # FeatherRobot configs
    version: str
    """The version of the Feather Robot (different from the FeatherSDK version)."""
    set_atexit_handler: bool = PyDField(default=True)
    """Whether to set the atexit handler for the robot."""
    set_signal_handlers: bool = PyDField(default=True)
    """Whether to set the signal handlers for the robot."""
    kill_on_failed_abort: bool = PyDField(default=True)
    """Whether to kill the process if the robot fails to abort cleanly, to avoid hanging processes."""
    skip_find_actuators: bool = PyDField(default=False)
    """Whether to skip the finding of actuators. This is useful for debugging purposes, flashing the motor firmware,
       etc. The robot will not attempt to find any actuators or initialize them."""
    skip_initialization: bool = PyDField(default=False)
    """Whether to skip the initialization of the robot. This is useful for debugging purposes, flashing the motor
       firmware, etc. The robot will stop initialization after finding all motors, but not initializing them."""
    urdf_path: Optional[Path] = PyDField(default=None)
    """The path to the URDF model for the robot. None uses the installed URDF from ``get_robot_urdf_path()``."""

    # Infrastructure configs
    logging_config: Optional[LoggingConfig] = PyDField(default=None)
    """The configuration for logging."""
    comms_manager_config: Optional[CommsManagerConfig] = PyDField(default=None)
    """The configuration for the comms manager."""

    # Service configs
    data_monitor_service_config: Optional[DataMonitorServiceConfig] = PyDField(default=None)
    """The configuration for the data monitor service, or None to disable"""
    battery_service_config: Optional[BatteryServiceConfig] = PyDField(default=None)
    """The configuration for the battery service, or None to disable"""
    camera_service_config: Optional[ZedCameraServiceConfig] = PyDField(default=None)
    """The configuration for the camera service, or None to disable"""
    odometry_service_config: Optional[WheelOdometryServiceConfig] = PyDField(default=None)
    """Wheel-encoder odometry, or None to disable. Requires a navigation system."""

    # Actuator system configs
    neck_system_config: Optional[TwoDOFRobstrideNeckSystemConfig] = PyDField(default=None)
    """The configuration for the neck system, or None to disable"""
    torso_system_config: Optional[LinearEZMotionTorsoSystemConfig] = PyDField(default=None)
    """The configuration for the torso system, or None to disable"""
    navigation_system_config: Optional[ThreeWheelHolonomicRobstrideNavigationSystemConfig] = PyDField(default=None)
    """The configuration for the navigation system, or None to disable"""
    arms_system_config: Optional[ArmsSystemConfig] = PyDField(default=None)
    """The configuration for the arms system, or None to disable"""
    manipulation_system_config: Optional[ManipulationSystemConfig] = PyDField(default=None)
    """The configuration for the manipulation system, or None to disable"""

    # Custom actuator systems/modules
    custom_actuator_system_configs: List[ActuatorSystemConfig] = PyDField(default_factory=list)
    """Allows users to add custom actuator systems to the robot."""
    custom_robot_module_configs: List[RobotModuleConfig] = PyDField(default_factory=list)
    """Allows users to add custom robot modules to the robot."""

    def only(self, *systems: str) -> "FeatherRobotConfig":
        """This config with every system and service in `SYSTEMS` left out (None) except the ones named, so a robot
        started from it starts only those. The logging, comms and data monitor configs are kept.

        Raises:
            ValueError: A name is not in `SYSTEMS`.
        """
        unknown = set(systems) - set(SYSTEMS)
        if unknown:
            raise ValueError(f"No systems named {sorted(unknown)}; they are {sorted(SYSTEMS)}")
        return self.model_copy(update={field: None for name, field in SYSTEMS.items() if name not in systems})


SYSTEMS = {
    "arms": "arms_system_config", "grippers": "manipulation_system_config", "neck": "neck_system_config",
    "torso": "torso_system_config", "base": "navigation_system_config", "cameras": "camera_service_config",
    "odometry": "odometry_service_config", "battery": "battery_service_config",
}
"""The systems and services `FeatherRobotConfig.only()` can keep, by short name, and their config fields."""


_ActType = Actuator[ActuatorConfig, _TelemetryParameters]
"""Type of a generic base actuator with base configs/telemetry parameters."""

_ModuleType = Union[RobotModule[RobotModuleConfig], ActuatorSystem[ActuatorSystemConfig, _ActType]]
"""A robot module or an actuator system stored in ``FeatherRobot.modules``."""


class FeatherRobot(Abortable, Configurable[FeatherRobotConfig]):
    """A Feather robot.
    
    Parameters
    ----------
    config: FeatherRobotConfig
        The configuration for the robot. Will be deep-copied and frozen.
    _client: Optional["FeatherClient"] = None
        Do not set manually! Only used internally by the FeatherClient. Initializes the robot instance assuming it is
        just meant for data storage and API interfacing instead of controlling the physical robot.
    """
    
    def __init__(self, config: FeatherRobotConfig, _client: Optional["FeatherClient"] = None):
        start_time = currtime()

        self.comms_manager: Optional[CommsManager] = None
        """Handles lower-level communication within the robot."""
        self.modules: Dict[str, _ModuleType] = {}
        """Dictionary of module name to module."""
        self.actuator_systems: Dict[str, ActuatorSystem[ActuatorSystemConfig, _ActType]] = {}
        """Dictionary of actuator system name to actuator system."""
        self.actuators_by_name: Dict[str, _ActType] = {}
        """Dictionary of actuator name to actuator."""
        self.actuators_by_id: Dict[int, _ActType] = {}
        """Dictionary of actuator integer ID to actuator."""
        self.global_telemetry: Optional[FeatherTelemetry] = None
        """Stores all telemetry data for the robot."""

        self.data_monitor_service: Optional[DataMonitorService] = None
        """Manages the data monitoring and robot telemetry gathering, if using."""
        self.battery_service: Optional[BatteryService] = None
        """Manages the battery state and detects power events like hitting the estop, if using."""
        self.camera_service: Optional[ZedCameraService] = None
        """Manages the ZED camera, if using."""
        self.odometry_service: Optional[WheelOdometryService] = None
        """Dead-reckons base pose from wheel encoders, if using."""

        self.neck_system: Optional[TwoDOFRobstrideNeckSystem] = None
        """Manages the neck, if using."""
        self.torso_system: Optional[LinearEZMotionTorsoSystem] = None
        """Manages the torso lift, if using."""
        self.navigation_system: Optional[ThreeWheelHolonomicRobstrideNavigationSystem] = None
        """Manages the navigation, if using."""
        self.arms_system: Optional[ArmsSystem] = None
        """Manages the arms, if using."""
        self.manipulation_system: Optional[ManipulationSystem[ManipulationSystemConfig, _ActType]] = None
        """Manages the manipulation, if using."""

        self.custom_actuator_systems: List[ActuatorSystem[ActuatorSystemConfig, _ActType]] = []
        """Allows users to add custom actuator systems to the robot."""
        self.custom_robot_modules: List[RobotModule[RobotModuleConfig]] = []
        """Allows users to add custom robot modules to the robot."""

        self.urdf_loader: Optional[URDFLoader] = None
        """Loads the URDF model for the robot, if using."""

        self._client = _client
        self.is_initialized = False
        """True once ``FeatherRobot.__init__`` has finished. Feedback-delay checks stay off until then."""

        Abortable.__init__(self)

        # Actual initialization. See design doc for why we intialize in this order.
        try:
            _build_directories()

            if self.config.logging_config is not None:
                init_logging(self.config.logging_config)
            
            if self._client is None:
                get_robot_instance_lock(override_lock=False, strict=False)
            
            # Before the Teensy connection opens: a cold compile starves the comms thread, and the Teensy drops a
            # client it hears nothing from for 5 s (every actuator then "not found").
            compile_telemetry_kernels()
            self.comms_manager = CommsManager(ensure(self.config.comms_manager_config), self)
            
            urdf_path = self.config.urdf_path if self.config.urdf_path is not None else get_robot_urdf_path()
            self.urdf_loader = URDFLoader(urdf_path, self)
            self._create_services_and_actuator_systems()

            if self._client is None:
                if self.config.skip_find_actuators:
                    info(f"Skipping finding actuators")
                    return
                
                self._find_actuators(self.actuators_by_id.values())
                if self.battery_service is not None:
                    self.battery_service._find()

                if self.config.skip_initialization:
                    info(f"Skipping initialization of actuators and systems")
                    return
                
                self._init_actuators_and_modules()

                # Read the firmware versions of all actuators (that are available) and print them
                acts = self.actuators_by_id.values()
                log_json(NOTICE, "actuator_metadata", {
                    "robstride": {a.config.id: {
                        'firmware_version': a.get_firmware_version(),
                        'device_id': hex(a.get_device_id()),
                    } for a in acts if isinstance(a, RobstrideMotor)},
                })
            
            # Set the exit handlers only after everything has been initialized, otherwise they may capture signals
            # during critical initialization, and shutdown the comms manager, before returning. If this happens during
            # critical sections like the arms being calibrated, we won't be able to send any commands to the motors
            # to freeze them safely.
            self._set_exit_handlers()
            self.is_initialized = True
        except:
            log_json(NOTICE, "robot_init", {
                "config": self.config.model_dump_json(), 
                "success": False, 
                "error": format_exc()
            })
            error(f"Error initializing FeatherRobot:\n{format_exc()}")
            self.abort()
            raise
        
        log_json(NOTICE, "robot_init", {"config": self.config.model_dump_json(), "success": True, "error": None})
        info(f"FeatherRobot initialized successfully")
        perftime_insert("FeatherRobot.__init__-success", currtime() - start_time, currtime())

    @perftime(prefix="FeatherRobot")
    def _set_exit_handlers(self) -> None:
        """Set the exit handlers for the robot."""
        if self.config.set_atexit_handler:
            atexit.register(self.abort)
        if self.config.set_signal_handlers:
            for sig in [signal.SIGTERM, signal.SIGINT]:
                signal.signal(sig, lambda signum, frame: self.abort())
    
    @perftime(prefix="FeatherRobot")
    def _init_actuators_and_modules(self) -> None:
        """Initialize the actuators and modules after they have been created."""
        for actuator in self.actuators_by_id.values():
            actuator._on_endpoint_found()
        for system in self.actuator_systems.values():
            debug(f"Initializing {system.__class__.__name__}")
            system._initialize()
    
    @perftime(prefix="FeatherRobot")
    def _create_services_and_actuator_systems(self) -> None:
        """Create the services and actuator systems."""
        # Use different configs if we are a client talking to a server
        dms_config = self.config.data_monitor_service_config if self._client is None else None
        bs_config = self.config.battery_service_config if self._client is None else None
        cs_config = self.config.camera_service_config if self._client is None else None
        odom_config = self.config.odometry_service_config if self._client is None else None

        # Create the global telemetry manager before the actuators, so their telemetries can be added to it.
        self.global_telemetry = FeatherTelemetry(self)

        # Construct all the services and actuator systems
        self.data_monitor_service = create_from_config(DataMonitorService, dms_config, self)
        self.battery_service = create_from_config(BatteryService, bs_config, self)
        self.camera_service = create_from_config(ZedCameraService, cs_config, self)
        for config in self.config.custom_robot_module_configs:
            self.custom_robot_modules.append(ensure(create_from_config(RobotModule, config, self)))

        self.neck_system = create_from_config(TwoDOFRobstrideNeckSystem, self.config.neck_system_config, self)
        self.torso_system = create_from_config(LinearEZMotionTorsoSystem, self.config.torso_system_config, self)
        self.navigation_system = create_from_config(
            ThreeWheelHolonomicRobstrideNavigationSystem, self.config.navigation_system_config, self
        )

        # Wheel odometry needs `navigation_system` (already created above).
        self.odometry_service = create_from_config(WheelOdometryService, odom_config, self)
        self.arms_system = create_from_config(ArmsSystem, self.config.arms_system_config, self)
        self.manipulation_system = create_from_config(ManipulationSystem, self.config.manipulation_system_config, self)
        for config in self.config.custom_actuator_system_configs:
            self.custom_actuator_systems.append(ensure(create_from_config(ActuatorSystem, config, self)))

        # Add all modules, actuator systems, and actuators to the robot dictionaries
        modules = self.custom_robot_modules + self.custom_actuator_systems + \
            [m for m in self.__dict__.values() if isinstance(m, RobotModule)]
        self.modules = no_dupe_dict((v.config.module_name, v) for v in modules)

        act_systems = self.custom_actuator_systems + \
            [s for s in self.__dict__.values() if isinstance(s, ActuatorSystem)]
        self.actuator_systems = no_dupe_dict((v.config.module_name, v) for v in act_systems)

        self.actuators_by_id, self.actuators_by_name = self._check_actuators_unique()

        log_json(NOTICE, "actuator_id_to_name", {k: v.config.name for k, v in self.actuators_by_id.items()})

        # Initialize the global telemetry manager after the actuators have been created.
        self.global_telemetry.initialize()

    @perftime(prefix="FeatherRobot")
    def _check_actuators_unique(self) -> Tuple[Dict[int, _ActType], Dict[str, _ActType]]:
        """Check that the actuators are unique by their ID and name, and return a dictionary mapping ID to actuator."""
        names = [(a.name, a) for s in self.actuator_systems.values() for a in s.actuators_by_name.values()]
        ids = [(a.config.id, a) for s in self.actuator_systems.values() for a in s.actuators_by_name.values()]
        return no_dupe_dict(ids), no_dupe_dict(names)

    @perftime(prefix="FeatherRobot")
    def _find_actuators(self, actuators: Iterable[_ActType], allow_not_found: bool = False) -> None:
        """Find the actuators on the physical CAN interfaces.

        Args:
            actuators: The actuators to find.
            allow_not_found: If True, do not raise an error if an actuator is not found.
        """
        info(f"Searching for actuators...")

        acts = list(actuators)
        actuator_order = [a for a in acts if a.config.shares_endpoint_with is None] + \
            [a for a in acts if a.config.shares_endpoint_with is not None]

        missing_actuators = []
        for act in actuator_order:
            # If this actuator shares an endpoint with another actuator, attempt to copy that endpoint.
            shared = act.config.shares_endpoint_with
            if shared is not None:
                shared = [shared] if isinstance(shared, str) else shared
                shared_missing = False

                # Go through each shared actuator checking that it exists, and copying its endpoint if it does.
                for name in shared:
                    if name not in self.actuators_by_name:
                        debug(f"Actuator `{act.config.name}` shares endpoint with unknown act `{name}`, ignoring")
                        continue

                    other_actuator = self.actuators_by_name[name]

                    if other_actuator.config.has_endpoint():
                        act.config.set_endpoint(other_actuator.config.endpoint)
                    else:
                        error(f"Actuator `{act.config.name}` shares endpoint with `{name}`, but `{name}` "
                              f"does not have an endpoint set")
                        shared_missing = True
                    
                    break
                
                # If an existing shared actuator is missing, then we assert this actuator is also missing.
                if shared_missing:
                    missing_actuators.append(act.config.name)
                    continue

            # If the actuator has a defined endpoint, then assert it exists there
            if act.config.has_endpoint():
                try:
                    if not act._probe_endpoint(act.config.endpoint):
                        missing_actuators.append(act.config.name)
                except Exception as e:
                    raise RuntimeError(f"Could not find actuator {act.config.name} on expected {act.config.endpoint}")
                continue
            
            # Otherwise, try to find it on any of the CAN endpoints
            for ep in range(ensure(self.comms_manager).num_can_endpoints):
                found = False

                try:
                    found = act._probe_endpoint(ep)
                except Exception as e:
                    debug(f"Error probing endpoint {ep} for actuator {act.config.name}: {e}")
                finally:
                    act.config.clear_endpoint()

                if found:
                    act.config.set_endpoint(ep)

                    # Set all actuators in the system to the same endpoint if not set and using. We still check that
                    # we can find them later, but this makes it faster to find them all.
                    system = act.system
                    if system.config.actuators_share_same_endpoint:
                        for other_actuator in system.actuators_by_name.values():
                            if other_actuator.config.has_endpoint() and other_actuator.config.endpoint != ep:
                                raise ValueError(f"Actuators for system {system.config.module_name} must share the same"
                                                 f" endpoint: {other_actuator.config.endpoint} != {ep}")
                            other_actuator.config.set_endpoint(ep)

                            # Actuators already probed (and marked missing) before this bus was discovered need a
                            # re-probe here — they will not be visited again in this loop.
                            if other_actuator is not act and other_actuator.config.name in missing_actuators:
                                if other_actuator._probe_endpoint(ep):
                                    missing_actuators.remove(other_actuator.config.name)
                                else:
                                    other_actuator.config.clear_endpoint()
                    
                    break
            
            if not act.config.has_endpoint():
                missing_actuators.append(act.config.name)
        
        # Print info on which actuators were found on which endpoints
        n_actuators = len([a for a in acts if a.config.has_endpoint()])
        used_endpoints = set(a.config.endpoint for a in acts if a.config.has_endpoint())
        endpoint_map = {e: sorted([a.config.name for a in acts if a.config.endpoint == e]) for e in used_endpoints}
        endpoints_str = '\n\t'.join([f"{e}: {list(endpoint_map[e])}" for e in sorted(endpoint_map.keys())])
        info(f"Found {n_actuators} / {len(acts)} actuators on {len(used_endpoints)} endpoints:\n\t{endpoints_str}")
        log_json(NOTICE, "actuator_find", {
            "n_actuators": n_actuators,
            "total_actuators": len(acts),
            "endpoint_map": endpoint_map,
            "missing_actuators": missing_actuators,
        })
        
        if len(missing_actuators) > 0 and not allow_not_found:
            raise ActuatorNotFoundError(f"Could not find actuators {missing_actuators} on any CAN endpoint.")

    def _abort_impl(self) -> None:
        """Handle system abort and stop all systems. Signum and frame are only used for signal handling."""
        commit_suicide = self.config.kill_on_failed_abort

        try:
            # Modules that, if not None, will be aborted in a specific order after all other modules
            manual_ordering = [self.data_monitor_service]

            if self._client is None:
                # Call on_abort on all modules that are not in the manual ordering.
                for module in [m for m in self.modules.values() if m not in manual_ordering]:
                    with try_log_exc(f"Error in on_abort for {module.__class__.__name__}, ignoring: {format_exc()}"):
                        info(f"Aborting {module.__class__.__name__}")
                        module._abort_impl()
                
                if self.comms_manager is not None:
                    comms_name = self.comms_manager.__class__.__name__
                    with try_log_exc(f"Error in on_abort for {comms_name}, ignoring: {format_exc()}"):
                        info(f"Aborting {comms_name}")
                        self.comms_manager._abort_impl()

                if self.data_monitor_service is not None:
                    dms_name = self.data_monitor_service.__class__.__name__
                    with try_log_exc(f"Error in on_abort for {dms_name}, ignoring: {format_exc()}"):
                        info(f"Aborting {dms_name}")
                        self.data_monitor_service._abort_impl()
                
                # Release the robot instance lock.
                with try_log_exc(f"Error in on_abort for release_robot_instance_lock, ignoring: {format_exc()}"):
                    release_robot_instance_lock(allow_no_lock=True, timeout=0.3)
            
            # Terminate the client's server process if needed.
            else:
                if self._client._server_proc is not None:
                    with try_log_exc(f"Error in on_abort for server process, ignoring: {format_exc()}"):
                        self._client._server_proc.abort()
            
            # Abort all running threads.
            try:
                info(f"Aborting all running executables")
                _FeatherExecutable.abort_all_and_join()
                commit_suicide = False
            except Exception as e:
                error(f"Error aborting threads: {e}. We will kill ourself to avoid hanging processes.")
            
        except Exception as e:
            error(f"Uncaught error in on_abort: {e}")
        finally:
            # No matter what, try to flush the logs.
            with suppress(Exception):
                flush_logs()
            
            # Kill ourself if we were not able to abort all threads.
            if commit_suicide:
                os.kill(os.getpid(), signal.SIGKILL)


@perftime()
def initialize_feather_system(
    config_or_path: Optional[Union[FeatherRobotConfig, Path, str]] = None, ros_node: Optional[_ROSNodeType] = None
) -> FeatherRobot:
    """Initialize the feather system from a config file.
    
    Args:
        config_or_path: The configuration for the robot, or the path to a config file. If None, load the config from the
            default configs directory.
        ros_node: If using ROS, pass the ROS node here. This is mainly used to set the ROS logger for logging.
    """
    if config_or_path is None:
        config_or_path = get_robot_config_path()
    
    conf = config_or_path if isinstance(config_or_path, FeatherRobotConfig) else load_configs(Path(config_or_path))
    if not isinstance(conf, FeatherRobotConfig):
        raise TypeError(f"\"{config_or_path}\" did not export a `{FeatherRobotConfig.__name__}`, got {type(conf)}")
    
    if ros_node is not None:
        if conf.kill_on_failed_abort or conf.set_atexit_handler or conf.set_signal_handlers:
            raise ValueError("Cannot set any of `kill_on_failed_abort`, `set_atexit_handler`, or `set_signal_handlers` "
                             "when using ROS, as the robot will be killed by the ROS node shutdown.")
        ensure(conf.logging_config).set_using_ros_logger(ros_node.get_logger())
    
    return FeatherRobot(conf)
