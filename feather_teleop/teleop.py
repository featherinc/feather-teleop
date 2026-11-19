# Assembles a teleoperation session on the robot: the headset, the arm followers, the parts and the loop.
#
# settings = config.load()
# robot = initialize_feather_system(sdk_config(settings))
# teleop = QuestTeleop(robot, settings)
# teleop.ready()     # torso up, arms to the start posture
# teleop.run()       # demonstrations until interrupted
# teleop.home()
# teleop.close()
from typing import List, Optional

from feathersdk.robot.actuator_systems.arm_kinematics import ArmIK, Guards, GuardSettings

from feather_teleop import config
from feather_teleop.control.controller import Controller, ControllerSettings
from feather_teleop.control.follower import HOME, WRIST_VIEW, ArmFollower, ArmSettings, hand_alignment
from feather_teleop.headsets.table import TableHeadset, TableLink
from feather_teleop.headsets.worn import ServiceVideo, WornHeadset
from feather_teleop.loop import RELEASE_AFTER, DemonstrationLoop, Part
from feather_teleop.parts.arms import Arms, raise_torso
from feather_teleop.parts.base import Base, BaseSettings
from feather_teleop.parts.cameras import Cameras
from feather_teleop.parts.display import Display
from feather_teleop.parts.neck import HeadNeck, NeckSettings, StickNeck, camera_elevation
from feather_teleop.parts.sticks import STICK_RATE
from feather_teleop.parts.torso import Torso, TorsoSettings

CONTROLLER_SIDE = {"left": "l", "right": "r"}   # arm side to the headset's controller key
OTHER_SIDE = {"left": "right", "right": "left"}


def open_headset(settings: dict, say=print):
    # Open the headset the settings name.
    if settings["headset"] == "worn":
        worn = settings["worn"]
        files = [None if worn[name] is None else config.folder(worn[name]) for name in ("cert", "key")]
        return WornHeadset(worn["address"], worn["port"], *files, say=say)
    source = settings["table"]["source"]
    if source == "usb":
        return TableHeadset()
    if source == "network":
        return TableLink(settings["table"]["port"])
    raise ValueError(f"table.source is {source!r}; it must be 'usb' or 'network'")


def build_follower(side: str, robot, headset, settings: dict) -> ArmFollower:
    # Build the follower of one arm, with its solver, guards and controller.
    arm = getattr(robot.arms_system, f"{side}_arm")
    start_posture = [settings["start_posture"].get(joint.urdf_joint, 0.0) for joint in arm.describe().joints]
    ik = ArmIK.for_arm(arm, tool_offset=settings["tool_offset"][side], rest_posture=start_posture)
    guards = Guards.for_arm(ik, arm, GuardSettings(body_boxes=settings["robot"]["body_boxes"]))

    worn = settings["headset"] == "worn"
    hand = OTHER_SIDE[side] if settings["mirror"] else side   # mirrored: the controller in the opposite hand
    controller_settings = ControllerSettings.worn() if worn else ControllerSettings()
    controller = Controller(headset, CONTROLLER_SIDE[hand], controller_settings, mirror=settings["mirror"], spatial_scale=settings["spatial_scale"], release_after=RELEASE_AFTER)
    if controller.settings.absolute_orientation:
        controller.hand_alignment = hand_alignment(ik, start_posture)

    gripper = getattr(robot.manipulation_system, f"{side}_gripper", None)   # None also without grippers
    return ArmFollower(side, arm, ik, controller, guards, ArmSettings(**settings["arm"]), start_posture=start_posture, gripper=gripper, double_press_action=WRIST_VIEW if worn else HOME)


class QuestTeleop:
    # A teleoperation session.
    #
    # Creating it enables the arms and, unless `collision_margin` is null, the SDK's collision check. Shutting the robot down is left to the caller.
    #
    # Args:
    #     robot: The SDK's robot, from `initialize_feather_system()`.
    #     settings: The settings. The default is config/quest.yaml.
    #     headset: The headset. The default is opened as the settings say.
    #     say: Called with each message for the operator.
    #
    # Raises:
    #     ValueError: The settings ask for something the robot's config does not have, or do not fit together.

    def __init__(self, robot, settings: Optional[dict] = None, headset=None, say=print):
        self.robot = robot
        self.say = say
        self.settings = config.load() if settings is None else settings
        self._check_settings()
        self.worn = self.settings["headset"] == "worn"
        self.headset = open_headset(self.settings, self.say) if headset is None else headset
        self.parts: List[Part] = []
        try:
            self._build()
        except BaseException:
            self.close()   # or the headset's port stays taken until the kernel restarts
            raise

    def _build(self) -> None:
        # Build the followers, the parts and the loop, and enable the arms.
        robot = self.robot
        s = self.settings
        self.followers = [build_follower(side, robot, self.headset, s) for side in s["arms"]]
        if len(self.followers) == 2:   # each arm keeps clear of the other
            self.followers[0].other = self.followers[1]
            self.followers[1].other = self.followers[0]
        self._add_parts()
        self.loop = DemonstrationLoop(self.headset, self.parts, period=self.followers[0].tick, episodes=config.folder(s["episodes"]), task=s["task"], say=self.say)
        robot.arms_system.enable()
        if s["collision_margin"] is not None:
            robot.arms_system.set_collision_check(True, margin=s["collision_margin"])

    def _check_settings(self) -> None:
        # Raise ValueError if the settings do not fit together or ask for what the robot lacks.
        robot = self.robot
        s = self.settings
        if s["headset"] not in ("table", "worn"):
            raise ValueError(f"headset is {s['headset']!r}; it must be 'table' or 'worn'")
        if s["headset"] == "worn" and s["mirror"]:
            raise ValueError("mirror is for the table headset; worn, you face the way the robot does")
        systems = {"arms": robot.arms_system, "base": robot.navigation_system, "neck": robot.neck_system, "torso": robot.torso_system, "cameras": robot.camera_service}
        unknown = set(s["parts"]) - set(systems)
        if unknown or "arms" not in s["parts"]:
            optional = ", ".join(name for name in systems if name != "arms")
            raise ValueError(f"parts is {s['parts']}; it must include arms and may add any of {optional}")
        missing = [name for name in s["parts"] if systems[name] is None]
        if missing:
            raise ValueError(f"the robot's config has no {', '.join(missing)}; remove it from `parts` in quest.yaml")
        if not s["arms"] or set(s["arms"]) - set(CONTROLLER_SIDE):
            raise ValueError(f"arms is {s['arms']}; it must be left, right or both")
        absent = [side for side in s["arms"] if getattr(robot.arms_system, f"{side}_arm") is None]
        if absent:
            raise ValueError(f"the robot's config has no {' or '.join(absent)} arm; remove it from `arms` in quest.yaml")

    def _add_parts(self) -> None:
        # Build the parts in the order they are started and stepped.
        #
        # The arms come last, so anything slow to start (the recorders) is running before they move.
        robot = self.robot
        s = self.settings
        wanted = s["parts"]
        # The base and the stick-driven neck are commanded at the sticks' rate, slower than the arms'.
        divisor = max(1, round(1.0 / (self.followers[0].tick * STICK_RATE)))
        parts = self.parts   # filled as they are built, so a failed build can close the ones it made
        if self.worn:
            parts.append(self._display())
        if "cameras" in wanted:
            parts.append(Cameras(robot.camera_service, self.say))
        if "base" in wanted:
            parts.append(Base(robot.navigation_system, BaseSettings(**s["base"]), divisor, turn_with_stick=self.worn))
        if "neck" in wanted:
            neck_settings = NeckSettings(**s["neck"])
            if self.worn:
                cameras = robot.camera_service
                elevation = camera_elevation(cameras) if cameras is not None else None
                parts.append(HeadNeck(robot.neck_system, self.headset, neck_settings, elevation))
            else:
                parts.append(StickNeck(robot.neck_system, neck_settings, divisor))
        if "torso" in wanted:
            parts.append(Torso(robot.torso_system, self.followers, self._torso_settings(), buttons=self.worn, say=self.say))
        torso_speed = self._torso_settings().speed if "torso" in wanted else None
        parts.append(Arms(self.followers, self.headset, RELEASE_AFTER, robot.torso_system, torso_speed, self.say))

    def _display(self) -> Display:
        # The worn headset's picture, from the camera service's live stream.
        cameras = self.robot.camera_service
        show_video = cameras is not None and self.settings["worn"]["video"]
        worn = self.settings["worn"]
        video = ServiceVideo(cameras, self.headset, video_port=worn["video_port"]) if show_video else None
        return Display(self.headset, video, self.followers, self.say, debug=worn["debug"])

    def _torso_settings(self) -> TorsoSettings:
        return TorsoSettings(**self.settings["torso"])

    def ready(self) -> None:
        # Raise the torso (if in `parts`), then move the arms to the start posture. Not collision checked.
        self._move_arms(self.settings["start_posture"])

    def home(self) -> None:
        # Raise the torso (if in `parts`), then move the arms to their zero posture. Not collision checked.
        self._move_arms({})

    def _move_arms(self, posture: dict) -> None:
        # Move the arms to `posture` (by URDF joint; joints not named go to 0) and wait.
        if "torso" in self.settings["parts"]:
            raise_torso(self.robot.torso_system, self._torso_settings().speed)
        goal = {joint.name: posture.get(joint.urdf_joint, 0.0) * joint.urdf_direction for follower in self.followers for joint in follower.joints}
        self.robot.arms_system.go_to(goal, max_velocity=self.settings["arm"]["return_speed"])

    def run(self) -> None:
        # Run demonstrations until Ctrl+C (a notebook's stop button).
        self.loop.run()

    def close(self) -> None:
        # Close every part and the headset. The arms stay enabled, holding.
        try:
            for part in self.parts:
                part.close()
        finally:
            self.headset.close()
