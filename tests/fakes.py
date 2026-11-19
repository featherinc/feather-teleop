# Stand-ins for the headset, the SDK arm and the gripper, and the small arm model the solver runs on.
from types import SimpleNamespace

import numpy as np
import pinocchio as pin
from feathersdk.robot.actuator_systems.arm_kinematics import ArmIK, Guards, GuardSettings, IKSettings
from feathersdk.robot.actuator_systems.collision import CollisionError
from feathersdk.robot.actuator_systems.planning import MotionLimits

from feather_teleop import config
from feather_teleop.control.controller import Controller, ControllerSettings
from feather_teleop.control.follower import HOME, ArmFollower, ArmSettings, hand_alignment
from feather_teleop.control.maths import to_matrix
from feather_teleop.headsets import Reading
from feather_teleop.loop import RELEASE_AFTER

# A made-up robot for the tests: quest.yaml leaves every robot's values for the user to measure.
TEST_ROBOT = {"config": "v0.5.1", "urdf": "robot.urdf", "grippers": ["left", "right"], "cameras": {}, "tool_offset": {"left": [-0.024, 0.001, -0.219], "right": [0.024, -0.001, -0.219]}, "body_boxes": [{"x": [-0.18, 0.20], "y": [-0.10, 0.10]}, {"x": [-0.26, 0.25], "y": [-0.25, 0.25], "below": 0.35}], "body_clear": 0.10, "start_posture": {"EpL": -1.484, "EpR": 1.484}, "neck_pitch_sign": -1, "neck_yaw_sign": -1}


def robot_settings() -> dict:
    # quest.yaml's settings, run on TEST_ROBOT.
    raw = config.read()
    return config.resolve({**raw, "robot": "test", "robots": {"test": TEST_ROBOT}})


SETTINGS = robot_settings()
TICK = 0.02    # s, the fake arm's tick
LIMIT = 3.0    # rad, every joint's range is +-LIMIT
SPEED = 2.0    # rad/s, every joint's speed limit
BENT = np.array([-0.4, 0.5, 0.3, -1.3, 0.2, 0.3, -0.2])   # the model arm, elbow bent and out to the side

# A table controller's axes as the robot's: its -z is forward, -x left, +y up. Worn poses come in the robot's.
TABLE = {"forward": np.array([0.0, 0.0, -1.0]), "left": np.array([-1.0, 0.0, 0.0]), "up": np.array([0.0, 1.0, 0.0])}
WORN = {"forward": np.array([1.0, 0.0, 0.0]), "left": np.array([0.0, 1.0, 0.0]), "up": np.array([0.0, 0.0, 1.0])}


def arm_model(path):
    # A seven-joint arm shaped like the robot's: three shoulder joints at one point, an elbow, three at the wrist.
    origins = ("0 0 1", "0 0 0", "0 0 0", "0 0 -0.3", "0 0 -0.3", "0 0 0", "0 0 0")
    axes = ("0 1 0", "1 0 0", "0 0 1", "0 1 0", "0 0 1", "1 0 0", "0 1 0")
    links, parent = ['<link name="base"/>'], "base"
    for i, (axis, origin) in enumerate(zip(axes, origins)):
        links.append(f'<link name="l{i}"><inertial><origin xyz="0 0 -0.1"/><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="J{i}" type="revolute"><parent link="{parent}"/><child link="l{i}"/><origin xyz="{origin}"/><axis xyz="{axis}"/><limit lower="-{LIMIT}" upper="{LIMIT}" effort="10" velocity="{SPEED}"/></joint>')
        parent = f"l{i}"
    path.write_text(f'<?xml version="1.0"?><robot name="arm">{"".join(links)}</robot>')
    return pin.buildModelFromUrdf(str(path))


class Headset:
    # A Quest: a pose and buttons per controller and the head, fresh unless `age` says otherwise.

    def __init__(self):
        self.pose = {"l": np.eye(4), "r": np.eye(4), "head": np.eye(4)}
        self.grip = {"l": False, "r": False}
        self.stick = {"l": False, "r": False}
        self.trigger = {"l": 0.0, "r": 0.0}
        self.sticks = {"leftJS": (0.0, 0.0), "rightJS": (0.0, 0.0)}
        self.pressed = set()
        self.visible = {"l", "r", "head"}
        self.age = 0.0
        self.epoch = 0

    def latest(self) -> Reading:
        buttons = {name: name in self.pressed for name in "ABXY"}
        buttons.update(RG=self.grip["r"], LG=self.grip["l"], RJ=self.stick["r"], LJ=self.stick["l"], rightTrig=(self.trigger["r"],), leftTrig=(self.trigger["l"],), **self.sticks)
        poses = {name: pose.copy() for name, pose in self.pose.items() if name in self.visible}
        return Reading(poses, buttons, self.age)

    def move(self, name, delta):
        self.pose[name][:3, 3] += delta

    def turn(self, name, rotation_vector):
        self.pose[name][:3, :3] = to_matrix(np.asarray(rotation_vector, float)) @ self.pose[name][:3, :3]

    def close(self):
        pass


class FakeArm:
    # An SDK arm in follow mode that is at its newest target at once. `refuse` makes a moving target collide.

    def __init__(self, start=BENT):
        self.names = [f"m{i}" for i in range(7)]
        self.direction = np.array([1 if i % 2 else -1 for i in range(7)])
        self.position = np.array(start, float) * self.direction   # motor frame
        self.mode = "targets"
        self.command = None
        self.refuse = False
        self.sent = []      # (positions, velocities) of every target
        self.speed = None   # the follow limits' velocities

    def describe(self):
        joints = [SimpleNamespace(name=self.names[i], urdf_joint=f"J{i}", urdf_direction=int(self.direction[i]), position=(-LIMIT, LIMIT), velocity=SPEED, acceleration=30.0) for i in range(7)]
        limits = MotionLimits(np.full(7, SPEED), np.full(7, 30.0), np.full(7, 1000.0))
        return SimpleNamespace(joints=joints, rate=1.0 / TICK, limits=lambda: limits)

    def read_state(self):
        command = None if self.command is None else SimpleNamespace(position=self.command.copy(), velocity=np.zeros(7))
        return SimpleNamespace(names=tuple(self.names), position=self.measured(), command=command)

    def measured(self):
        return self.position.copy()

    def set_control_mode(self, mode):
        self.mode = mode

    def set_follow_limits(self, max_velocities):
        self.speed = np.array([max_velocities[name] for name in self.names])

    def set_follow_target(self, positions, velocities):
        assert self.mode == "follow"
        q = np.array([positions[name] for name in self.names])
        v = np.array([velocities[name] for name in self.names])
        assert np.all(np.abs(v) <= self.speed), "a velocity above the follow limits: the SDK raises ValueError"
        if self.refuse and not np.allclose(q, self.position):
            raise CollisionError("the arm would touch the body", None)
        self.position = self.command = q
        self.sent.append((q, v))


class Gripper:
    # A gripper whose jaws go where they are sent, unless `blocked_at` stops them there. `fault` makes it raise; `late` makes its feedback that many readings old.

    def __init__(self, opening=1.0, blocked_at=None, fault=None, late=0):
        self.opening = opening
        self.blocked_at = blocked_at
        self.fault = fault
        self.late = late
        self.sent = []
        self._readings = []

    def get_opening(self):
        if self.fault:
            raise RuntimeError(self.fault)
        self._readings.append(self.opening)
        return self._readings[max(0, len(self._readings) - 1 - self.late)]

    def go_to(self, opening):
        if self.fault:
            raise RuntimeError(self.fault)
        self.sent.append(opening)
        self.opening = opening if self.blocked_at is None else max(opening, self.blocked_at)


class Clock:
    # A clock the test advances, so a follower's steps don't depend on how fast the test runs.

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def make_ik(tmp_path, tool_offset=(0.0, 0.0, 0.0)):
    model = arm_model(tmp_path / "arm.urdf")
    return ArmIK(pin, model, range(7), range(7), "J6", [-LIMIT] * 7, [LIMIT] * 7, [SPEED] * 7, IKSettings(), tool_offset, BENT)


def make_follower(tmp_path, worn=False, mirror=False, arm=None, gripper=None, double_press=HOME, **arm_settings):
    # A follower of the right arm on a FakeArm, started. Returns (follower, headset, arm, clock).
    headset, arm, clock = Headset(), arm or FakeArm(), Clock()
    robot = SETTINGS["robot"]
    ik = make_ik(tmp_path, robot["tool_offset"]["right"])
    guards = Guards(ik, "J0", "J3", GuardSettings(body_boxes=robot["body_boxes"], body_clear=robot["body_clear"]))
    controller_settings = ControllerSettings.worn() if worn else ControllerSettings()
    controller = Controller(headset, "l" if mirror else "r", controller_settings, mirror, spatial_scale=1.0, release_after=RELEASE_AFTER, clock=clock)
    if worn:
        controller.hand_alignment = hand_alignment(ik, BENT)
    settings = ArmSettings(**{**SETTINGS["arm"], "floor": -10.0, **arm_settings})
    follower = ArmFollower("right", arm, ik, controller, guards, settings, BENT, gripper, double_press_action=double_press, clock=clock)
    follower.start()
    return follower, headset, arm, clock


def run(follower, clock, ticks, each=None):
    # Step `ticks` times, a tick apart; `each(i)` runs before step i.
    for i in range(ticks):
        if each:
            each(i)
        clock.now += TICK
        follower.step()


def hand(follower):
    # The tool point's position and the hand's rotation at the follower's last target.
    return follower.ik.pose(follower.q * follower.direction)


def press_twice(headset, side="r"):
    # `each` for run(): two short grip presses, 8 ticks apart.
    return lambda i: headset.grip.update({side: i < 3 or 8 <= i < 11})
