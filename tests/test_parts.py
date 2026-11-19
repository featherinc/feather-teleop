# The parts a demonstration drives, on stand-ins for the SDK's systems: base, necks, torso, cameras, display.
from types import SimpleNamespace

import numpy as np
import pytest
from fakes import TABLE, Headset, make_follower, run
from feathersdk.robot.actuator_systems.collision import CollisionError

from feather_teleop.control.maths import to_matrix
from feather_teleop.headsets import Reading
from feather_teleop.parts.arms import Arms
from feather_teleop.parts.base import Base, BaseSettings
from feather_teleop.parts.cameras import Cameras
from feather_teleop.parts.display import Display
from feather_teleop.parts.neck import HeadNeck, NeckSettings, StickNeck, camera_elevation, head_angles
from feather_teleop.parts.sticks import STICK_DEADZONE
from feather_teleop.parts.torso import TORSO_RETRY, Torso, TorsoSettings

BASE = BaseSettings(linear_speed=0.2, angular_speed=0.333)
SIGNS = NeckSettings(speed=1.0, pitch_sign=-1, yaw_sign=-1)   # the motors turn + down and + right


def reading(**buttons):
    # A fresh sample with these buttons changed.
    return Reading({}, {**Headset().latest().buttons, **buttons}, 0.0)


def scaled(deflection):
    # A stick's command at this deflection, past the deadzone.
    return (deflection - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)


@pytest.mark.parametrize("worn", [False, True])
def test_base(worn):
    sent = []
    navigation = SimpleNamespace(enable=lambda: None, disable=lambda: None, set_strict_directions=lambda strict: None, set_velocity=lambda *velocity: sent.append(velocity))
    base = Base(navigation, BASE, tick_divisor=1, turn_with_stick=worn)
    assert base.step(reading(leftJS=(0.0, 1.0)), 0.05)["base"] == pytest.approx((0.2, 0.0, 0.0))   # forward
    assert base.step(reading(leftJS=(1.0, 0.0)), 0.05)["base"] == pytest.approx((0.0, -0.2, 0.0))  # right
    assert base.step(reading(leftJS=(0.1, 0.0)), 0.05)["base"] == (0.0, 0.0, 0.0)                   # deadzone
    turn = base.step(reading(X=True, rightJS=(-0.6, 0.0)), 0.05)["base"][2]
    # Table: X turns left. Worn: the right stick turns, and X does nothing.
    assert turn == pytest.approx(0.333 * scaled(0.6) if worn else 0.333)
    assert base.step(None, 0.05)["base"] == (0.0, 0.0, 0.0) and sent[-1] == (0.0, 0.0, 0.0)   # stale: stop


class Neck:
    # Two motors where they were last sent, `behind` rad short of it. `broken` makes telemetry raise.

    def __init__(self, pitch=-0.2, yaw=0.1, pitch_range=(-1.0, 0.3)):
        def motor(name, low, high):
            return SimpleNamespace(config=SimpleNamespace(name=name, angle_range=(low, high)), command_angle_range=lambda: (low - 1.0, high + 1.0))
        self.pitch_motor, self.yaw_motor = motor("pitch", *pitch_range), motor("yaw", -1.2, 1.2)
        self.position = {"pitch": pitch, "yaw": yaw}
        self.behind, self.broken, self.sent = 0.0, False, []

    def read_state(self):
        if self.broken:
            raise ValueError("no telemetry")
        return SimpleNamespace(names=list(self.position), position=np.array(list(self.position.values())) - self.behind)

    def set_targets(self, targets, speeds=None):
        for motor in (self.pitch_motor, self.yaw_motor):
            low, high = motor.config.angle_range
            assert low <= targets[motor.config.name] <= high, "the SDK refuses a target outside the range"
        self.position = dict(targets)
        self.sent.append(speeds)


def test_stick_neck():
    neck = Neck()
    part = StickNeck(neck, NeckSettings(speed=1.0, pitch_sign=1, yaw_sign=-1), tick_divisor=1)
    part.start(None)
    down_left = reading(rightJS=(-0.6, -0.8))   # full deflection
    for _ in range(10):   # 0.5 s at 1 rad/s
        record = part.step(down_left, 0.05)
    assert record["neck"] == pytest.approx({"pitch": -0.6, "yaw": -0.2})
    assert part.step(None, 0.05)["neck"] == record["neck"]   # stale: it holds
    for _ in range(100):
        record = part.step(down_left, 0.05)
    assert record["neck"]["pitch"] == -1.0   # never past the range


class Head:
    # A worn headset with only a head, looking `up` and `left` rad.

    def __init__(self, up=0.0, left=0.0):
        self.up, self.left, self.epoch, self.tracked, self.look = up, left, 1, True, None

    def reading(self):
        pose = np.eye(4)
        pose[:3, :3] = to_matrix([0, 0, self.left]) @ to_matrix([0, -self.up, 0])
        return Reading({"head": pose} if self.tracked else {}, {}, 0.0)


def steps(part, head, count, dt=0.05):
    for _ in range(count):
        record = part.step(head.reading(), dt)
    return record


def test_head_angles_and_elevation():
    assert head_angles(Head(up=0.3, left=-0.5).reading().poses["head"]) == pytest.approx((0.3, -0.5))
    assert head_angles(Head(up=np.pi / 2).reading().poses["head"]) is None   # straight up: no heading
    for accel, up in (((0.19, 0.32, -9.94), -1.53), ((0.0, -9.81, 0.0), 0.0), ((0.0, -6.94, 6.94), 0.785)):
        cameras = SimpleNamespace(get_imu_data=lambda accel=accel: accel)
        assert camera_elevation(cameras)() == pytest.approx(up, abs=0.01)
    with pytest.raises(ValueError, match="not at rest"):
        camera_elevation(SimpleNamespace(get_imu_data=lambda: (0.0, 0.1, 0.2)))()


def test_head_neck():
    neck, head = Neck(), Head(left=0.4)   # already looking left at the start: that is the reference
    part = HeadNeck(neck, head, SIGNS)
    part.start(None)
    steps(part, head, 1)
    assert neck.position == pytest.approx({"pitch": -0.2, "yaw": 0.1}) and head.look == pytest.approx((0.0, 0.4))
    head.up, head.left = 0.1, 0.6
    record = steps(part, head, 10)
    assert neck.position == pytest.approx({"pitch": -0.3, "yaw": -0.1}) and record["head"] == pytest.approx((0.1, 0.6))
    assert head.look == pytest.approx((0.1, 0.6)) and neck.sent[-1] == {"pitch": 1.0, "yaw": 1.0}

    neck.behind = 0.03   # the camera looks where the neck is measured, not where it was sent
    steps(part, head, 1)
    assert head.look == pytest.approx((0.13, 0.63))
    neck.behind, neck.broken = 0.0, True   # no telemetry: the page keeps the picture in front of the head
    steps(part, head, 1)
    assert head.look is None
    neck.broken = False

    assert part.step(None, 0.05)["head"] is None   # stale: it holds
    head.tracked = False
    steps(part, head, 1)
    held = dict(neck.position)
    head.tracked, head.left, head.epoch = True, 2.0, 2   # recentred: the same head, new numbers
    steps(part, head, 5)
    assert neck.position == pytest.approx(held)
    head.left = 2.1
    steps(part, head, 5)
    assert neck.position["yaw"] == pytest.approx(held["yaw"] - 0.1)
    part.stop()
    assert head.look is None


def test_head_neck_limits():
    neck, head = Neck(pitch=0.45), Head()   # resting past its 0.3 limit: brought in, not refused
    part = HeadNeck(neck, head, NeckSettings(speed=0.5, pitch_sign=-1, yaw_sign=-1))
    part.start(None)
    steps(part, head, 1)
    assert neck.position["pitch"] == pytest.approx(0.3)
    head.up = 1.4
    steps(part, head, 1)
    assert neck.position["pitch"] == pytest.approx(0.3 - 0.5 * 0.05)   # no faster than its speed
    steps(part, head, 100)
    assert neck.position["pitch"] == pytest.approx(-1.0)   # and no further than its range

    neck, head = Neck(pitch=0.2, pitch_range=(-1.5, 0.3)), Head(up=0.1, left=0.5)
    part = HeadNeck(neck, head, SIGNS, elevation=lambda: -1.2)   # resting with the camera 1.2 rad down
    part.start(None)
    steps(part, head, 40)
    assert neck.position == pytest.approx({"pitch": -1.1, "yaw": 0.1})   # the camera ends 0.1 above level
    assert head.look == pytest.approx((0.1, 0.5))
    with pytest.raises(ValueError, match="1 or -1"):
        HeadNeck(neck, head, NeckSettings(speed=1.0, pitch_sign=1, yaw_sign=0.5))


class Lift:
    max_velocity_mm_s = 100.0

    def __init__(self, refuse_down=False):
        self.asked, self.moving, self.refuse_down = [], True, refuse_down

    def update_velocity(self, velocity):
        if velocity < 0 and self.refuse_down:
            raise CollisionError("an arm is below", None)
        self.asked.append(velocity)

    def read_state(self):
        return SimpleNamespace(height=300.0, moving=self.moving)


def test_torso_follows_the_hands(tmp_path):
    follower, *_ = make_follower(tmp_path)
    lift = Lift()
    torso = Torso(lift, [follower], TorsoSettings(rise_above=0.10, sink_below=-0.60), say=lambda line: None)
    shoulder, wrist = follower.points().shoulder[2], follower.points().wrist[2]
    tool = follower.ik.pose(follower.q * follower.direction)[0][2]

    def velocity(above_shoulder, gripping=True):
        follower.controller.gripping = gripping
        follower.target = np.array([0.3, 0.0, shoulder + above_shoulder + (tool - wrist)])
        return torso.step(reading(), 0.02)["torso"]["velocity"]

    assert velocity(0.20, gripping=False) == 0.0   # only a gripped hand moves it
    assert [velocity(h) for h in (0.20, 0.07, 0.04, -0.70)] == [50.0, 50.0, 0.0, -50.0]   # with hysteresis
    assert lift.asked == [50.0, 0.0, -50.0]   # only changes are sent


def test_torso_buttons_and_refusals(tmp_path):
    lift, clock = Lift(), [0.0]
    torso = Torso(lift, [], TorsoSettings(0.10, -0.60), buttons=True, clock=lambda: clock[0])
    assert torso.step(reading(Y=True), 0.02)["torso"]["velocity"] == 50.0   # Y raises
    lift.moving = False   # the motor missed it: sent again after TORSO_RETRY
    torso.step(reading(Y=True), 0.02)
    clock[0] += TORSO_RETRY
    torso.step(reading(Y=True), 0.02)
    assert lift.asked == [50.0, 50.0]
    assert torso.step(reading(X=True), 0.02)["torso"]["velocity"] == -50.0   # X lowers
    assert torso.step(reading(), 0.02)["torso"]["velocity"] == 0.0

    said, lift = [], Lift(refuse_down=True)
    torso = Torso(lift, [], TorsoSettings(0.10, -0.60), buttons=True, say=said.append, clock=lambda: clock[0])
    torso.step(reading(Y=True), 0.02)
    torso.step(reading(X=True), 0.02)   # refused while rising: the rise is halted
    torso.step(reading(X=True), 0.02)   # not asked again at once
    assert lift.asked == [50.0, 0.0] and said == ["torso: an arm is below"]
    clock[0] += TORSO_RETRY
    torso.step(reading(X=True), 0.02)   # asked again later, and refused again, said once
    torso.step(reading(Y=True), 0.02)   # up is never held back
    assert lift.asked == [50.0, 0.0, 50.0] and len(said) == 1


def test_cameras(tmp_path):
    faults, stopped = {"left_wrist": "not delivering"}, {"stereo": {"failure": None, "missed": 2}}
    service = SimpleNamespace(names=["stereo", "left_wrist"], faults=faults, start_recording=lambda path: None, stop_recording=lambda: stopped)
    said = []
    cameras = Cameras(service, say=said.append)
    cameras.start(SimpleNamespace(path=tmp_path))
    assert cameras.step(None, 0.05) == {}   # known faults don't stop a demonstration
    faults["stereo"] = "stalled"
    with pytest.raises(RuntimeError, match="stereo: stalled"):
        cameras.step(None, 0.05)   # a new one does
    cameras.stop()
    cameras.stop()   # once only
    assert said == ["recording without left_wrist (not delivering)", "camera stereo: 2 frames were not recorded"]


def test_display():
    statuses, said, debug, shown, now = [], [], [], {}, [100.0]
    headset = SimpleNamespace(set_status=lambda **fields: statuses.append(fields), set_debug=debug.append)
    video = SimpleNamespace(running=False, fault=None, restarts=0, budget=True, failure=lambda: video.fault, close=lambda: None)

    def show(side, on):
        if on and not shown.get(side) and not video.budget:   # as the service: only a change can be refused
            raise ValueError("over the encoders' budget")
        shown[side] = on
    video.show = show
    video.start = lambda: setattr(video, "running", True)
    video.stop = lambda: setattr(video, "running", False)
    video.restart = lambda: setattr(video, "restarts", video.restarts + 1)
    followers = [SimpleNamespace(side=side, wrist_view=True, refused=None, held_back=[], gripper=SimpleNamespace(fault=None)) for side in ("left", "right")]
    display = Display(headset, video, followers, say=said.append, debug=True, clock=lambda: now[0])

    def step(seconds=0.02):
        now[0] += seconds
        display.step(None, seconds)

    display.start(None)
    assert video.running and statuses == [{"phase": "ready"}, {"phase": "active"}]
    assert not any(f.wrist_view for f in followers)   # each demonstration starts with the stereo picture alone
    followers[1].wrist_view = True
    step()
    assert shown == {"left": False, "right": True}
    video.budget = False
    followers[0].wrist_view = True
    step()
    assert not followers[0].wrist_view and "no left wrist view" in said[-1]   # refused: the double press undone

    video.fault = "stream:stereo: stalled"
    step()
    step()   # started again once, not every tick
    assert video.restarts == 1 and said[-2:] == ["headset video: stream:stereo: stalled", "headset video: started again"]
    step(2.0)
    assert video.restarts == 2
    video.fault = None
    step()
    assert said[-1] == "headset video: back"

    followers[1].held_back = ["J3"]
    for _ in range(30):
        step()
    assert debug[0] == {"faults": [], "hz": None} and debug[-1]["faults"] == ["right held back: J3"] and debug[-1]["hz"] > 0
    display.stop()
    assert not video.running and statuses[-1] == {"phase": "ready"}


def test_arms(tmp_path):
    follower, headset, arm, clock = make_follower(tmp_path)
    said = []
    arms = Arms([follower], headset, release_after=0.2, say=said.append)
    states = []
    for change in ({"grip": True}, {"stick": True}, {"stick": False}, {"grip": False}):
        for name, value in change.items():
            getattr(headset, name)["r"] = value
        record = arms.step(None, 0.02)["arms"]["right"]
        states.append(said[-1])
    assert states == ["right arm: following", "right arm: following", "right arm: following, position locked", "right arm: holding"]
    assert record["following"] is False and len(record["command"]) == 7

    headset.grip["r"] = True
    run(follower, clock, 30, lambda i: headset.move("r", 0.002 * TABLE["forward"]))
    headset.grip["r"] = False
    for stop, why in ((lambda: headset.pressed.add("B"), "B pressed"), (lambda: setattr(arm, "refuse", True), "held back")):
        follower.start()
        arm.position = arm.position + 0.1   # away from the start posture
        follower.q = arm.position.copy()
        stop()
        with pytest.raises(RuntimeError, match=why):
            arms._go_to_start_posture()
        headset.pressed.clear()
        arm.refuse = False
