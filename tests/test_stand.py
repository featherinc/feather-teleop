# The session on FeatherSDK's test stand: the real systems, with only the link to the motor controller faked. Skipped without FEATHERSDK pointing at an SDK checkout (see conftest.py).
import copy
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

stand = pytest.importorskip("tests.unit_tests.robot._stand")

from fakes import SETTINGS, Headset  # noqa: E402

from feather_teleop.headsets import Reading  # noqa: E402
from feather_teleop.parts.base import Base, BaseSettings  # noqa: E402
from feather_teleop.parts.neck import NeckSettings, StickNeck  # noqa: E402
from feather_teleop.teleop import QuestTeleop  # noqa: E402

BENT = [0.3, 0.2, -0.4, 0.8, 0.3, -0.5, 0.2]   # a posture of the stand's arm, by joint


def reading(**buttons):
    return Reading({}, {**Headset().latest().buttons, **buttons}, 0.0)


def test_base_and_neck(tmp_path, monkeypatch):
    navigation, teensy = stand.base_stand(tmp_path / "base", monkeypatch)
    base = Base(navigation, BaseSettings(**SETTINGS["base"]), tick_divisor=2)
    base.step(reading(leftJS=(0.0, 1.0)), 0.04)
    navigation._thread_step()
    assert all(teensy.speed(motor) > 0 for motor in navigation.drive_motors)   # forward: every wheel drives
    base.step(None, 0.04)
    navigation._thread_step()
    assert [teensy.speed(motor) for motor in navigation.drive_motors] == [0.0] * 3
    base.close()
    assert not navigation._enabled

    neck, _ = stand.neck_stand(tmp_path / "neck", monkeypatch)
    part = StickNeck(neck, NeckSettings(speed=1.0, pitch_sign=1, yaw_sign=-1), tick_divisor=1)
    part.start(None)
    for _ in range(200):   # held long: never past the range the SDK accepts
        record = part.step(reading(rightJS=(0.0, -1.0)), 0.05)
    assert record["neck"][neck.pitch_motor.config.name] == pytest.approx(neck.describe().joints[0].position[0])


def stand_teleop(tmp_path, monkeypatch, parts=("arms",)):
    # A session on the stand's arms: its joints, no body to keep clear of, demonstrations in tmp_path.
    arms, _ = stand.arms_stand(tmp_path / "robot", monkeypatch)
    loader = SimpleNamespace(pin_import=arms.left_arm.pin_import, pin_model=arms.left_arm.pin_model)
    robot = SimpleNamespace(arms_system=arms, urdf_loader=loader, manipulation_system=None, torso_system=None, navigation_system=None, neck_system=None, camera_service=None)
    settings = copy.deepcopy(SETTINGS)
    joints = arms.left_arm.describe().joints
    settings.update(parts=list(parts), arms=["left"], headset="table", collision_margin=None, spatial_scale=1.0, episodes=str(tmp_path / "episodes"), start_posture={joint.urdf_joint: BENT[i] for i, joint in enumerate(joints)}, tool_offset={"left": [0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0]})
    settings["arm"].update(return_speed=1.0, speed_scale=1.0, floor=-10.0)
    settings["robot"]["body_boxes"] = []
    headset = Headset()
    return arms, QuestTeleop(robot, settings, headset=headset, say=lambda line: None), headset


def test_a_demonstration(tmp_path, monkeypatch):
    arms, teleop, headset = stand_teleop(tmp_path, monkeypatch)
    follower = teleop.followers[0]

    def ticks(count, each=None):
        for i in range(count):
            if each:
                each(i)
            teleop.loop.tick(headset.latest())
            time.sleep(follower.tick)

    with stand.ticking(arms.left_arm), stand.ticking(arms.right_arm):
        ticks(3)
        assert teleop.loop.episode is None and arms.left_arm.control_mode.value == "targets"   # idle until A
        headset.pressed = {"A"}
        ticks(1)   # blocks while the arm travels to its start posture
        headset.pressed = set()
        assert arms.left_arm.control_mode.value == "follow" and follower.at_start_posture(0.02)

        start = follower.tool_pose()[0]
        headset.grip["l"] = True
        ticks(160, lambda i: headset.move("l", np.array([0.0, 0.0, -0.03]) / 25) if 1 <= i <= 25 else None)
        measured = np.array(arms.left_arm.read_state().position) * follower.direction
        assert follower.ik.pose(measured)[0] == pytest.approx(start + [0.03, 0.0, 0.0], abs=5e-3)   # 3 cm ahead

        headset.grip["l"] = False
        headset.pressed = {"A"}
        ticks(1)
        assert teleop.loop.episode is None and arms.left_arm.control_mode.value == "targets"   # braked, holding

    folder = next((tmp_path / "episodes").iterdir())
    info = yaml.safe_load((folder / "episode.yaml").read_text())
    assert info["outcome"] == "success" and info["joints"]["left"][0] == arms.left_arm.describe().joints[0].urdf_joint
    assert len((folder / "samples.jsonl").read_text().splitlines()) == 161


def test_ready_and_home(tmp_path, monkeypatch):
    arms, teleop, _ = stand_teleop(tmp_path, monkeypatch)
    follower = teleop.followers[0]
    with stand.ticking(arms.left_arm), stand.ticking(arms.right_arm):
        teleop.ready()
        measured = np.array(arms.left_arm.read_state().position)
        assert measured == pytest.approx(np.array(BENT) * follower.direction, abs=0.05)
        teleop.home()
        assert np.array(arms.left_arm.read_state().position) == pytest.approx(np.zeros(7), abs=0.05)
    teleop.close()
