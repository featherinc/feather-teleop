# The SDK config a session builds from quest.yaml.
import copy
from pathlib import Path

import pytest

from fakes import TEST_ROBOT, robot_settings

from feather_teleop import config
from feather_teleop.config import sdk_config


def settings(tmp_path, parts=("arms", "torso"), headset="table", **robot):
    s = copy.deepcopy(robot_settings())
    urdf = tmp_path / "robot.urdf"
    urdf.touch()
    s["robot"] = {**s["robot"], "urdf": str(urdf), **robot}
    s.update(parts=list(parts), headset=headset)
    return s


def test_sdk_config(tmp_path):
    conf = sdk_config(settings(tmp_path))
    assert conf.arms_system_config and conf.torso_system_config and conf.manipulation_system_config
    assert conf.navigation_system_config is None and conf.neck_system_config is None
    assert conf.camera_service_config is None and conf.odometry_service_config is None
    assert conf.set_signal_handlers is False and conf.urdf_path == Path(tmp_path / "robot.urdf")

    conf = sdk_config(settings(tmp_path, grippers=["right"], headset="worn"))
    assert conf.manipulation_system_config.left_gripper is None and conf.manipulation_system_config.right_gripper
    assert conf.camera_service_config is not None   # worn: the picture needs the cameras
    assert sdk_config(settings(tmp_path, grippers=[])).manipulation_system_config is None

    conf = sdk_config(settings(tmp_path, parts=("arms", "cameras"), cameras={"serial": 12345678}))
    assert conf.camera_service_config.serial == 12345678
    with pytest.raises(ValueError, match="not camera service fields"):
        sdk_config(settings(tmp_path, parts=("arms", "cameras"), cameras={"lens": 1}))
    with pytest.raises(FileNotFoundError, match="robots.test.urdf"):
        sdk_config(settings(tmp_path, urdf=str(tmp_path / "missing.urdf")))


def test_every_robot_value_is_entered_by_hand(tmp_path):
    path = tmp_path / "quest.yaml"
    path.write_text(config.DEFAULT_FILE.read_text())
    with pytest.raises(ValueError, match="robot: None; set it to one of c_sample, d_sample"):
        config.load(path)   # the robot is not chosen for the user
    for robot in ("c_sample", "d_sample"):   # nor any of its values
        path.write_text(config.DEFAULT_FILE.read_text().replace("robot: null", f"robot: {robot}", 1))
        with pytest.raises(ValueError, match=f"robots.{robot}: measure and fill in " + ", ".join(config.ROBOT_KEYS)):
            config.load(path)
    with pytest.raises(ValueError, match="robots.mine: measure and fill in body_clear$"):
        config.resolve({**config.read(), "robot": "mine", "robots": {"mine": {**TEST_ROBOT, "body_clear": None}}})
    assert config.read()["table"]["port"]   # the laptop reads the file without a robot


@pytest.mark.parametrize("change, why", [({"headset": "pocket"}, "headset is 'pocket'"), ({"headset": "worn", "mirror": True}, "mirror is for the table headset"), ({"parts": ["base"]}, "must include arms"), ({"parts": ["arms", "neck"]}, "the robot's config has no neck"), ({"arms": ["middle"]}, "arms is"), ({"arms": ["left"]}, "no left arm")])
def test_session_settings(change, why):
    from types import SimpleNamespace

    from feather_teleop.teleop import QuestTeleop
    robot = SimpleNamespace(arms_system=SimpleNamespace(left_arm=None, right_arm=object()), navigation_system=None, neck_system=None, torso_system=None, camera_service=None)
    s = {**copy.deepcopy(robot_settings()), "parts": ["arms"], "arms": ["right"], "mirror": False, **change}
    with pytest.raises(ValueError, match=why):
        QuestTeleop(robot, s)   # refused before the headset is opened
