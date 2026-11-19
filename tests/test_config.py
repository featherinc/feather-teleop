# The SDK config a session builds from quest.yaml.
import copy
from pathlib import Path

import pytest

from feather_teleop import config
from feather_teleop.config import sdk_config


def settings(tmp_path, parts=("arms", "torso"), headset="table", **robot):
    s = copy.deepcopy(config.load())
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
    with pytest.raises(FileNotFoundError, match="robot.urdf"):
        sdk_config(settings(tmp_path, urdf=str(tmp_path / "missing.urdf")))


@pytest.mark.parametrize("change, why", [({"headset": "pocket"}, "headset is 'pocket'"), ({"headset": "worn", "mirror": True}, "mirror is for the table headset"), ({"parts": ["base"]}, "must include arms"), ({"parts": ["arms", "neck"]}, "the robot's config has no neck"), ({"arms": ["middle"]}, "arms is"), ({"arms": ["left"]}, "no left arm")])
def test_session_settings(change, why):
    from types import SimpleNamespace

    from feather_teleop.teleop import QuestTeleop
    robot = SimpleNamespace(arms_system=SimpleNamespace(left_arm=None, right_arm=object()), navigation_system=None, neck_system=None, torso_system=None, camera_service=None)
    s = {**copy.deepcopy(config.load()), "parts": ["arms"], "arms": ["right"], "mirror": False, **change}
    with pytest.raises(ValueError, match=why):
        QuestTeleop(robot, s)   # refused before the headset is opened
