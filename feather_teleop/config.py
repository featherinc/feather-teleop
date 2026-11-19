# Loads config/quest.yaml, and builds the FeatherSDK config a session starts the robot with.
from pathlib import Path

import yaml

# The repository root; holds only for an editable install (`pip install -e .`).
ROOT = Path(__file__).resolve().parent.parent
DEFAULT_FILE = ROOT / "config" / "quest.yaml"


def load(path=DEFAULT_FILE) -> dict:
    # Read the settings file into a dict.
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def folder(name) -> Path:
    # Resolve a folder named in the settings: absolute as given, otherwise inside the repository.
    path = Path(name).expanduser()
    return path if path.is_absolute() else ROOT / path


def sdk_config(settings: dict):
    # Return the SDK's config for the robot's version, with only the session's systems and `robot` laid over it.
    #
    # Raises:
    #     FileNotFoundError: The SDK has no config of that version, or the URDF is not there.
    #     ValueError: A camera field is not a camera service field.
    from feathersdk.configs import load_user_config  # the robot's computer has the SDK; a laptop need not

    robot = settings["robot"]
    systems = _systems(settings)
    conf = load_user_config(robot["config"]).only(*systems)
    # Without the SDK's Ctrl+C handler, Ctrl+C (a notebook's stop button) ends the loop instead of shutting down.
    update = {"set_signal_handlers": False}

    if robot["urdf"] is not None:
        urdf = Path(robot["urdf"]).expanduser()
        if not urdf.is_file():
            raise FileNotFoundError(f"robot.urdf: no file at {urdf}")
        update["urdf_path"] = urdf
    grippers = conf.manipulation_system_config
    if grippers is not None:
        update["manipulation_system_config"] = grippers.model_copy(update={f"{side}_gripper": None for side in ("left", "right") if side not in robot["grippers"]})
    if robot["cameras"] and conf.camera_service_config is not None:
        update["camera_service_config"] = _with_cameras(conf.camera_service_config, robot["cameras"])
    return conf.model_copy(update=update)


def _systems(settings: dict) -> set:
    # The SDK systems a session with these settings uses.
    systems = {"arms", *settings["parts"]}
    if settings["headset"] == "worn":
        systems.add("cameras")   # the picture in the headset
    if settings["robot"]["grippers"]:
        systems.add("grippers")
    return systems


def _with_cameras(own, fields: dict):
    # The camera service config `own`, with `fields` laid over it.
    unknown = fields.keys() - type(own).model_fields.keys()
    if unknown:
        raise ValueError(f"robot.cameras: {sorted(unknown)} are not camera service fields")
    from feathersdk.robot.services.camera_service import CameraResolution
    if "mono_cameras" in fields:   # the settings name a resolution; the SDK wants the enum
        fields = {**fields, "mono_cameras": {name: {**camera, "resolution": CameraResolution[camera["resolution"]]} for name, camera in fields["mono_cameras"].items()}}
    return type(own).model_validate({**own.model_dump(), **fields})
