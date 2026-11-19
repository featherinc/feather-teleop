import sys
import importlib.util
from feathersdk.utils.feathertypes import Optional, ModuleType, Path, PathLike, ensure
from feathersdk.utils.system import register_reset_system_function
from feathersdk.utils.constants import _set_in_configs
from .configurable import PyDBaseModel


_EXPORTED_CONFIG: Optional[PyDBaseModel] = None

_FEATHER_CONFIG_PACKAGE_NAME = "featherconf"


def load_configs(config_path: PathLike) -> PyDBaseModel:
    """Load the configs from the given directory."""
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file \"{config_path}\" does not exist")
    if not config_path.is_file():
        raise IsADirectoryError(f"Config file \"{config_path}\" is not a file")
    
    try:
        _set_in_configs(True)
    
        # A config loaded before in this process left its modules under the package. Drop them, or this config's
        # relative imports would get that config's (another robot version's) systems.
        prefix = _FEATHER_CONFIG_PACKAGE_NAME + "."
        for name in [n for n in sys.modules if n == _FEATHER_CONFIG_PACKAGE_NAME or n.startswith(prefix)]:
            del sys.modules[name]

        # Create a fake package to use relative imports
        package = ModuleType(_FEATHER_CONFIG_PACKAGE_NAME)
        package.__path__ = [str(config_path.parent)]
        sys.modules[_FEATHER_CONFIG_PACKAGE_NAME] = package

        # Build the config module
        module_name = f"{_FEATHER_CONFIG_PACKAGE_NAME}.{config_path.stem}"
        spec = ensure(importlib.util.spec_from_file_location(module_name, config_path))
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module

        # Set the module's package and file so relative imports work and exceptions are formatted correctly
        module.__package__ = _FEATHER_CONFIG_PACKAGE_NAME
        module.__file__ = str(config_path)

        ensure(spec.loader).exec_module(module)

        # Check the config was exported correctly
        if _EXPORTED_CONFIG is None:
            raise ValueError("FeatherConfig has not been exported")
        if not isinstance(_EXPORTED_CONFIG, PyDBaseModel):
            t1, t2 = PyDBaseModel.__name__, type(_EXPORTED_CONFIG).__name__
            raise TypeError(f"Expected exported config to be a subclass of `{t1}`, got `{t2}`")
        
        return _EXPORTED_CONFIG
    finally:
        _set_in_configs(False)


USER_CONFIGS = Path(__file__).resolve().parent / "user_configs"
"""The robot configs that ship with the SDK, one folder per robot version."""


def load_user_config(version: str) -> PyDBaseModel:
    """Load the robot config of a robot version that ships with the SDK (`user_configs/<version>/robot_conf.py`).

    Raises:
        FileNotFoundError: There is no config of that version. The message lists the versions there are.
    """
    path = USER_CONFIGS / version / "robot_conf.py"
    if not path.is_file():
        versions = sorted(folder.name for folder in USER_CONFIGS.iterdir() if (folder / "robot_conf.py").is_file())
        raise FileNotFoundError(f"No robot config of version {version!r}; there are {', '.join(versions)}")
    return load_configs(path)


def export_config(config: PyDBaseModel) -> None:
    """Used within feather configs to set the final config object when loaded"""
    global _EXPORTED_CONFIG
    _EXPORTED_CONFIG = config


def __on_system_reset_configs(forked: bool) -> None:
    """Reset the exported config to None when the system is reset."""
    global _EXPORTED_CONFIG
    _EXPORTED_CONFIG = None
register_reset_system_function(__on_system_reset_configs)
