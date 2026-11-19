# Tests run without a robot or a headset.
#
# Some drive FeatherSDK's real systems on the SDK's test stand, which is not part of an installed SDK: set FEATHERSDK to the `feathersdk` folder of an SDK checkout to run them. Without it they are skipped.
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
if "FEATHERSDK" in os.environ:
    SDK = Path(os.environ["FEATHERSDK"]).resolve()
    sys.path[:0] = [str(SDK / "src"), str(SDK)]


def pytest_configure(config):
    from feathersdk.logging import ERROR, LoggingConfig, init_logging
    init_logging(LoggingConfig(disable_file_logging=True, console_level=ERROR, log_init_info=False))
