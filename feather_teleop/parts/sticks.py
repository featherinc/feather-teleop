# The joysticks, as the base and the neck read them.
import math
from typing import Optional

from feather_teleop.headsets import STICK, Reading

STICK_DEADZONE = 0.15  # fraction of full joystick deflection that is ignored
STICK_RATE = 20.0      # Hz, how often the base and the stick-driven neck are commanded
LEFT_STICK, RIGHT_STICK = STICK["l"], STICK["r"]


def apply_deadzone(x: float, y: float, zone: float):
    # Apply a radial deadzone to a stick's axes, rescaled so its edge is 0 and full deflection is 1.
    deflection = math.hypot(x, y)
    if deflection <= zone:
        return 0.0, 0.0
    scale = min(1.0, (deflection - zone) / (1.0 - zone)) / deflection
    return x * scale, y * scale


def stick(reading: Optional[Reading], name: str):
    # Return a stick's (left, up) with the deadzone applied, or None without a reading.
    if reading is None or name not in reading.buttons:
        return None
    right, up = reading.buttons[name]   # the Quest's sticks give + right
    return apply_deadzone(-right, up, STICK_DEADZONE)
