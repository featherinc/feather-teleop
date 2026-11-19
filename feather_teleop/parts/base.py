# The base: driven by the left joystick.
from dataclasses import dataclass
from typing import Optional

from feather_teleop.headsets import Reading
from feather_teleop.loop import Part
from feather_teleop.parts.sticks import LEFT_STICK, RIGHT_STICK, stick


@dataclass
class BaseSettings:
    # The `base` section of quest.yaml.
    linear_speed: float    # m/s
    angular_speed: float   # rad/s


class Base(Part):
    # The left joystick drives the base; X and Y turn it, or the right stick when `turn_with_stick`.
    #
    # Args:
    #     navigation: The SDK navigation system.
    #     settings: The base settings.
    #     tick_divisor: Run on every n-th loop tick.
    #     turn_with_stick: Turn with the right stick (sideways) instead of X and Y.
    name = "base"

    def __init__(self, navigation, settings: BaseSettings, tick_divisor: int, turn_with_stick: bool = False):
        self.navigation = navigation
        self.settings = settings
        self.tick_divisor = tick_divisor
        self.turn_with_stick = turn_with_stick
        navigation.enable()
        # Drive at once, scaled by each wheel's heading error, instead of waiting for the wheels to turn.
        navigation.set_strict_directions(False)

    def step(self, reading: Optional[Reading], dt: float) -> dict:
        velocity = (0.0, 0.0, 0.0)
        drive = stick(reading, LEFT_STICK)
        if drive is not None:
            left, forward = drive
            s = self.settings
            velocity = (forward * s.linear_speed, left * s.linear_speed, self._turn(reading) * s.angular_speed)
        self.navigation.set_velocity(*velocity)
        return {"base": velocity}   # vx, vy (m/s) and yaw rate (rad/s), in the robot's frame

    def _turn(self, reading: Reading) -> float:
        # Return the turn command, +1 left to -1 right.
        if self.turn_with_stick:
            return (stick(reading, RIGHT_STICK) or (0.0, 0.0))[0]
        return int(bool(reading.buttons.get("X"))) - int(bool(reading.buttons.get("Y")))

    def stop(self) -> None:
        self.navigation.set_velocity(0.0, 0.0, 0.0)

    def close(self) -> None:
        self.navigation.disable()
