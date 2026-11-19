# The neck: moved by the right joystick (table), or following the operator's head (worn).
import math
from dataclasses import dataclass
from typing import Callable, Dict, Optional

import numpy as np

from feather_teleop.episode import Episode
from feather_teleop.headsets import HEAD, Reading
from feather_teleop.loop import Part
from feather_teleop.parts.sticks import RIGHT_STICK, stick

MIN_HEAD_HEADING = 0.1  # below this the head looks too nearly up or down to give a heading
GRAVITY_AT_REST = 5.0   # m/s^2, the least an accelerometer at rest can read


@dataclass
class NeckSettings:
    # The `neck` section of quest.yaml.
    speed: float       # rad/s
    pitch_sign: int    # +1 or -1, so that looking (or pushing the stick) up pitches the camera up
    yaw_sign: int      # +1 or -1, so that looking (or pushing the stick) left yaws the camera left


class _NeckJoint:
    # A neck motor's name, sign, and the range `set_targets()` accepts: its calibrated range within the config's.

    def __init__(self, motor, sign: int):
        if sign not in (1, -1):
            raise ValueError(f"a neck sign is {sign}; it must be 1 or -1")
        low, high = motor.command_angle_range()
        config_low, config_high = motor.config.angle_range
        self.name = motor.config.name
        self.sign = sign
        self.low = max(low, config_low)
        self.high = min(high, config_high)

    def clamp(self, angle: float) -> float:
        return min(self.high, max(self.low, angle))


class StickNeck(Part):
    # The right joystick moves the neck (table headset): up and down pitch it, left and right yaw it.
    #
    # Args:
    #     neck: The SDK neck system.
    #     settings: The neck settings.
    #     tick_divisor: Run on every n-th loop tick.
    name = "neck"

    def __init__(self, neck, settings: NeckSettings, tick_divisor: int):
        self.neck = neck
        self.settings = settings
        self.tick_divisor = tick_divisor
        self.pitch = _NeckJoint(neck.pitch_motor, settings.pitch_sign)
        self.yaw = _NeckJoint(neck.yaw_motor, settings.yaw_sign)
        self.target = {}

    def start(self, episode: Episode) -> None:
        state = self.neck.read_state()
        measured = dict(zip(state.names, state.position))
        self.target = {joint.name: joint.clamp(measured[joint.name]) for joint in (self.pitch, self.yaw)}
        self.neck.set_targets(self.target, dict.fromkeys(self.target, self.settings.speed))

    def step(self, reading: Optional[Reading], dt: float) -> dict:
        moved = stick(reading, RIGHT_STICK)
        if moved is not None:
            left, up = moved
            step = self.settings.speed * dt
            for joint, amount in ((self.pitch, up), (self.yaw, left)):
                self.target[joint.name] = joint.clamp(self.target[joint.name] + joint.sign * amount * step)
        self.neck.set_targets(self.target)
        return {"neck": dict(self.target)}


def head_angles(pose: np.ndarray):
    # Return how far (rad) a pose's x axis points up from the horizon and left of the frame's x, or None.
    #
    # None when it points too nearly straight up or down to give a heading.
    x, y, z = pose[:3, 0].tolist()
    level = math.hypot(x, y)
    if level < MIN_HEAD_HEADING:
        return None
    return math.atan2(z, level), math.atan2(y, x)


def camera_elevation(cameras) -> Callable[[], float]:
    # Return a function giving the stereo camera's elevation above the horizon (rad), from its accelerometer.
    #
    # At rest the accelerometer reads straight up, in the camera's axes (+z forward).
    def elevation() -> float:
        ax, ay, az = cameras.get_imu_data()
        size = math.sqrt(ax * ax + ay * ay + az * az)
        if size < GRAVITY_AT_REST:
            raise ValueError(f"the stereo camera's accelerometer reads {size:.1f} m/s^2, so it is not at rest")
        return math.asin(max(-1.0, min(1.0, az / size)))
    return elevation


class HeadNeck(Part):
    # The neck follows the operator's head (worn headset), within each motor's range and `speed`.
    #
    # Yaw is relative to where the head faces at the start. Pitch is absolute when `elevation` is given, else relative too. Each tick it sets `headset.look`, so the page can place the picture where it was taken.
    #
    # Args:
    #     neck: The SDK neck system.
    #     headset: The WornHeadset.
    #     settings: The neck settings.
    #     elevation: A function giving the camera's elevation (rad), as `camera_elevation()` makes, or None.
    name = "neck"

    def __init__(self, neck, headset, settings: NeckSettings, elevation: Optional[Callable[[], float]] = None):
        self.neck = neck
        self.headset = headset
        self.speed = settings.speed
        self.elevation = elevation
        self.joints = (_NeckJoint(neck.pitch_motor, settings.pitch_sign), _NeckJoint(neck.yaw_motor, settings.yaw_sign))
        self.speeds = {joint.name: settings.speed for joint in self.joints}
        self.target: Dict[str, float] = {}
        self.neutral: Dict[str, float] = {}   # the neck's angles that match the head's reference direction
        self.reference = None                 # the head's (up, left) reference, rad
        self.level = None                     # rad, the pitch motor's angle at which the camera looks level
        self._epoch = None

    def start(self, episode: Episode) -> None:
        measured = self._measured()
        self.target = {joint.name: joint.clamp(measured[joint.name]) for joint in self.joints}
        self.reference = None
        self.headset.look = None
        if self.elevation is not None:
            pitch = self.joints[0]
            self.level = measured[pitch.name] - pitch.sign * self.elevation()

    def _measured(self) -> Dict[str, float]:
        state = self.neck.read_state()
        return dict(zip(state.names, state.position.tolist()))

    def step(self, reading: Optional[Reading], dt: float) -> dict:
        angles = head_angles(reading.poses[HEAD]) if reading is not None and HEAD in reading.poses else None
        if angles is not None:   # else the neck holds
            epoch = self.headset.epoch
            if self.reference is None or epoch != self._epoch:
                self._take_reference(angles, epoch)
            self._move_toward(angles, self.speed * dt)
        self.neck.set_targets(self.target, self.speeds)
        self._tell_look()
        return {"head": angles}

    def _take_reference(self, angles, epoch) -> None:
        # Match the head's direction now to the neck's, at the start or after the tracking frame moved.
        self._epoch = epoch
        self.reference = angles
        self.neutral = dict(self.target)
        if self.level is not None:
            self.reference = (0.0, angles[1])
            self.neutral[self.joints[0].name] = self.level

    def _move_toward(self, angles, step: float) -> None:
        # Move each joint's target up to `step` (rad) toward where the head's `angles` point it.
        for joint, angle, reference in zip(self.joints, angles, self.reference):
            turned = math.remainder(angle - reference, math.tau)   # the short way round
            wanted = joint.clamp(self.neutral[joint.name] + joint.sign * turned)
            current = self.target[joint.name]
            self.target[joint.name] = current + max(-step, min(step, wanted - current))

    def _tell_look(self) -> None:
        # Set `headset.look` from how far the neck has measurably turned from its neutral.
        if self.reference is None:
            return
        try:
            measured = self._measured()
        except (ValueError, RuntimeError):   # no telemetry this tick: the page fixes the picture to the head
            self.headset.look = None
            return
        self.headset.look = tuple(reference + (measured[joint.name] - self.neutral[joint.name]) / joint.sign for joint, reference in zip(self.joints, self.reference))

    def stop(self) -> None:
        self.reference = None
        self.headset.look = None
