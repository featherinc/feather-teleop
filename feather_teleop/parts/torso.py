# The torso: following the hands' height (table), or moved by Y and X (worn).
import time
from dataclasses import dataclass
from typing import List, Optional

from feathersdk.robot.actuator_systems.collision import CollisionError

from feather_teleop.control.follower import ArmFollower
from feather_teleop.headsets import Reading
from feather_teleop.loop import Part

TORSO_RETRY = 0.25   # s before a torso move that did not happen is sent again


@dataclass
class TorsoSettings:
    # The `torso` section of quest.yaml sets the band; hysteresis and speed are tuned defaults.
    rise_above: float          # m, the torso rises while a wrist is higher than this above its shoulder
    sink_below: float          # m, and sinks while it is lower than this
    hysteresis: float = 0.05   # m, a move stops once the wrist is back inside the band by this much
    speed: float = 0.5         # fraction of the lift's top speed, also when it rises at the start


class Torso(Part):
    # The torso, moved by the hands' height (table headset) or, worn, by Y (up) and X (down).
    #
    # Rises while a gripping wrist is above its shoulder's band, sinks while below; never sinks with a hand near the floor.
    #
    # Args:
    #     torso: The SDK torso system.
    #     followers: The arms' followers.
    #     settings: The torso settings.
    #     buttons: Move with Y and X instead of following the hands.
    #     say: Called with each message for the operator.
    #     clock: Returns the time in seconds. Replaced in tests.
    name = "torso"

    def __init__(self, torso, followers: List[ArmFollower], settings: TorsoSettings, buttons: bool = False, say=print, clock=time.monotonic):
        self.torso = torso
        self.followers = followers
        self.settings = settings
        self.buttons = buttons
        self.say = say
        self.clock = clock
        self.speed = settings.speed * torso.max_velocity_mm_s   # mm/s
        self.velocity = 0.0                                     # the velocity last accepted by the SDK
        self._sent_at = 0.0
        self._refused_at: Optional[float] = None
        self._refusal: Optional[str] = None

    def direction_wanted(self, reading: Optional[Reading] = None) -> int:
        # Return +1 to rise, -1 to sink or 0 to stay. Arms that disagree keep it still.
        if self.buttons:
            if reading is None:
                return 0
            return int(bool(reading.buttons.get("Y"))) - int(bool(reading.buttons.get("X")))
        driven = [follower for follower in self.followers if follower.controller.gripping and not follower.returning]
        votes = [self._vote(follower) for follower in driven]
        if 1 in votes and -1 not in votes:
            return 1
        if -1 in votes and 1 not in votes and not any(self._near_floor(follower) for follower in driven):
            return -1
        return 0

    def _vote(self, follower: ArmFollower) -> int:
        # Return one arm's wish: +1 if its wrist is being sent above the band, -1 below, 0 inside.
        s = self.settings
        arm = follower.points()
        # The band is for the wrist but the target is for the tool point, so shift the target by their offset.
        wrist_to_tool = _tool_height(follower) - arm.wrist[2]
        wrist_target = arm.wrist[2] if follower.target is None else follower.target[2] - wrist_to_tool
        height = wrist_target - arm.shoulder[2]
        # A move under way continues until the wrist is back inside by the hysteresis.
        if height > s.rise_above - (s.hysteresis if self.velocity > 0 else 0.0):
            return 1
        if height < s.sink_below + (s.hysteresis if self.velocity < 0 else 0.0):
            return -1
        return 0

    def _near_floor(self, follower: ArmFollower) -> bool:
        # Return True if the tool point is within `hysteresis` of the arm's floor.
        return _tool_height(follower) < follower.settings.floor + self.settings.hysteresis

    def step(self, reading: Optional[Reading], dt: float) -> dict:
        state = self.torso.read_state()
        self._set_velocity(self.direction_wanted(reading) * self.speed, state.moving)
        return {"torso": {"velocity": self.velocity, "height": float(state.height)}}   # mm/s, mm

    def _set_velocity(self, velocity: float, moving: bool = True) -> None:
        # Send `velocity` (mm/s) if it is new; resend a nonzero one every TORSO_RETRY while the lift is still.
        now = self.clock()
        if velocity == self.velocity:
            if velocity == 0.0 or moving or now - self._sent_at < TORSO_RETRY:
                return
        elif velocity < 0.0 and self._refused_at is not None and now - self._refused_at < TORSO_RETRY:
            return   # a move down was refused a moment ago
        try:
            self.torso.update_velocity(velocity)
        except CollisionError as refused:   # the SDK will not lower the torso onto an arm
            if self.velocity != 0.0:
                self.torso.update_velocity(0.0)
            self.velocity = 0.0
            self._refused_at = now
            if str(refused) != self._refusal:
                self._refusal = str(refused)
                self.say(f"torso: {refused}")
            return
        self.velocity = velocity
        self._sent_at = now
        self._refused_at = None
        self._refusal = None

    def stop(self) -> None:
        self._refused_at = None
        self._set_velocity(0.0)


def _tool_height(follower: ArmFollower) -> float:
    # Return the height (m) of the arm's commanded tool point.
    return follower.ik.pose(follower.q * follower.direction)[0][2]
