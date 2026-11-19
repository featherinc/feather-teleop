# The arms follow the controllers.
import time
from typing import List, Optional

import numpy as np

from feather_teleop.control.follower import AT_START, ArmFollower
from feather_teleop.episode import Episode
from feather_teleop.headsets import Reading
from feather_teleop.loop import Part

SLACK = 2.0    # the move to the start posture may take SLACK times its full-speed time, plus SETTLE
SETTLE = 5.0   # s


class Arms(Part):
    # The arms follow the controllers. Each demonstration starts with the torso up and the arms in their start posture.
    #
    # Args:
    #     followers: One `ArmFollower` per driven arm.
    #     reader: The headset. B, or losing the stream, stops the move to the start posture.
    #     release_after: s; an older sample counts as losing the stream.
    #     torso: The SDK torso, or None. The arms' models follow its height, so hand positions stay in the base frame.
    #     torso_speed: Fraction of the lift's top speed to raise the torso at the start, or None to leave it.
    #     say: Called with each message for the operator.
    name = "arms"

    def __init__(self, followers: List[ArmFollower], reader, release_after: float, torso=None, torso_speed: Optional[float] = None, say=print):
        self.followers = followers
        self.reader = reader
        self.release_after = release_after
        self.torso = torso
        self.torso_speed = torso_speed
        self.say = say
        self._said = {}
        self._sync_torso_height()

    def _sync_torso_height(self) -> None:
        # Tell each arm's model where the torso is.
        if self.torso is not None:
            for follower in self.followers:
                follower.ik.follow_torso(self.torso)

    def start(self, episode: Episode) -> None:
        if self.torso is not None and self.torso_speed is not None:
            raise_torso(self.torso, self.torso_speed)
            self._sync_torso_height()
        try:
            for follower in self.followers:
                follower.start()
            self._go_to_start_posture()
        except BaseException:   # also on Ctrl+C, so no arm is left moving to a target
            self.stop()
            raise
        episode.info["joints"] = {follower.side: [joint.urdf_joint for joint in follower.joints] for follower in self.followers}
        self._said = {}

    def _go_to_start_posture(self) -> None:
        # Move every arm to its start posture through follow mode, so the SDK's collision check applies.
        #
        # Raises:
        #     RuntimeError: An arm did not get there.
        followers = self.followers
        if all(follower.at_start_posture(AT_START) for follower in followers):
            return
        self.say("arms to their start posture (B stops them)")
        # s the slowest joint needs at its return speed
        longest = max(float(np.max(np.abs(follower.q - follower.start_posture) / follower.return_speed)) for follower in followers)
        for follower in followers:
            follower.return_to_start(stoppable_by_grip=False)

        deadline = time.monotonic() + SLACK * longest + SETTLE
        period = followers[0].tick
        due = time.monotonic()
        while not self._arrived(deadline):
            self._sync_torso_height()
            for follower in followers:
                follower.step()
            due = max(due + period, time.monotonic())
            time.sleep(max(0.0, due - time.monotonic()))

    def _arrived(self, deadline: float) -> bool:
        # Return True once no arm is still returning; raise if the move was refused, stopped or too slow.
        refusals = [f"{follower.side}: {follower.refused}" for follower in self.followers if follower.refused]
        if refusals:
            raise RuntimeError("held back on the way to the start posture: " + "; ".join(refusals))
        if not any(follower.returning for follower in self.followers):
            return True
        reading = self.reader.latest()
        if reading.age > self.release_after or reading.buttons.get("B"):
            raise RuntimeError("stopped on the way to the start posture: B pressed, or the headset lost")
        if time.monotonic() > deadline:
            raise RuntimeError("the arms took too long to reach the start posture")
        return False

    def step(self, reading: Optional[Reading], dt: float) -> dict:
        # The followers read the headset and keep time themselves.
        self._sync_torso_height()
        record = {}
        for follower in self.followers:
            follower.step()
            record[follower.side] = self._record(follower)
            self._report(follower)
        return {"arms": record}

    @staticmethod
    def _record(follower: ArmFollower) -> dict:
        # Return one arm's fields for this tick: angles in rad (motor frame), positions in m (base frame).
        direction = follower.direction
        return {
            "command": follower.q,                     # the follow target
            "measured": follower.measured,
            "hand_command": follower.ik.pose(follower.q * direction)[0],
            "hand_measured": follower.ik.pose(follower.measured * direction)[0],
            "target": follower.target,                 # None without the grip
            "following": follower.controller.gripping,
            "gripper": follower.gripper.closure,       # 0 open, 1 closed
            "refused": follower.refused,               # the SDK's reason, or None
            "held_back": follower.held_back,           # joints limited by the leash
            "position_locked": follower.position_locked,
        }

    def _report(self, follower: ArmFollower) -> None:
        # Tell the operator when an arm's state changes.
        if follower.controller.grip_untracked:
            state = "grip held, but the headset does not see that controller"
        elif follower.refused:
            state = f"held back: {follower.refused}"
        elif follower.held_back:
            state = f"{', '.join(follower.held_back)} not following; is something in the way?"
        elif follower.gripper.fault:
            state = f"gripper off: {follower.gripper.fault}"
        elif follower.returning:
            state = "returning to its start posture"
        elif not follower.controller.gripping:
            state = "holding"
        elif follower.position_locked:
            state = "following, position locked"
        else:
            state = "following"
        if state != self._said.get(follower.side):
            self._said[follower.side] = state
            self.say(f"{follower.side} arm: {state}")

    def stop(self) -> None:
        for follower in self.followers:
            follower.stop()


def raise_torso(torso, speed: float) -> None:
    # Raise the torso to the top at `speed` (fraction of its top speed) and wait, giving the arms the most room.
    #
    # Raises:
    #     TimeoutError: The lift stopped short of the top. It is stopped there.
    torso.go_to(torso.describe().height_range[1], velocity=speed * torso.max_velocity_mm_s, wait=True)
