# The worn headset's picture, and its debug readout.
import time
from typing import List, Optional

from feather_teleop.control.follower import ArmFollower
from feather_teleop.episode import Episode
from feather_teleop.headsets import Reading
from feather_teleop.loop import Part

DEBUG_EVERY = 0.5      # s between two debug readouts sent to the page
RESTART_AFTER = 2.0    # s at least between two restarts of a stalled stream
FAULT_WIDTH = 90       # characters of each fault line the readout shows


class Display(Part):
    # The worn headset's picture during a demonstration: the stereo camera, plus a wrist camera while its arm's `wrist_view` is on.
    #
    # Start it first, so the camera service budgets the stream and the recording together. A stalled stream is restarted.
    #
    # Args:
    #     headset: The WornHeadset.
    #     video: The ServiceVideo, or None for no picture.
    #     followers: The arms' followers, whose `wrist_view` shows their wrist camera.
    #     say: Called with each message for the operator.
    #     debug: Send the page its debug readout.
    #     clock: Returns the time in seconds.
    name = "display"

    def __init__(self, headset, video=None, followers: List[ArmFollower] = (), say=print, debug: bool = False, clock=time.monotonic):
        self.headset = headset
        self.video = video
        self.followers = list(followers)
        self.say = say
        self.debug = debug
        self.clock = clock
        self.running = False
        self._failure = None       # the video failure last told to the operator
        self._restarted = None     # when the stream was last started again
        self._debug_at = None      # when the last readout was sent
        self._ticks = 0            # loop ticks since then
        headset.set_status(phase="ready")
        if debug:
            headset.set_debug({"faults": [], "hz": None})

    def start(self, episode: Episode) -> None:
        self.headset.set_status(phase="active")
        for follower in self.followers:
            follower.wrist_view = False   # each demonstration starts with the stereo picture alone
        if self.video is not None:
            self.video.start()   # raises if over the encoders' budget, so the demonstration does not start
        self.running = True

    def step(self, reading: Optional[Reading], dt: float) -> dict:
        if self.video is not None:
            for follower in self.followers:
                try:
                    self.video.show(follower.side, follower.wrist_view)
                except ValueError as refused:   # over the encoders' budget: said, and the double press undone
                    follower.wrist_view = False
                    self.say(f"headset video: no {follower.side} wrist view: {refused}")
            self._watch_video()
        if self.debug:
            self._tell_debug()
        return {}

    def _watch_video(self) -> None:
        # Tell the operator when the picture stops or comes back, and start a stopped stream again.
        failure = self.video.failure()
        if failure != self._failure:   # told once; the arms go on without the picture
            if failure or self._failure:
                self.say(f"headset video: {failure}" if failure else "headset video: back")
            self._failure = failure
        now = self.clock()
        if failure is None or (self._restarted is not None and now - self._restarted < RESTART_AFTER):
            return
        self._restarted = now
        try:
            self.video.restart()   # the service does not start a stalled encoder again by itself
            self.say("headset video: started again")
        except (ValueError, RuntimeError, OSError) as error:   # over the budget, or the service gone: tried again later
            self.say(f"headset video: could not start again ({type(error).__name__}: {error})")

    def faults(self) -> List[str]:
        # What is wrong now, one short line each: the video's fault, and per arm a refused step, joints held back and a gripper given up on.
        found = []
        failure = None if self.video is None else self.video.failure()
        if failure:
            found.append(f"video: {failure}")
        for follower in self.followers:
            if follower.refused:
                found.append(f"{follower.side} arm refused: {follower.refused}")
            if follower.held_back:
                found.append(f"{follower.side} held back: {', '.join(follower.held_back)}")
            if follower.gripper.fault:
                found.append(f"{follower.side} gripper off: {follower.gripper.fault}")
        return [line[:FAULT_WIDTH] for line in found]

    def _tell_debug(self) -> None:
        now = self.clock()
        self._ticks += 1
        if self._debug_at is None:
            self._debug_at, self._ticks = now, 0
        elif now - self._debug_at >= DEBUG_EVERY:
            self.headset.set_debug({"faults": self.faults(), "hz": round(self._ticks / (now - self._debug_at), 1)})
            self._debug_at, self._ticks = now, 0

    def stop(self) -> None:
        self.running = False
        if self.video is not None:
            self.video.stop()
        self.headset.set_status(phase="ready")

    def close(self) -> None:
        if self.video is not None:
            self.video.close()
