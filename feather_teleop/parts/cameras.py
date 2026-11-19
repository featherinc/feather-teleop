# The cameras: recorded into each demonstration's folder.
from typing import Optional

from feather_teleop.episode import Episode
from feather_teleop.headsets import Reading
from feather_teleop.loop import Part


class Cameras(Part):
    # Records every camera of the SDK's camera service; a camera that fails mid-demonstration ends it.
    #
    # Args:
    #     cameras: The SDK camera service.
    #     say: Called with each message for the operator.
    name = "cameras"

    def __init__(self, cameras, say=print):
        self.cameras = cameras
        self.say = say
        self._recording = False
        self._known_faults = set()

    def _recording_faults(self) -> dict:
        # Return the faults that affect a recording: a camera not delivering, or its recorder failing.
        names = set(self.cameras.names)
        return {name: why for name, why in self.cameras.faults.items() if name in names or name.startswith("recording:")}

    def start(self, episode: Episode) -> None:
        self._known_faults = set(self._recording_faults())
        if self._known_faults:
            self.say(f"recording without {', '.join(sorted(self._known_faults))} (not delivering)")
        self.cameras.start_recording(episode.path)
        self._recording = True

    def step(self, reading: Optional[Reading], dt: float) -> dict:
        new_faults = {name: why for name, why in self._recording_faults().items() if name not in self._known_faults}
        if new_faults:
            raise RuntimeError("; ".join(f"{name}: {why}" for name, why in new_faults.items()))
        return {}

    def stop(self) -> None:
        if not self._recording:
            return
        self._recording = False   # cleared first, so a stop that raises is not retried
        for name, result in self.cameras.stop_recording().items():
            if result["failure"]:
                self.say(f"camera {name}: {result['failure']}")
            elif result["missed"]:
                self.say(f"camera {name}: {result['missed']} frames were not recorded")
