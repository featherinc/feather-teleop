# The demonstration loop: idle until A, then step every part each tick until A (success) or B (failure).
import time
from typing import List, Optional

from feathersdk.logging import log_event

from feather_teleop.episode import FAILURE, INTERRUPTED, SUCCESS, Episode
from feather_teleop.headsets import Reading

RELEASE_AFTER = 0.2   # s; an older sample is stale: the arms hold and the base stops
FAIL_AFTER = 2.0      # s without a sample before a running demonstration ends as a failure


class Part:
    # One thing the loop drives: the arms, the base, the cameras, ...
    #
    # Raising from `start()` refuses the demonstration; raising from `step()` ends it as a failure.
    name = "part"
    tick_divisor = 1   # `step()` runs on every n-th loop tick

    def start(self, episode: Episode) -> None:
        # Called when a demonstration starts.
        ...

    def step(self, reading: Optional[Reading], dt: float) -> dict:
        # Called every `tick_divisor` loop ticks during a demonstration.
        #
        # Args:
        #     reading: The newest sample, or None while it is stale.
        #     dt: The nominal time since this part's last step (s).
        #
        # Returns:
        #     The fields to record for this tick.
        return {}

    def stop(self) -> None:
        # Called when the demonstration ends, and after a failed start (so also without one).
        ...

    def close(self) -> None:
        # Called when the session is over.
        ...


class DemonstrationLoop:
    # Runs demonstrations until interrupted.
    #
    # Args:
    #     reader: The headset, anything with `latest()` returning a `Reading`.
    #     parts: The parts to drive. They are started and stepped in this order, and stopped in reverse.
    #     period: The loop's period (s).
    #     episodes: The folder demonstrations are saved in.
    #     task: A description saved with each demonstration.
    #     release_after: A sample older than this (s) is stale. Parts get no reading then.
    #     fail_after: After this long (s) without a sample, a running demonstration ends as a failure.
    #     say: Called with each message for the operator.

    def __init__(self, reader, parts: List[Part], period: float, episodes, task: str, release_after: float = RELEASE_AFTER, fail_after: float = FAIL_AFTER, say=print):
        self.reader = reader
        self.parts = list(parts)
        self.period = period
        self.episodes = episodes
        self.task = task
        self.release_after = release_after
        self.fail_after = fail_after
        self.say = say
        self.episode: Optional[Episode] = None
        self._previous_buttons: Optional[dict] = None
        self._ticks = 0

    def run(self) -> None:
        # Loop until Ctrl+C (a notebook's stop button). An open demonstration is saved as interrupted.
        self.say("idle: press A to start a demonstration")
        due = time.monotonic()
        waiting = False
        try:
            while True:
                reading = self.reader.latest()
                if reading.age > self.fail_after:
                    if self.episode is not None:
                        self._end(FAILURE, "headset lost")
                    if not waiting:
                        self.say("waiting for the headset: is it awake and tracking, and is its app (table) or page (worn) running?")
                    waiting = True
                else:
                    waiting = False
                    self.tick(reading)
                # After a late tick the next one follows at once, not a full period later.
                due = max(due + self.period, time.monotonic())
                time.sleep(max(0.0, due - time.monotonic()))
        except KeyboardInterrupt:
            if self.episode is not None:
                self._end(INTERRUPTED, "interrupted")
            self.say("stopped")
        except BaseException:   # SystemExit and other non-Exception errors still save the demonstration
            if self.episode is not None:
                self._end(FAILURE, "error")
            raise

    def tick(self, reading: Reading) -> None:
        # Run one loop tick on `reading`. `run()` calls this at the loop's period.
        reading = self._handle_buttons(reading)
        if self.episode is not None:
            self._step_parts(reading)

    def _handle_buttons(self, reading: Reading) -> Reading:
        # Start or end a demonstration on A or B. Returns the reading to step the parts with.
        buttons = self._buttons(reading)
        if self.episode is None:
            if self._pressed("A", buttons):
                self._begin()
                # Starting can take seconds (the torso, the move to the start posture): read afresh.
                reading = self.reader.latest()
                buttons = self._buttons(reading)
        elif self._pressed("B", buttons):   # B wins if both are pressed
            self._end(FAILURE, "B pressed")
        elif self._pressed("A", buttons):
            self._end(SUCCESS, "A pressed")
        self._previous_buttons = buttons
        return reading

    def _step_parts(self, reading: Reading) -> None:
        # Step the parts due this tick and record what they return. A part that raises ends the demonstration.
        fresh = None if self._is_stale(reading) else reading
        record = {}
        for part in self.parts:
            if self._ticks % part.tick_divisor:
                continue
            try:
                record.update(part.step(fresh, part.tick_divisor * self.period))
            except Exception as error:
                self._end(FAILURE, f"{part.name}: {type(error).__name__}: {error}")
                return
        self.episode.add(age=reading.age, buttons=self._buttons(reading), **record)
        self._ticks += 1

    def _is_stale(self, reading: Reading) -> bool:
        return reading.age > self.release_after

    def _buttons(self, reading: Reading) -> Optional[dict]:
        # The reading's buttons, or None if it is stale.
        return None if self._is_stale(reading) else reading.buttons

    def _pressed(self, button: str, buttons: Optional[dict]) -> bool:
        # Return True on the tick a button goes down.
        #
        # It must have been reported up on the previous tick, so a controller that reconnects with it held is not a press.
        before = self._previous_buttons
        if not buttons or not before or button not in before:
            return False
        return bool(buttons.get(button)) and not before[button]

    def _begin(self) -> None:
        episode = Episode(self.episodes, self.task, [part.name for part in self.parts])
        try:
            for part in self.parts:
                part.start(episode)
        except BaseException as error:   # includes Ctrl+C
            self._stop_parts()
            episode.discard()
            if not isinstance(error, Exception):
                raise
            self.say(f"not started ({type(error).__name__}: {error}); press A to try again")
            return
        self.episode = episode
        self._ticks = 0
        log_event("episode", "quest_teleop", action="start", path=str(episode.path))
        self.say(f"recording {episode.path}: A ends it as a success, B as a failure")

    def _end(self, outcome: str, why: str) -> None:
        self._stop_parts()
        path = self.episode.end(outcome, self.say)
        self.episode = None
        log_event("episode", "quest_teleop", action="end", outcome=outcome, path=str(path))
        self.say(f"demonstration saved as {outcome} ({why}): {path}")

    def _stop_parts(self) -> None:
        # Stop every part, the last started first, even if one fails to.
        #
        # The arms start last, so they stop first: nothing moves while a slower part (the recorders) finishes.
        for part in reversed(self.parts):
            try:
                part.stop()
            except Exception as error:
                self.say(f"{part.name} did not stop cleanly: {type(error).__name__}: {error}")
