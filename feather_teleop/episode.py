# The record of one demonstration: a folder holding samples.jsonl (one line per loop tick) and episode.yaml.
#
# The cameras record into the same folder as `<name>.h264` plus `<name>.jsonl` (each frame's time); `to_mp4()` muxes each into an MP4 when the demonstration ends.
import json
import shutil
import threading
import time
from datetime import datetime
from fractions import Fraction
from pathlib import Path

import yaml

SUCCESS, FAILURE, INTERRUPTED = "success", "failure", "interrupted"
MICROSECONDS = Fraction(1, 1_000_000)   # the MP4s' time base


class Episode:
    # Writes one demonstration.
    #
    # Args:
    #     root: The folder that holds every demonstration.
    #     task: A description of the task, saved with it.
    #     parts: Names of the parts that are running.

    def __init__(self, root, task: str, parts):
        now = datetime.now().astimezone()
        # Milliseconds: B then A can start the next demonstration within the same second.
        self.path = Path(root) / now.strftime("%Y%m%d-%H%M%S-%f")[:-3]
        self.path.mkdir(parents=True)
        # A was pressed now, before the move to the start posture; the recorded motion starts at the first sample.
        self.info = {"task": task, "parts": list(parts), "start": now.isoformat(timespec="seconds")}
        self._samples = open(self.path / "samples.jsonl", "w", encoding="utf-8")

    def add(self, **fields) -> None:
        # Append one tick. Numpy values are converted to plain numbers and lists.
        self._samples.write(json.dumps({"t": time.time(), **fields}, default=_plain) + "\n")

    def end(self, outcome: str, say=print) -> Path:
        # Close the record with its outcome (SUCCESS, FAILURE or INTERRUPTED) and return its folder.
        #
        # The camera recordings are converted to MP4 in the background.
        self._samples.close()
        self.info.update(outcome=outcome, end=datetime.now().astimezone().isoformat(timespec="seconds"))
        (self.path / "episode.yaml").write_text(yaml.safe_dump(self.info, sort_keys=False), encoding="utf-8")
        convert_in_background(self.path, say)
        return self.path

    def discard(self) -> None:
        # Remove a demonstration that never started.
        self._samples.close()
        shutil.rmtree(self.path)


def _plain(value):
    # json.dumps fallback for numpy scalars and arrays.
    return value.tolist() if hasattr(value, "tolist") else float(value)


def to_mp4(h264) -> Path:
    # Write `<name>.mp4` beside `<name>.h264` without re-encoding, and remove the `.h264`.
    #
    # Frames keep their times from `<name>.jsonl`, so a dropped frame lengthens the one before it.
    #
    # Raises:
    #     ValueError: There are no frames, or the stream and the frame list disagree on how many. Nothing is removed then.
    import av  # PyAV comes with the [robot] extra only

    h264 = Path(h264)
    mp4 = h264.with_suffix(".mp4")
    partial = mp4.with_suffix(".mp4.part")
    times_ns = _frame_times(h264.with_suffix(".jsonl"))
    if not times_ns:
        raise ValueError(f"{h264.name}: no frames")

    frames = 0
    try:
        with av.open(str(h264), format="h264") as source, av.open(str(partial), mode="w", format="mp4") as out:
            stream = source.streams.video[0]
            video = out.add_stream_from_template(stream)
            video.time_base = MICROSECONDS
            for packet in source.demux(stream):
                if packet.size == 0:   # the demuxer's end-of-stream flush
                    continue
                if frames < len(times_ns):
                    packet.stream = video
                    # There are no B-frames, so decode order is display order.
                    packet.pts = packet.dts = (times_ns[frames] - times_ns[0]) // 1000
                    packet.time_base = MICROSECONDS
                    out.mux(packet)
                frames += 1
        if frames != len(times_ns):
            raise ValueError(f"{h264.name}: {frames} frames in the stream, {len(times_ns)} in the frame list")
        partial.replace(mp4)
    finally:
        partial.unlink(missing_ok=True)
    h264.unlink()
    return mp4


def _frame_times(jsonl: Path) -> list:
    # Each frame's camera time (ns), from a recording's frame list.
    lines = jsonl.read_text(encoding="utf-8").splitlines()
    return [json.loads(line)["camera_ns"] for line in lines if line]


def convert_folder(folder, say=print) -> list:
    # Convert every camera recording in a demonstration's folder. Returns the MP4s made.
    #
    # A recording that fails to convert is reported and its `.h264` kept.
    made = []
    for h264 in sorted(Path(folder).glob("*.h264")):
        if h264.stat().st_size == 0 or not h264.with_suffix(".jsonl").exists():
            continue   # a camera that never delivered
        try:
            made.append(to_mp4(h264))
        except Exception as error:
            say(f"{h264.name} was not converted ({type(error).__name__}: {error}); the .h264 is kept")
    return made


def convert_in_background(folder, say=print) -> threading.Thread:
    # Run `convert_folder()` on its own thread, so the loop is not held up for the seconds it takes.
    thread = threading.Thread(target=convert_folder, args=(folder, say), name="episode-mp4")
    thread.start()
    return thread
