# The demonstration loop with stand-in parts, and the record it writes, camera MP4s included.
import json
from fractions import Fraction

import numpy as np
import pytest
import yaml
from fakes import Headset

from feather_teleop import episode
from feather_teleop.headsets import Reading
from feather_teleop.loop import DemonstrationLoop, Part

TICK = 0.005   # s, the loop's period in these tests


class Recorder(Part):
    # A part that notes what the loop does to it, and can refuse to start or fail at a step.

    def __init__(self, name="recorder", fail_start=False, fail_at=None, stopped=None):
        self.name, self.fail_start, self.fail_at = name, fail_start, fail_at
        self.calls, self.steps, self.stopped = [], 0, stopped if stopped is not None else []

    def start(self, episode):
        self.calls.append("start")
        if self.fail_start:
            raise RuntimeError("not ready")

    def step(self, reading, dt):
        self.steps += 1
        if self.steps == self.fail_at:
            raise RuntimeError("broke")
        return {"count": self.steps, "stale": reading is None}

    def stop(self):
        self.calls.append("stop")
        self.stopped.append(self.name)


def run(tmp_path, script, parts):
    # Run the loop while `script(headset)`, a generator, changes the headset between ticks.
    #
    # Returns the demonstrations saved, each as (episode.yaml, samples), and what the loop said.
    headset, said = Headset(), []
    steps = script(headset)

    class Scripted:
        def latest(self):
            try:
                next(steps)
            except StopIteration:
                raise KeyboardInterrupt from None   # the notebook's stop button
            return headset.latest()

    DemonstrationLoop(Scripted(), parts, TICK, tmp_path, "a task", release_after=0.2, fail_after=2.0, say=said.append).run()
    saved = []
    for folder in sorted(path for path in tmp_path.iterdir() if path.is_dir()):
        samples = [json.loads(line) for line in (folder / "samples.jsonl").read_text().splitlines()]
        saved.append((yaml.safe_load((folder / "episode.yaml").read_text()), samples))
    return saved, said


def press(headset, button, ticks_after=3):
    yield   # a tick with the button up, so the next one counts as a press
    headset.pressed = {button}
    yield
    headset.pressed = set()
    for _ in range(ticks_after):
        yield


@pytest.mark.parametrize("ending, outcome", [("A", "success"), ("B", "failure"), (None, "interrupted")])
def test_outcomes(tmp_path, ending, outcome):
    stopped = []
    parts = [Recorder("cameras", stopped=stopped), Recorder("arms", stopped=stopped)]

    def script(headset):
        yield   # idle: nothing is recorded yet
        yield from press(headset, "A", ticks_after=5)
        if ending:
            yield from press(headset, ending)

    saved, said = run(tmp_path, script, parts)
    (info, samples), = saved
    assert info["outcome"] == outcome and info["task"] == "a task" and info["parts"] == ["cameras", "arms"]
    assert [s["count"] for s in samples] == list(range(1, len(samples) + 1)) and len(samples) >= 5
    assert parts[0].calls == ["start", "stop"] and stopped == ["arms", "cameras"]   # the arms stop first
    assert said[-1] == "stopped"


def test_failures(tmp_path):
    first, refusing = Recorder(), Recorder(fail_start=True)
    saved, said = run(tmp_path, lambda headset: press(headset, "A"), [first, refusing])
    assert saved == [] and first.calls == ["start", "stop"]   # a refused start leaves nothing behind
    assert any("not started (RuntimeError: not ready)" in line for line in said)

    saved, said = run(tmp_path / "failing", lambda headset: press(headset, "A", ticks_after=8), [Recorder(fail_at=3)])
    (info, samples), = saved
    assert info["outcome"] == "failure" and len(samples) == 2 and any("RuntimeError: broke" in line for line in said)


def test_stale_and_lost_stream(tmp_path):
    def script(headset):
        yield from press(headset, "A")
        headset.age = 0.5   # stale: the parts get no reading
        yield
        yield
        headset.age = 3.0   # lost: the demonstration ends
        yield
        yield

    saved, said = run(tmp_path, script, [Recorder()])
    (info, samples), = saved
    assert info["outcome"] == "failure" and samples[-1]["stale"] and not samples[0]["stale"]
    assert any("headset lost" in line for line in said) and any("waiting for the headset" in line for line in said)


def test_a_button_held_when_its_controller_reconnects_is_not_a_press(tmp_path):
    part = Recorder()

    def script(headset):
        real = headset.latest
        headset.latest = lambda: Reading(real().poses, {"X": False}, 0.0)   # the right controller is off
        yield
        yield
        headset.pressed = {"A"}
        headset.latest = real   # back, with A already held
        yield
        yield

    saved, _ = run(tmp_path, script, [part])
    assert saved == [] and part.calls == []


av = pytest.importorskip("av")


def recording(folder, name, frames, times_ns):
    # A raw H.264 file like the camera service writes (no B-frames), `frames` long, and its .jsonl.
    encoder = av.CodecContext.create("libx264", "w")
    encoder.width, encoder.height, encoder.pix_fmt = 64, 48, "yuv420p"
    encoder.time_base, encoder.options = Fraction(1, 30), {"bf": "0"}
    with open(folder / f"{name}.h264", "wb") as out:
        for i in range(frames):
            picture = av.VideoFrame.from_ndarray(np.full((48, 64, 3), (i * 7) % 255, np.uint8), format="rgb24")
            picture = picture.reformat(format="yuv420p")
            picture.pts = i
            for packet in encoder.encode(picture):
                out.write(bytes(packet))
        for packet in encoder.encode(None):
            out.write(bytes(packet))
    (folder / f"{name}.jsonl").write_text("".join(json.dumps({"frame": i, "camera_ns": int(t), "host_ns": 0}) + "\n" for i, t in enumerate(times_ns)))


def test_mp4(tmp_path):
    start, gaps = 1_700_000_000_000_000_000, [33_333_333] * 9
    gaps[4] = 100_000_000   # a dropped frame: 100 ms between two
    times = start + np.concatenate([[0], np.cumsum(gaps)])
    recording(tmp_path, "stereo", 10, times)
    recording(tmp_path, "right_wrist", 6, np.arange(5) * 33_333_333)   # the frame list is one short
    (tmp_path / "left_wrist.h264").write_bytes(b"")                     # a camera that never delivered
    (tmp_path / "left_wrist.jsonl").write_text("")
    said = []
    assert episode.convert_folder(tmp_path, say=said.append) == [tmp_path / "stereo.mp4"]
    assert not (tmp_path / "stereo.h264").exists() and (tmp_path / "stereo.jsonl").exists()
    with av.open(str(tmp_path / "stereo.mp4")) as source:
        stream = source.streams.video[0]
        shown = [float(packet.pts * stream.time_base) for packet in source.demux(stream) if packet.size]
    assert shown == pytest.approx(list((times - start) / 1e9), abs=2e-6)   # each frame at its camera's time
    assert (tmp_path / "right_wrist.h264").exists() and not list(tmp_path.glob("right_wrist.mp4*"))
    assert said == ["right_wrist.h264 was not converted (ValueError: right_wrist.h264: 6 frames in the stream, 5 in the frame list); the .h264 is kept"]
