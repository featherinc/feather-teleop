"""What the camera process does with the frames besides capturing them: records every camera as H.264 at full quality
(`Recording`), streams every camera live as RTP to one computer (`Stream`), and serves every camera as MJPEG to a
browser (`WebView`). Each reads the frames from the cameras' slots on threads of its
own, as any consumer would, so none of them can hold up capture.

A camera whose stream is its own picture (not scaled down, not thinned: `camera_encoding.shares_encoder`) is encoded
once for both (`SharedEncoders`): the recording's file and the stream's packets are the same access units, instead of
the camera being handed over and encoded twice (for a 3840x1080 stereo picture: two encoders and two 16.6 MB copies a
frame)."""
import json
import random
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

from feathersdk.robot.services.camera_encoding import (
    RTP_CLOCK, Encoded, H264Encoder, is_keyframe, rtp_packets, scaled_size, shares_encoder)
from feathersdk.robot.services.camera_frames import FrameSlot
from feathersdk.utils.feathertypes import Any, Callable, Dict, List, Tuple

WAIT = 0.5
"""Seconds an output waits for a camera's next frame before looking whether it should stop."""
RECORD_DELAY = 1.0
"""Seconds a recording encoder may be behind before a frame is dropped (counted as missed): NVENC is 59-75 ms
behind on the robot, so only an encoder that has stalled loses frames."""
STREAM_DELAY = 0.15
"""Seconds a stream encoder may be behind before a frame is dropped: twice NVENC's own latency, so a live view
never queues more than that."""


TAIL = 1.0
"""Seconds a recording on a shared encoder waits at its stop for its last frame to come out of the encoder."""


def _size(slot: FrameSlot) -> Tuple[int, int]:
    return slot.shape[1], slot.shape[0]


class _SharedEncoder:
    """One camera encoded at its own size and rate for every sink attached: a sink gets each access unit
    (`sink.unit(encoded)`) from the first keyframe after it attached, and is told if the encoder stops
    (`sink.failed(why)`). Frames are dropped while the encoder is behind by more than a recording allows
    (`RECORD_DELAY`) if one is attached, else by more than a live view does (`STREAM_DELAY`)."""

    def __init__(self, name: str, slot: FrameSlot, encoder: str, fps: int, bitrate: int, idr_interval: int) -> None:
        self.name, self.failure = name, None
        self._slot = slot
        self._lock = threading.Lock()
        self._sinks: Dict[Any, bool] = {}
        """Each sink, and whether it still waits for a keyframe."""
        self._stopping = threading.Event()
        self._encoder = H264Encoder(encoder, _size(slot), _size(slot), fps, bitrate, idr_interval, self._unit,
                                    STREAM_DELAY)
        self._thread = threading.Thread(target=self._run, name=f"encode_{name}", daemon=True)
        self._thread.start()

    def attach(self, sink: Any) -> None:
        with self._lock:
            self._sinks[sink] = True
            self._set_delay()

    def detach(self, sink: Any) -> bool:
        """Takes `sink` off; True if none is left."""
        with self._lock:
            self._sinks.pop(sink, None)
            self._set_delay()
            return not self._sinks

    def hand_over(self) -> List[Any]:
        """Takes every sink off and returns them: they move to the encoder that replaces this one."""
        with self._lock:
            sinks, self._sinks = list(self._sinks), {}
        return sinks

    def _set_delay(self) -> None:
        recording = any(getattr(sink, "records", False) for sink in self._sinks)
        self._encoder.max_delay = RECORD_DELAY if recording else STREAM_DELAY

    def _unit(self, encoded: Encoded) -> None:
        key = is_keyframe(encoded.unit)
        with self._lock:
            sinks = [sink for sink, waiting in self._sinks.items() if key or not waiting]
            for sink in sinks:
                self._sinks[sink] = False
        for sink in sinks:
            try:
                sink.unit(encoded)
            except Exception as error:  # noqa: BLE001  one sink's trouble (a full disk) is its own
                sink.failed(str(error))

    def _run(self) -> None:
        buffer = np.empty(self._slot.raw_shape, np.uint8)  # BGRA as stored: the encoder takes it unconverted
        last = self._slot.seq()
        try:
            while not self._stopping.is_set():
                frame = self._slot.wait_raw(last, WAIT, out=buffer)
                if frame is None:
                    continue
                last = frame.seq
                self._encoder.encode(frame.image, frame.seq, frame.camera_ns, frame.host_ns)
        except Exception as error:  # noqa: BLE001  the encoder stopped: every sink is told
            self.failure = str(error)
            with self._lock:
                sinks = list(self._sinks)
            for sink in sinks:
                sink.failed(self.failure)

    def close(self) -> None:
        self._stopping.set()
        self._thread.join(timeout=5.0)
        self._encoder.close()


class SharedEncoders:
    """The cameras encoded once for their recording and their stream (`covers(name)`: `camera_encoding.
    shares_encoder`), each by one `_SharedEncoder` that runs while a sink has it: `take(name, sink)` attaches one
    (starting the encoder if it isn't running, or has stopped), `release(name, sink)` takes it off (the encoder ends
    with its last sink, on a thread of its own: its flush keeps nobody waiting). Encoded at the higher of the
    recording's and the stream's bitrates and the shorter of their keyframe intervals, so the stream is as good as it
    was alone and a stream that joins waits at most that interval for its first picture. A recording that joins a
    running encoder gets a new one in its place (its sinks move over): an encoder's first unit is a keyframe, so the
    file starts with the recording's first frame, not up to an interval later; a stream meanwhile misses the frames
    of the change."""

    def __init__(self, slots: Dict[str, FrameSlot], fps: Dict[str, int], encoder: str, stream_fps: int,
                 stream_height: int, bitrate: int, idr_interval: int) -> None:
        self._slots, self._fps, self._encoding = slots, fps, (encoder, bitrate, idr_interval)
        self._covered = {name for name, slot in slots.items()
                         if shares_encoder(_size(slot), fps[name], stream_fps, stream_height)}
        self._lock = threading.Lock()
        self._running: Dict[str, _SharedEncoder] = {}

    def covers(self, name: str) -> bool:
        return name in self._covered

    def take(self, name: str, sink: Any) -> None:
        with self._lock:
            shared = self._running.get(name)
            before = []
            if shared is not None and (shared.failure is not None or getattr(sink, "records", False)):
                before = [] if shared.failure is not None else shared.hand_over()  # stopped, or a recording joins
                self._end(self._running.pop(name))
                shared = None
            if shared is None:
                encoder, bitrate, idr_interval = self._encoding
                shared = self._running[name] = _SharedEncoder(name, self._slots[name], encoder, self._fps[name],
                                                              bitrate, idr_interval)
            for one in (*before, sink):
                shared.attach(one)

    def release(self, name: str, sink: Any) -> None:
        with self._lock:
            shared = self._running.get(name)
            if shared is not None and shared.detach(sink):
                self._end(self._running.pop(name))

    @staticmethod
    def _end(shared: _SharedEncoder) -> None:
        threading.Thread(target=shared.close, name=f"encode_end_{shared.name}", daemon=True).start()

    def stop(self) -> None:
        """Ends every encoder still running (the process is ending)."""
        with self._lock:
            running, self._running = list(self._running.values()), {}
        for shared in running:
            shared.close()


class _Rtp:
    """Access units out as RTP/H.264 packets to `target` over UDP, without waiting for the network."""

    def __init__(self, target: Tuple[str, int]) -> None:
        self.sent_frames = 0
        self._target = target
        self._ssrc, self._seq = random.getrandbits(32), random.getrandbits(16)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)

    def send(self, encoded: Encoded) -> None:
        timestamp = encoded.camera_ns * RTP_CLOCK // 1_000_000_000
        packets = rtp_packets(encoded.unit, self._seq, timestamp, self._ssrc)
        self._seq = (self._seq + len(packets)) & 0xFFFF
        for packet in packets:
            try:
                self._socket.sendto(packet, self._target)
            except OSError:  # the send buffer is full or the network is down: a lost packet (BlockingIOError too)
                pass
        self.sent_frames += 1

    def close(self) -> None:
        self._socket.close()


class _CameraRecording:
    """One camera to `<name>.h264` and `<name>.jsonl` in `folder`."""

    def __init__(self, name: str, slot: FrameSlot, folder: Path, encoder: str, fps: int, bitrate: int,
                 idr_interval: int, report: Callable[[tuple], None]) -> None:
        self.name, self.written, self.failure = name, 0, None
        self.first = slot.seq()  # the camera's last frame before the start
        self.last = float("inf")  # its last frame before the stop, which stop() sets
        self._slot, self._report = slot, report
        self._video = open(folder / f"{name}.h264", "wb")
        self._index = open(folder / f"{name}.jsonl", "w", encoding="utf-8")
        try:
            self._encoder = H264Encoder(encoder, _size(slot), _size(slot), fps, bitrate, idr_interval, self._write,
                                        RECORD_DELAY)
        except Exception:
            self._video.close()
            self._index.close()
            raise

    def _write(self, encoded: Encoded) -> None:
        self._video.write(encoded.unit)
        self._index.write(json.dumps({"frame": self.written, "seq": encoded.seq, "camera_ns": encoded.camera_ns,
                                      "host_ns": encoded.host_ns}) + "\n")
        self.written += 1

    def run(self, stopping: threading.Event) -> None:
        buffer = np.empty(self._slot.raw_shape, np.uint8)  # BGRA as stored: the encoder takes it unconverted
        last = self.first
        try:
            while not stopping.is_set():
                frame = self._slot.wait_raw(last, WAIT, out=buffer)
                if frame is None:
                    continue
                if frame.seq > self.last:  # taken after the stop
                    return
                last = frame.seq
                self._encoder.encode(frame.image, frame.seq, frame.camera_ns, frame.host_ns)
        except Exception as error:
            self.failure = str(error)
            self._report(("fault", f"recording:{self.name}", self.failure))

    def close(self) -> Dict[str, Any]:
        """Flushes the encoder into the files, closes them and returns the camera's summary."""
        self._encoder.close()  # its last units reach the file before the file closes
        self._video.close()
        self._index.close()
        return {"frames": self.written, "missed": self.last - self.first - self.written, "failure": self.failure}


class _SharedRecording:
    """One camera to `<name>.h264` and `<name>.jsonl` in `folder` from its shared encoder: the units from the first
    keyframe after the start (at once when the recording started the encoder; else up to a keyframe interval later,
    and the frames before it count as missed) to the camera's last frame before the stop."""
    records = True

    def __init__(self, name: str, slot: FrameSlot, folder: Path, shared: SharedEncoders,
                 report: Callable[[tuple], None]) -> None:
        self.name, self.written, self.failure = name, 0, None
        self.first = slot.seq()
        self.last = float("inf")
        self._shared, self._report = shared, report
        self._lock, self._done, self._closed = threading.Lock(), threading.Event(), False
        self._video = open(folder / f"{name}.h264", "wb")
        self._index = open(folder / f"{name}.jsonl", "w", encoding="utf-8")
        try:
            shared.take(name, self)
        except Exception:
            self._video.close()
            self._index.close()
            raise

    def unit(self, encoded: Encoded) -> None:
        with self._lock:
            if self._closed or encoded.seq > self.last:  # taken after the stop
                self._done.set()
                return
            self._video.write(encoded.unit)
            self._index.write(json.dumps({"frame": self.written, "seq": encoded.seq, "camera_ns": encoded.camera_ns,
                                          "host_ns": encoded.host_ns}) + "\n")
            self.written += 1
            if encoded.seq >= self.last:
                self._done.set()

    def failed(self, why: str) -> None:
        if self.failure is None:
            self.failure = why
            self._report(("fault", f"recording:{self.name}", why))
        self._done.set()

    def run(self, stopping: threading.Event) -> None:
        stopping.wait()
        if self.failure is None:
            self._done.wait(TAIL)  # its last frame is still in the encoder: about 70 ms on the robot

    def close(self) -> Dict[str, Any]:
        self._shared.release(self.name, self)
        with self._lock:
            self._closed = True
            self._video.close()
            self._index.close()
        return {"frames": self.written, "missed": self.last - self.first - self.written, "failure": self.failure}


class Recording:
    """Every camera to `directory`: `<name>.h264` (an Annex B stream, plays in any H.264 decoder: `ffplay`) and
    `<name>.jsonl`, a line per frame in the file: its index, sequence number, the camera's and the host's time.
    `stop()` flushes the encoders and returns per camera the frames written, the camera's frames between the start
    and the stop that aren't in the file (`missed`: the recorder was busy with an earlier one, its encoder was behind,
    or that camera's recording failed) and the `failure` that ended it early, or None."""

    def __init__(self, slots: Dict[str, FrameSlot], fps: Dict[str, int], directory: str, encoder: str, bitrate: int,
                 idr_interval: int, report: Callable[[tuple], None], shared: SharedEncoders = None) -> None:
        folder = Path(directory)
        if not folder.is_dir():
            raise ValueError(f"no folder {folder} to record into")
        self._slots = slots
        self._stopping = threading.Event()
        self._cameras: List[Any] = []
        try:
            for name, slot in slots.items():
                if shared is not None and shared.covers(name):  # one encoder for it and its stream
                    self._cameras.append(_SharedRecording(name, slot, folder, shared, report))
                    continue
                self._cameras.append(_CameraRecording(name, slot, folder, encoder, fps[name], bitrate, idr_interval,
                                                      report))
        except Exception:
            for camera in self._cameras:
                camera.close()
            raise
        self._threads = [threading.Thread(target=c.run, args=(self._stopping,), name=f"record_{c.name}", daemon=True)
                         for c in self._cameras]
        for thread in self._threads:
            thread.start()

    def stop(self) -> Dict[str, Dict[str, Any]]:
        for camera in self._cameras:
            camera.last = self._slots[camera.name].seq()
        self._stopping.set()
        for thread in self._threads:
            thread.join(timeout=5.0)
        return {camera.name: camera.close() for camera in self._cameras}


class _CameraStream:
    """One camera's live RTP stream to `target`: its frames thinned to `fps`, scaled to `height` and encoded."""

    def __init__(self, name: str, slot: FrameSlot, target: Tuple[str, int], encoder: str, fps: int, height: int,
                 bitrate: int, idr_interval: int, report: Callable[[tuple], None]) -> None:
        self.name = name
        self._slot, self._report = slot, report
        self._period = 1.0 / fps  # once here, not per frame
        self._rtp = None
        self._encoder = H264Encoder(encoder, _size(slot), scaled_size(_size(slot), height), fps, bitrate,
                                    idr_interval, self._send, STREAM_DELAY)
        self._rtp = _Rtp(target)  # after the encoder: no socket left open if it fails to start

    @property
    def sent_frames(self) -> int:
        return 0 if self._rtp is None else self._rtp.sent_frames

    def _send(self, encoded: Encoded) -> None:
        if self._rtp is not None:
            self._rtp.send(encoded)

    def run(self, stopping: threading.Event) -> None:
        buffer = np.empty(self._slot.raw_shape, np.uint8)  # BGRA as stored: the encoder takes it unconverted
        last, due = self._slot.seq(), 0.0
        try:
            while not stopping.is_set():
                frame = self._slot.wait_raw(last, WAIT, out=buffer)
                if frame is None:
                    continue
                last = frame.seq
                now = time.monotonic()
                if now < due - 0.25 * self._period:  # thinned to fps; a quarter period early is on time
                    continue
                due = max(due + self._period, now)
                self._encoder.encode(frame.image, frame.seq, frame.camera_ns, frame.host_ns)
        except Exception as error:
            self._report(("fault", f"stream:{self.name}", str(error)))
        finally:
            self.close()

    def close(self) -> None:
        self._encoder.close()
        self._rtp.close()


class _SharedStream:
    """One camera's live RTP stream to `target` from its shared encoder: the camera's own picture at its own rate."""

    def __init__(self, name: str, target: Tuple[str, int], shared: SharedEncoders,
                 report: Callable[[tuple], None]) -> None:
        self.name = name
        self._shared, self._report = shared, report
        self._failed = threading.Event()
        self._rtp = _Rtp(target)
        try:
            shared.take(name, self)
        except Exception:
            self._rtp.close()
            raise

    @property
    def sent_frames(self) -> int:
        return self._rtp.sent_frames

    def unit(self, encoded: Encoded) -> None:
        self._rtp.send(encoded)

    def failed(self, why: str) -> None:
        if not self._failed.is_set():
            self._failed.set()
            self._report(("fault", f"stream:{self.name}", why))

    def run(self, stopping: threading.Event) -> None:
        try:
            while not stopping.is_set() and not self._failed.is_set():
                stopping.wait(WAIT)
        finally:
            self.close()

    def close(self) -> None:
        self._shared.release(self.name, self)
        self._rtp.close()


class Stream:
    """Cameras live to `host`, camera `i` (in the service's order) on UDP port `port + i` whether or not the others
    stream, at `fps` (never above a camera's own) and scaled to `height` (never up). `show(names)` sets which cameras
    stream, starting and ending each one's encoder: one not watched costs nothing. `camera_encoding.receiver_command(
    port)` plays one."""

    def __init__(self, slots: Dict[str, FrameSlot], camera_fps: Dict[str, int], host: str, port: int, encoder: str,
                 fps: int, height: int, bitrate: int, idr_interval: int, report: Callable[[tuple], None],
                 names: List[str], shared: SharedEncoders = None) -> None:
        self._shared = shared
        self.ports = {name: port + i for i, name in enumerate(slots)}
        self._slots, self._camera_fps, self._host, self._report = slots, camera_fps, host, report
        self._encoding = (encoder, fps, height, bitrate, idr_interval)
        self._running: Dict[str, Tuple[_CameraStream, threading.Thread, threading.Event]] = {}
        self._ended: List[Tuple[_CameraStream, threading.Thread]] = []
        """Streams `show()` ended: each closes its encoder on its own thread (a flush can take a while), joined at
        `stop()`, so a change never keeps the caller waiting."""
        self._sent: Dict[str, int] = {}
        """Frames sent by the ended streams whose threads have finished, per camera."""
        self.show(names)

    def show(self, names: List[str]) -> Dict[str, int]:
        """Streams exactly `names` from now on (ending the others); returns their ports. Every new encoder starts
        here, so one that can't is the request's error (the cameras already started keep streaming)."""
        unknown = [name for name in names if name not in self.ports]
        if unknown:
            raise ValueError(f"no camera {', '.join(unknown)}: the cameras are {', '.join(self.ports)}")
        for name in [name for name, (_, thread, _) in self._running.items()
                     if name not in names or not thread.is_alive()]:  # unwanted, or ended by a fault: restarts
            self._end(name)
        for name in names:
            if name not in self._running:
                self._start(name)
        return {name: self.ports[name] for name in names}

    def _start(self, name: str) -> None:
        encoder, fps, height, bitrate, idr_interval = self._encoding
        if self._shared is not None and self._shared.covers(name):  # one encoder for it and its recording
            camera = _SharedStream(name, (self._host, self.ports[name]), self._shared, self._report)
        else:
            camera = _CameraStream(name, self._slots[name], (self._host, self.ports[name]), encoder,
                                   min(fps, self._camera_fps[name]), height, bitrate, idr_interval, self._report)
        stopping = threading.Event()
        thread = threading.Thread(target=camera.run, args=(stopping,), name=f"stream_{name}", daemon=True)
        self._running[name] = (camera, thread, stopping)
        thread.start()

    def _end(self, name: str) -> None:
        camera, thread, stopping = self._running.pop(name)
        stopping.set()  # its run() closes the encoder and the socket
        self._ended.append((camera, thread))
        self._collect(wait=False)

    def _collect(self, wait: bool) -> None:
        """Counts the frames of the ended streams that have finished (all of them, waiting, with `wait`) and forgets
        them: a session toggling a camera for hours keeps no list of them."""
        for camera, thread in self._ended:
            if wait:
                thread.join(timeout=5.0)
            if not thread.is_alive():
                self._sent[camera.name] = self._sent.get(camera.name, 0) + camera.sent_frames
        self._ended = [(camera, thread) for camera, thread in self._ended if thread.is_alive()]

    def stop(self) -> Dict[str, int]:
        """Ends every camera's stream; returns the frames each camera that streamed sent."""
        for name in list(self._running):
            self._end(name)
        self._collect(wait=True)
        return dict(self._sent)


class WebView:
    """Every camera as MJPEG in a browser: `http://<the robot>:<port>/` shows them in a grid (the first camera across
    the top, the rest below), `/<name>.mjpg` is one stream. Each open page gets each camera's new frames, scaled to
    `height` and JPEG-encoded, up to `fps` a second; a camera without a new frame for `WAIT` (not open, or being
    reopened) shows a "no signal" picture until it delivers."""

    def __init__(self, slots: Dict[str, FrameSlot], port: int, fps: float, height: int, quality: int) -> None:
        import cv2
        view = self
        names = list(slots)
        sizes = {name: scaled_size(_size(slot), height) for name, slot in slots.items()}
        no_signal = {name: _no_signal_part(cv2, size, quality) for name, size in sizes.items()}  # once, not per wait

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args) -> None:
                pass

            def do_GET(self) -> None:
                name = self.path.strip("/")
                if name == "":
                    tiles = "".join(f'<figure style="margin:0;{"grid-column:1/-1;" if i == 0 else ""}">'
                                    f'<img src="/{n}.mjpg" style="width:100%;display:block">'
                                    f'<figcaption>{n}</figcaption></figure>' for i, n in enumerate(names))
                    body = ('<!doctype html><title>Robot cameras</title>'
                            '<body style="margin:8px;background:#111;color:#ccc;font:14px sans-serif">'
                            f'<div style="display:grid;grid-template-columns:repeat({max(1, len(names) - 1)},1fr);'
                            f'gap:8px">{tiles}</div></body>').encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if not name.endswith(".mjpg") or name[:-5] not in slots:
                    self.send_error(404, f"no camera {name!r}: {', '.join(names)}")
                    return
                name = name[:-5]
                slot = slots[name]
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                buffer = np.empty(slot.raw_shape, np.uint8)
                last, period, due = 0, 1.0 / fps, time.monotonic()
                size = sizes[name]
                try:
                    while not view._stopping.is_set():
                        frame = slot.wait_raw(last, WAIT, out=buffer)
                        if frame is None:
                            self.wfile.write(no_signal[name])
                            continue
                        last = frame.seq
                        small = cv2.resize(frame.image, size, interpolation=cv2.INTER_AREA) \
                            if size[1] < slot.shape[0] else frame.image
                        # BGRA -> BGR after the scaling: the conversion runs on the small picture
                        self.wfile.write(_jpeg_part(cv2, cv2.cvtColor(small, cv2.COLOR_BGRA2BGR), quality))
                        due = max(due + period, time.monotonic() - period)
                        time.sleep(max(0.0, due - time.monotonic()))
                except (BrokenPipeError, ConnectionResetError):
                    pass  # the browser left

        self._stopping = threading.Event()
        self._server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever, name="web_view", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stopping.set()
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=5.0)


def _jpeg_part(cv2: Any, image: np.ndarray, quality: int) -> bytes:
    """`image` as one part of an MJPEG stream."""
    ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
            + jpeg.tobytes() + b"\r\n")


def _no_signal_part(cv2: Any, size: Tuple[int, int], quality: int) -> bytes:
    """A dark picture of `size` (width, height) saying "no signal", as one part of an MJPEG stream."""
    image = np.full((size[1], size[0], 3), 32, np.uint8)
    scale = size[1] / 240
    (width, height), _ = cv2.getTextSize("no signal", cv2.FONT_HERSHEY_SIMPLEX, scale, max(1, round(2 * scale)))
    cv2.putText(image, "no signal", ((size[0] - width) // 2, (size[1] + height) // 2), cv2.FONT_HERSHEY_SIMPLEX,
                scale, (200, 200, 200), max(1, round(2 * scale)))
    return _jpeg_part(cv2, image, quality)
