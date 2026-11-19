"""H.264 for the camera service, without any camera: the encoder (a subprocess: NVENC through GStreamer on a Jetson,
libx264 through ffmpeg elsewhere) that turns frames into access units, splitting its byte stream at access-unit
delimiters, and RTP packetizing (RFC 6184) with the GStreamer line that plays a stream. Testable on any machine with
ffmpeg."""
import fcntl
import os
import select
import struct
import subprocess
import threading
import time
from collections import deque

import numpy as np

from typing import Deque

from feathersdk.utils.feathertypes import Callable, List, NamedTuple, Optional, Tuple

AUD = b"\x00\x00\x00\x01\x09"
"""An access unit delimiter NAL (type 9) behind a four-byte start code: both encoders write one before every frame."""
RTP_CLOCK = 90_000
"""RTP's video clock (Hz)."""
RTP_PAYLOAD_TYPE = 96
MTU_PAYLOAD = 1200
"""Bytes of H.264 in one RTP packet: under any path MTU, with room for IP, UDP and RTP headers."""
STALL = 2.0
"""Seconds without output, with frames in flight, after which an encoder counts as stopped: ten times NVENC's
slowest frame on the robot (0.2 s)."""


# ---- The byte stream ----------------------------------------------------------------------------------------------
def split_access_units(stream: bytearray) -> List[bytes]:
    """Removes every complete access unit from the front of `stream` and returns them in order. A unit runs from one
    delimiter to the next, so the last one stays in `stream` until the following delimiter arrives."""
    units: List[bytes] = []
    start = stream.find(AUD)
    if start < 0:
        return units
    del stream[:start]
    while True:
        end = stream.find(AUD, len(AUD))
        if end < 0:
            return units
        units.append(bytes(stream[:end]))
        del stream[:end]


def nal_units(unit: bytes) -> List[bytes]:
    """The NAL units of an Annex B access unit, start codes removed."""
    starts = []
    at = 0
    while True:
        at = unit.find(b"\x00\x00\x01", at)
        if at < 0:
            break
        starts.append(at + 3)
        at += 3
    nals = []
    for i, start in enumerate(starts):
        end = starts[i + 1] - 3 if i + 1 < len(starts) else len(unit)
        nal = unit[start:end]
        while nal.endswith(b"\x00"):  # the next start code's leading zero (a four-byte start code)
            nal = nal[:-1]
        if nal:
            nals.append(nal)
    return nals


# ---- The encoder --------------------------------------------------------------------------------------------------
ENCODERS = ("nvenc", "x264")
"""`nvenc`: the Jetson's hardware encoder through GStreamer (`nvv4l2h264enc`). `x264`: libx264 through ffmpeg, for a
machine without NVENC (tests, desktops)."""


def encoder_command(kind: str, size: Tuple[int, int], out_size: Tuple[int, int], fps: int, bitrate: int,
                    idr_interval: int) -> List[str]:
    """The encoder subprocess: BGRA frames of `size` (width, height) on stdin (the frame slot's own; the alpha byte
    is ignored), scaled to `out_size`, H.264 access units
    on stdout, each behind a delimiter, SPS and PPS before every IDR, no B-frames (one unit per frame, in order)."""
    (width, height), (out_width, out_height) = size, out_size
    if out_width % 2 or out_height % 2:
        raise ValueError(f"H.264 needs an even picture size, got {out_width}x{out_height}")
    if kind == "nvenc":
        return [
            "gst-launch-1.0", "-q", "fdsrc", "fd=0", "blocksize=65536", "!",
            # BGRx straight into the VIC (nvvidconv): colour conversion and scaling off the CPU
            "rawvideoparse", "format=bgrx", f"width={width}", f"height={height}", f"framerate={fps}/1",
            f"plane-strides=<{width * 4}>", f"frame-size={width * height * 4}", "!",
            "queue", "max-size-buffers=2", "max-size-bytes=0", "max-size-time=0", "!",
            "nvvidconv", "!", f"video/x-raw(memory:NVMM),format=NV12,width={out_width},height={out_height}", "!",
            "nvv4l2h264enc", f"bitrate={bitrate}", "control-rate=1", "maxperf-enable=1", "preset-level=1",
            "num-B-Frames=0", f"iframeinterval={idr_interval}", f"idrinterval={idr_interval}", "insert-sps-pps=1",
            "insert-aud=1", "!", "video/x-h264,stream-format=byte-stream", "!", "fdsink", "fd=1", "sync=false",
        ]
    if kind == "x264":
        return [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgra",
            "-s", f"{width}x{height}", "-r", str(fps), "-i", "-", "-vf", f"scale={out_width}:{out_height}",
            "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p", "-bf", "0",
            "-b:v", str(bitrate), "-maxrate", str(bitrate), "-bufsize", str(bitrate // 2), "-g", str(idr_interval),
            "-x264-params", "aud=1:repeat-headers=1", "-flush_packets", "1", "-f", "h264", "-",
        ]
    raise ValueError(f"encoder {kind!r}: one of {', '.join(ENCODERS)}")


class Encoded(NamedTuple):
    """One frame's access unit with the stamps of the frame it encodes."""
    unit: bytes
    seq: int
    camera_ns: int
    host_ns: int


class H264Encoder:
    """One encoder subprocess of fixed sizes. `encode()` hands a frame over without waiting for the encoder, and
    drops it (returns False) while the oldest frame in flight was handed over more than `max_delay` seconds ago: a
    time, not a count of frames, since NVENC holds 59-75 ms of frames at any rate (two at 30 fps, four at 60). Each
    access unit goes to `on_unit` on the encoder's reader thread, with the stamps of its frame. `failure` is set once
    the encoder stops."""

    def __init__(self, kind: str, size: Tuple[int, int], out_size: Tuple[int, int], fps: int, bitrate: int,
                 idr_interval: int, on_unit: Callable[[Encoded], None], max_delay: float) -> None:
        self.size = size
        self.failure: Optional[str] = None
        self._on_unit = on_unit
        self.max_delay = max_delay
        self._stamps: Deque[Tuple[int, int, int, float]] = deque()
        """(seq, camera_ns, host_ns, when it was handed over) of each frame in flight."""
        self._lock = threading.Lock()
        command = encoder_command(kind, size, out_size, fps, bitrate, idr_interval)
        try:
            self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                             stderr=subprocess.PIPE, bufsize=0, close_fds=True,
                                             start_new_session=True)
        except FileNotFoundError:
            raise RuntimeError(f"the {kind} encoder needs `{command[0]}`, which is not installed") from None
        self._stdin = self._process.stdin.fileno()
        _widen_pipe(self._stdin)
        os.set_blocking(self._stdin, False)  # a frame's write waits at most STALL: an encoder that stopped reading
                                             # would otherwise block its caller for good, silently
        self._stderr: Deque[str] = deque(maxlen=20)
        """The encoder's last lines of stderr. Read all the time: a pipe nobody reads fills at 64 KB, and an encoder
        writing warnings into a full pipe stops encoding, silently."""
        self._reader = threading.Thread(target=self._read, name="h264_reader", daemon=True)
        self._reader.start()
        self._stderr_reader = threading.Thread(target=self._read_stderr, name="h264_stderr", daemon=True)
        self._stderr_reader.start()

    def encode(self, image: np.ndarray, seq: int, camera_ns: int, host_ns: int) -> bool:
        """Hands one frame (BGRA, `size`) to the encoder; False if it was dropped (the encoder is behind). Raises once
        the encoder has stopped."""
        if self.failure is not None:
            raise RuntimeError(self.failure)
        if (image.shape[1], image.shape[0], image.shape[2]) != (*self.size, 4):
            raise ValueError(f"frame {image.shape[1]}x{image.shape[0]}x{image.shape[2]}, encoder "
                             f"{self.size[0]}x{self.size[1]}x4 (BGRA)")
        now = time.monotonic()
        with self._lock:
            if self._stamps and now - self._stamps[0][3] > STALL:
                self._stalled("no output")
            if self._stamps and now - self._stamps[0][3] > self.max_delay:
                return False
            self._stamps.append((seq, camera_ns, host_ns, now))
        data = memoryview(np.ascontiguousarray(image)).cast("B")
        deadline = now + STALL
        try:
            while data:
                if not select.select([], [self._stdin], [], max(0.0, deadline - time.monotonic()))[1]:
                    self._stalled("not reading its input")
                try:
                    data = data[os.write(self._stdin, data):]
                except BlockingIOError:  # the pipe filled between select and write
                    pass
        except (BrokenPipeError, OSError) as error:
            self.failure = self.failure or f"the H.264 encoder stopped: {error}{self._stderr_tail()}"
            raise RuntimeError(self.failure) from error
        return True

    def _stalled(self, how: str) -> None:
        self.failure = f"the H.264 encoder stalled: {how} for {STALL} s{self._stderr_tail(ended=False)}"
        raise RuntimeError(self.failure)

    def _read(self) -> None:
        stream = bytearray()
        while True:
            chunk = self._process.stdout.read(65536)
            if not chunk:
                break
            stream += chunk
            for unit in split_access_units(stream):
                self._hand_over(unit)
        if stream.startswith(AUD):  # the stream ended: its last unit has no delimiter after it
            self._hand_over(bytes(stream))
        code = self._process.wait()
        if self.failure is None and code != 0:
            self.failure = f"the H.264 encoder exited with {code}{self._stderr_tail()}"

    def _hand_over(self, unit: bytes) -> None:
        with self._lock:
            seq, camera_ns, host_ns, _ = self._stamps.popleft() if self._stamps else (0, 0, 0, 0.0)
        self._on_unit(Encoded(unit, seq, camera_ns, host_ns))

    def _read_stderr(self) -> None:
        for line in self._process.stderr:
            self._stderr.append(line.decode(errors="replace").rstrip())

    def _stderr_tail(self, ended: bool = True) -> str:
        """": <the encoder's last line of stderr>", or "". Once it has `ended`, its last words are waited for (1 s)."""
        if ended:
            self._stderr_reader.join(timeout=1.0)
        return f": {self._stderr[-1]}" if self._stderr else ""

    def close(self) -> None:
        """Ends the stream (the encoder flushes its last units to `on_unit`) and the subprocess: 2 s, then killed."""
        try:
            self._process.stdin.close()
        except OSError:
            pass
        try:
            self._process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=5.0)
        self._reader.join(timeout=2.0)
        self._stderr_reader.join(timeout=2.0)


F_SETPIPE_SZ = 1031
"""Linux's fcntl for a pipe's capacity (Python's fcntl module has it only from 3.10 on some builds)."""


def _widen_pipe(fd: int) -> None:
    """A frame into the encoder in a few writes, not a hundred: Linux pipes hold 64 KB, a 1920x1200 frame is 6.9 MB,
    and each refill is a wake-up of both processes. Up to the system's `pipe-max-size` (1 MB by default)."""
    try:
        with open("/proc/sys/fs/pipe-max-size") as limit:
            fcntl.fcntl(fd, F_SETPIPE_SZ, int(limit.read()))
    except OSError:  # not Linux, or not allowed: the default pipe still works, only slower
        pass


# ---- RTP ----------------------------------------------------------------------------------------------------------
_RTP_HEADER = struct.Struct("!BBHII")


def rtp_packets(unit: bytes, seq: int, timestamp: int, ssrc: int, mtu: int = MTU_PAYLOAD) -> List[bytes]:
    """One access unit as RTP packets (RFC 6184): a NAL that fits is one packet, a bigger one is split into FU-A
    fragments; the marker bit ends the unit. `seq` is the first packet's sequence number; the caller adds
    `len(result)` for the next unit. Access unit delimiters are left out (RTP marks units itself)."""
    nals = [nal for nal in nal_units(unit) if nal[0] & 0x1F != 9]
    payloads: List[bytes] = []
    for nal in nals:
        if len(nal) <= mtu:
            payloads.append(nal)
            continue
        indicator = (nal[0] & 0xE0) | 28  # FU-A, the NAL's NRI
        nal_type = nal[0] & 0x1F
        body = nal[1:]
        for at in range(0, len(body), mtu - 2):
            start, end = at == 0, at + mtu - 2 >= len(body)
            header = (0x80 if start else 0) | (0x40 if end else 0) | nal_type
            payloads.append(bytes((indicator, header)) + body[at:at + mtu - 2])
    packets = []
    for i, payload in enumerate(payloads):
        marker = 0x80 if i == len(payloads) - 1 else 0
        packets.append(_RTP_HEADER.pack(0x80, marker | RTP_PAYLOAD_TYPE, (seq + i) & 0xFFFF,
                                        timestamp & 0xFFFFFFFF, ssrc) + payload)
    return packets


def receiver_command(port: int) -> List[str]:
    """The GStreamer window that plays one camera's stream arriving on UDP `port` of the computer it runs on (needs
    GStreamer with gst-libav: `avdec_h264`). The jitter buffer only puts packets back in order (latency 0): a frame
    shows as soon as it is whole."""
    return ["gst-launch-1.0", "-q", "udpsrc", f"port={port}",
            f"caps=application/x-rtp,media=video,clock-rate={RTP_CLOCK},encoding-name=H264,payload={RTP_PAYLOAD_TYPE}",
            "!", "rtpjitterbuffer", "latency=0", "drop-on-latency=true", "!", "rtph264depay", "!", "avdec_h264", "!",
            "videoconvert", "!", "autovideosink", "sync=false"]


def is_keyframe(unit: bytes) -> bool:
    """Whether an Annex B access unit holds an IDR picture: a decoder, or a file, can start at it."""
    return any(nal and nal[0] & 0x1F == 5 for nal in nal_units(unit))


def shares_encoder(size: Tuple[int, int], camera_fps: int, stream_fps: int, stream_height: int) -> bool:
    """Whether a camera of `size` at `camera_fps` is recorded and streamed from one encoder: its stream is the
    camera's own picture (`stream_height` doesn't scale it down, `stream_fps` doesn't thin it), which is what the
    recording encodes anyway. Otherwise each has its own."""
    return scaled_size(size, stream_height) == size and min(stream_fps, camera_fps) == camera_fps


def scaled_size(size: Tuple[int, int], height: int) -> Tuple[int, int]:
    """(width, height) of a frame of `size` scaled to `height` (never up), both even, as H.264 needs."""
    width, full = size
    height = min(height, full)
    return max(2, round(width * height / full) // 2 * 2), max(2, height // 2 * 2)
