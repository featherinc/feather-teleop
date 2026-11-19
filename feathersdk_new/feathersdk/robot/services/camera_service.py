"""The robot's cameras: the stereo ZED (2i or X) and any ZED X One (GMSL, one eye) cameras, such as wrist cameras.

They are captured in a process of their own (`camera_process`), so the ZED SDK's grabs never share the GIL with the
robot's control loops, and read here as stamped, read-only frames, newest wins: `latest()` is the newest frame now,
`wait()` the next one, `age()` how old the newest is. Every camera of the config is there from the start, open or not:
one that doesn't open, or stops delivering, is retried by the process and named in `faults` (logged once) until it
delivers; frames keep coming from the others. The stereo camera's IMU is read at its own rate:
`get_imu_data()` and `get_imu_angular_velocity()`."""
import glob
import multiprocessing
import os
import threading
import time

import numpy as np

from feathersdk.configs import PyDBaseModel, PyDField, pyd_model_validator
from feathersdk.logging import error, info
from feathersdk.robot.robot_module import RobotModule, RobotModuleConfig
from feathersdk.robot.services import camera_process
from feathersdk.robot.services.camera_encoding import shares_encoder
from feathersdk.robot.services.camera_frames import STEREO, Calibration, Frame, FrameSlot, Imu, ImuSlot, imu_path
from feathersdk.utils import currtime_ns
from feathersdk.utils.feathertypes import TYPE_CHECKING, Any, Dict, List, Literal, Optional, Tuple, UniqueEnum

IMU_MAX_AGE = 0.1
"""Seconds an IMU reading may be old before the getters raise: the IMU gives hundreds a second, so an older newest one
means the camera stopped."""

if TYPE_CHECKING:
    from feathersdk.robot.robot import FeatherRobot


class CameraResolution(UniqueEnum):
    """Resolution of one ZED eye.

    ZED 2i: HD720 (1280×720), HD1080 (1920×1080), HD2K (2208×1242).
    ZED X and ZED X One: HD1200 (1920×1200), HD1080 (1920×1080), SVGA (960×600).
    """
    HD720 = 0
    HD1080 = 1
    HD2K = 2
    HD1200 = 3
    SVGA = 4


class CameraColorSpace(UniqueEnum):
    """The color space of the camera image output."""
    BGR = 0
    RGB = 1


class ZedCameraVersion(UniqueEnum):
    """The version of the camera."""
    ZED_2i = 0
    ZED_X = 1


# One-eye size in pixels. A resolution absent from a camera's map cannot be opened.
_CAMERA_RESOLUTIONS: Dict[ZedCameraVersion, Dict[CameraResolution, Tuple[int, int]]] = {
    ZedCameraVersion.ZED_2i: {
        CameraResolution.HD720: (1280, 720),
        CameraResolution.HD1080: (1920, 1080),
        CameraResolution.HD2K: (2208, 1242),
    },
    ZedCameraVersion.ZED_X: {
        CameraResolution.HD1200: (1920, 1200),
        CameraResolution.HD1080: (1920, 1080),  # the D-sample's ZED X ran at HD1080 / 30 fps
        CameraResolution.SVGA: (960, 600),
    },
}


class BinocularCameraView(UniqueEnum):
    """The view of the binocular camera to use."""
    LEFT = 0
    RIGHT = 1
    BOTH = 2


class ZedOneCameraConfig(PyDBaseModel):
    """A ZED X One (one eye, GMSL), opened beside the stereo camera: a wrist camera."""
    serial: int = PyDField(gt=0)
    """The camera's serial number (`ZED_Explorer` lists them)."""
    resolution: CameraResolution = PyDField(default=CameraResolution.HD1200)
    """A mode of the ZED X."""
    fps: int = PyDField(gt=0, default=30)

    @pyd_model_validator(mode="after")
    def validate_resolution(self) -> "ZedOneCameraConfig":
        modes = _CAMERA_RESOLUTIONS[ZedCameraVersion.ZED_X]
        if self.resolution not in modes:
            raise ValueError(f"A ZED X One does not support {self.resolution.name}. Supported resolutions: "
                             f"{', '.join(item.name for item in modes)}.")
        return self

    def eye_size(self) -> Tuple[int, int]:
        """(width, height) of a frame in pixels."""
        return _CAMERA_RESOLUTIONS[ZedCameraVersion.ZED_X][self.resolution]


class WebViewConfig(PyDBaseModel):
    """Every camera as MJPEG in a browser at `http://<the robot>:<port>/`."""
    port: int = PyDField(default=5000, gt=0, lt=65536)
    fps: float = PyDField(default=30.0, gt=0.0)
    """Frames a second per open page, at most."""
    height: int = PyDField(default=360, gt=0)
    """Pixels each camera is scaled to."""
    quality: int = PyDField(default=70, ge=1, le=100)
    """JPEG quality."""


class ZedCameraServiceConfig(RobotModuleConfig):
    """Configuration for the camera service: the stereo camera, and any ZED X One cameras beside it."""
    module_name: str = PyDField(default="camera_service")
    """The name of the module."""
    version: ZedCameraVersion = PyDField(default=ZedCameraVersion.ZED_2i)
    """The version of the camera."""
    resolution: CameraResolution = PyDField(default=CameraResolution.HD720)
    """The resolution of the camera. Must be a mode of `version`."""
    fps: int = PyDField(gt=0, default=30)
    """The frame rate of the camera."""
    view: BinocularCameraView = PyDField(default=BinocularCameraView.BOTH)
    """Whether to use just the left/right, or both cameras. The stereo frames are the selected view."""
    color_space: CameraColorSpace = PyDField(default=CameraColorSpace.BGR)
    """The color space of every camera's frames."""
    brightness: int = PyDField(default=5)
    """The brightness of the camera."""
    whitebalance_temp: Optional[int] = PyDField(default=None)
    """The stereo camera's white balance temperature in kelvin, fixed. None (the default): automatic white balance. A
    fixed 5600 (the old default) gave a yellow picture under warm indoor light."""
    serial: Optional[int] = PyDField(default=None)
    """The stereo camera's serial number, when more than one ZED is plugged in; None opens the first found."""
    mono_cameras: Dict[str, ZedOneCameraConfig] = PyDField(default_factory=dict)
    """ZED X One cameras by name (`left_wrist`), opened beside the stereo one. `STEREO` is not a name for them."""
    pattern: bool = PyDField(default=False)
    """A moving test pattern of each camera's size and rate instead of the cameras: tests and dry runs without a ZED."""
    start_timeout: float = PyDField(gt=0.0, default=60.0)
    """Seconds allowed for every camera's first attempt to open (the ZED SDK takes several seconds each); a camera
    that doesn't open is no failure, it is retried and named in `faults`."""
    encoder: Literal["nvenc", "x264"] = PyDField(default="nvenc")
    """H.264 for recording and streaming: `nvenc`, the Jetson's hardware encoder (GStreamer's `nvv4l2h264enc`);
    `x264`, libx264 through `ffmpeg` on a machine without one."""
    recording_bitrate: int = PyDField(default=4_000_000, gt=0)
    """bit/s per camera while recording."""
    recording_idr_interval: int = PyDField(default=30, gt=0)
    """Frames between a recording's keyframes."""
    stream_port: int = PyDField(default=5004, gt=0, lt=65536)
    """UDP port of the first camera's stream on the receiving computer; camera `i` (in `names` order) uses + i."""
    stream_fps: int = PyDField(default=30, gt=0)
    """Frames a second each camera streams at (never above its own rate)."""
    stream_height: int = PyDField(default=720, gt=0)
    """Pixels each camera's stream is scaled to (never up; the width keeps the aspect ratio). A camera no taller than
    this, streamed at its own rate, is encoded once for its recording and its stream: both then use the higher of
    `recording_bitrate` and `stream_bitrate` and the shorter of the two keyframe intervals."""
    stream_bitrate: int = PyDField(default=4_000_000, gt=0)
    """bit/s per camera while streaming."""
    stream_idr_interval: int = PyDField(default=30, gt=0)
    """Frames between a stream's keyframes: how long a receiver that joins, or loses a packet, waits for a whole
    picture."""
    web: Optional[WebViewConfig] = PyDField(default=None)
    """Every camera in a browser while the service runs; None for none."""
    encode_budget: float = PyDField(default=400.0, gt=0.0)
    """Mpixel/s the encoders may take in all together: each recording and stream pipeline takes every frame it
    encodes at the camera's full size (a stream scales on the VIC). The figures below were measured while a CPU
    `videoconvert` still turned each frame into BGRx; frames now reach the VIC as BGRx, so the real limit is likely
    higher: measure before raising it. Past it the Jetson doesn't slow down evenly, it starves whole encoders (AGX
    Orin: 387 recording and streaming three cameras at 30 fps ran, 497 for
    two cameras at 60 fps starved two of four encoders every time). `start_recording()` and `start_stream()` refuse a
    load above it."""

    @pyd_model_validator(mode="after")
    def validate_resolution_for_version(self) -> "ZedCameraServiceConfig":
        modes = _CAMERA_RESOLUTIONS[self.version]
        if self.resolution not in modes:
            allowed = ", ".join(item.name for item in modes)
            raise ValueError(
                f"{self.version.name} does not support {self.resolution.name}. Supported resolutions: {allowed}."
            )
        if STEREO in self.mono_cameras:
            raise ValueError(f"{STEREO!r} is the stereo camera's name; name the ZED X One cameras otherwise.")
        return self

    def eye_size(self) -> Tuple[int, int]:
        """(width, height) of one eye of the stereo camera in pixels."""
        return _CAMERA_RESOLUTIONS[self.version][self.resolution]


class ZedCameraService(RobotModule[ZedCameraServiceConfig]):
    """The cameras, read as frames (`camera_frames.Frame`): `latest(name)`, `wait(name, after, timeout)` and
    `age(name)`, with `name` `STEREO` or a key of the config's `mono_cameras`. `names`, `shape()`, `fps()` and
    `calibration()` describe them; `faults` says which camera (or which camera's recording or stream) isn't delivering
    and why; `get_imu_data()` and `get_imu_angular_velocity()` are the stereo camera's newest IMU reading.
    `start_recording()` records every camera as H.264; `start_stream()` streams the cameras asked for live to one
    computer (`show_stream()` changes which);
    the config's `web` shows them in a browser.

    The capture process is spawned, which imports the program's main module again: a script that creates the robot
    (or this service) does so under `if __name__ == "__main__":`, as any program using `multiprocessing` does."""

    def __init__(self, config: ZedCameraServiceConfig, robot: 'FeatherRobot') -> None:
        super().__init__(config, robot)
        self._faults: Dict[str, str] = {}
        self._recording = False
        self._streaming: List[str] = []  # the cameras streaming
        self._calibrations: Dict[str, Calibration] = {}  # by name, once the camera has opened
        self._imu: Optional[ImuSlot] = None
        self._ended: Optional[str] = None
        self._slots: Dict[str, FrameSlot] = {}
        self._requests = 0  # the number of the last request sent to the process
        self._receiving = threading.Lock()  # one reader of the pipe at a time (two would split its messages)
        self._sending = threading.Lock()  # and one writer; a write never waits for the reader (nor the reverse), so
                                          # a full pipe in one direction can't stop the other draining

        self._prefix = f"/dev/shm/feather-camera-{os.getpid()}-"  # the frames' files, one per camera
        self._remove_files()  # left by an earlier process with this pid that never cleaned up
        context = multiprocessing.get_context("spawn")  # a fresh interpreter: none of this process's threads or locks
        self._lock = context.Lock()
        self._conn, child = context.Pipe()
        self._process = context.Process(target=camera_process.run, name=config.module_name, daemon=True,
                                        args=(child, self._lock, config, self._prefix))
        self._process.start()
        child.close()
        self._infos: Dict[str, camera_process.CameraInfo] = self._wait_ready(config.start_timeout)
        rgb = config.color_space == CameraColorSpace.RGB
        self._slots = {name: FrameSlot(i.path, i.shape, self._lock, rgb=rgb) for name, i in self._infos.items()}
        self._imu = ImuSlot(imu_path(self._prefix), self._lock)
        info(f"[{config.module_name}] cameras: " + ", ".join(
            f"{name} {i.shape[1]}x{i.shape[0]} at {i.fps} fps" for name, i in self._infos.items()))

    # ---- The cameras -------------------------------------------------------------------------------------------------
    @property
    def names(self) -> List[str]:
        """Every camera's name: `STEREO` first, then the mono cameras in config order."""
        return list(self._infos)

    def shape(self, name: str = STEREO) -> Tuple[int, int, int]:
        """(height, width, 3) of the camera's frames."""
        return self._info(name).shape

    def fps(self, name: str = STEREO) -> int:
        return self._info(name).fps

    def calibration(self, name: str = STEREO) -> Calibration:
        """The rectified pinhole model of the camera's left (or only) eye; raises while the camera hasn't opened."""
        self._info(name)
        self._drain()
        if name not in self._calibrations:
            raise RuntimeError(f"Camera {name!r} has not opened: {self._faults.get(name)}")
        return self._calibrations[name]

    @property
    def faults(self) -> Dict[str, str]:
        """Why each camera that isn't delivering isn't (not open yet, or being reopened), and each recording or stream
        that failed, by name; empty while all is well. Once the process has ended, every camera, with why: a poller
        learns of it here, as the readers do by raising."""
        self._drain()
        if self._ended is not None:
            return {name: self._ended for name in self._infos}
        return dict(self._faults)

    # ---- The frames --------------------------------------------------------------------------------------------------
    def latest(self, name: str = STEREO, out: Optional[np.ndarray] = None) -> Optional[Frame]:
        """The newest frame of the camera, or None before its first: a new read-only copy, or copied into `out` (an
        array of `shape(name)`, then the frame's image) for a consumer that reuses its buffer, which saves the
        allocation of a 5-7 MB array per frame. Raises if the capture process has ended."""
        self._check()
        return self._slot(name).read(out)

    def wait(self, name: str = STEREO, after: int = 0, timeout: float = 1.0,
             out: Optional[np.ndarray] = None) -> Optional[Frame]:
        """The newest frame once its `seq` is above `after` (the last frame's `seq` for the next one), or None after
        `timeout` seconds without one; `out` as in `latest()`. Raises if the capture process has ended."""
        self._check()
        return self._slot(name).wait(after, timeout, out)

    def age(self, name: str = STEREO) -> float:
        """Seconds since the camera's newest frame arrived; infinite before the first."""
        self._check()
        seq, _, host_ns = self._slot(name).header()
        return float("inf") if seq == 0 else (currtime_ns() - host_ns) / 1e9

    def get_imu_data(self) -> Tuple[float, float, float]:
        """The stereo camera's latest linear acceleration, m/s^2, gravity included. IMU frame: +X right of the camera,
        +Y down, +Z forward along the optical axis. Raises without a reading in the last `IMU_MAX_AGE` s."""
        return self._imu_reading().accel

    def get_imu_angular_velocity(self) -> Tuple[float, float, float]:
        """The stereo camera's latest angular velocity, rad/s, in the same frame. Raises without a reading in the last
        `IMU_MAX_AGE` s."""
        return self._imu_reading().gyro

    def reset_camera(self, name: str = STEREO) -> None:
        """Asks the process to close and reopen the camera (several seconds; its frames stop meanwhile)."""
        self._info(name)
        self._request("reopen", name)

    # ---- Recording and streaming -------------------------------------------------------------------------------------
    def start_recording(self, directory: str) -> None:
        """Records every camera into `directory` (which must exist) until `stop_recording()`: `<name>.h264` at full
        size and rate, and `<name>.jsonl`, a line per frame in the file (index, `seq`, camera and host times).
        Raises ValueError if the encoders would take in more than `encode_budget`."""
        self._check_budget(recording=True, streaming=self._streaming)
        self._request("record", os.fspath(directory))
        self._recording = True

    def stop_recording(self) -> Dict[str, Dict[str, Any]]:
        """Ends the recording; per camera, `frames` written, `missed` (the camera's frames between the start and the
        stop that aren't in the file: the recorder or its encoder couldn't keep up, or stopped) and `failure` (why
        that camera's recording stopped early, or None). Raises if not recording."""
        summary = self._request("stop_recording", None, timeout=15.0)
        self._recording = False
        self._forget_faults("recording:")
        return summary

    def start_stream(self, host: str, names: Optional[List[str]] = None) -> Dict[str, int]:
        """Streams the cameras `names` (every camera by default) live to `host` as RTP/H.264 at `stream_fps`,
        `stream_height` and `stream_bitrate` until `stop_stream()`; returns the UDP port each goes to (camera `i` in
        the service's order always on `stream_port + i`, whichever others stream). On that computer,
        `camera_encoding.receiver_command(port)` is the GStreamer window that plays one. `show_stream()` changes
        which cameras stream. Raises ValueError if the encoders would take in more than
        `encode_budget`."""
        names = self.names if names is None else self._known(names)
        self._check_budget(recording=self._recording, streaming=names)
        ports = self._request("stream", (host, names))
        self._streaming = names
        return ports

    def show_stream(self, names: List[str]) -> Dict[str, int]:
        """While streaming, streams exactly `names` from now on: an encoder starts for each camera added and ends for
        each one left out, so only the cameras being watched cost anything. Returns their ports."""
        names = self._known(names)
        self._check_budget(recording=self._recording, streaming=names)
        ports = self._request("show_stream", names)
        self._streaming = names
        self._forget_faults("stream:")
        return ports

    def stop_stream(self) -> Dict[str, int]:
        """Ends the stream; the frames each camera that streamed sent. Raises if not streaming."""
        sent = self._request("stop_stream", None)
        self._streaming = []
        self._forget_faults("stream:")
        return sent

    # ---- Inside -----------------------------------------------------------------------------------------------------
    def _check_budget(self, recording: bool, streaming: List[str]) -> None:
        """Raises ValueError if recording (every camera) and streaming the cameras `streaming` would have the
        encoders take in more than `encode_budget`, naming each camera's share. Each pipeline takes a frame at the
        camera's full size; a camera whose stream is its own picture has one pipeline for both
        (`camera_encoding.shares_encoder`)."""
        shares = {}
        for name in self.names:
            height, width, _ = self.shape(name)
            fps = self.fps(name)
            if shares_encoder((width, height), fps, self.config.stream_fps, self.config.stream_height):
                frames = fps if recording or name in streaming else 0
            else:
                frames = (fps if recording else 0) + (min(self.config.stream_fps, fps) if name in streaming else 0)
            shares[name] = width * height * frames / 1e6
        total = sum(shares.values())
        if total > self.config.encode_budget:
            raise ValueError(
                f"[{self.config.module_name}] recording {'on' if recording else 'off'} and streaming "
                f"{', '.join(streaming) if streaming else 'off'} would have the encoders take in {total:.0f} Mpixel/s, "
                f"more than "
                f"encode_budget {self.config.encode_budget:.0f}: "
                + ", ".join(f"{name} {share:.0f}" for name, share in shares.items())
                + ". Lower a camera's fps or size, or stream_fps.")

    def _known(self, names: List[str]) -> List[str]:
        """`names`, each a camera, once each, in the service's order; raises ValueError naming one that isn't."""
        for name in names:
            self._info(name)
        return [name for name in self.names if name in names]

    def _info(self, name: str) -> camera_process.CameraInfo:
        if name not in self._infos:
            raise ValueError(f"No camera {name!r}: the cameras are {', '.join(self._infos)}")
        return self._infos[name]

    def _slot(self, name: str) -> FrameSlot:
        self._info(name)
        return self._slots[name]

    def _imu_reading(self) -> Imu:
        """The newest IMU reading (the capture process takes each one, hundreds a second); raises if there is none
        or it is older than `IMU_MAX_AGE`, saying why the stereo camera isn't delivering when it isn't."""
        self._check()
        reading = self._imu.read()
        age = None if reading is None else (currtime_ns() - reading.host_ns) / 1e9
        if age is None or age > IMU_MAX_AGE:
            fault = self._faults.get(STEREO)
            raise RuntimeError(("No IMU reading from the stereo camera" if age is None else
                                f"The stereo camera's newest IMU reading is {age:.2f} s old")
                               + (f": {fault}" if fault else ""))
        return reading

    def _check(self) -> None:
        """Takes the process's reports; raises once it has ended, since no frame will come again."""
        self._drain()
        if self._ended is not None:
            raise RuntimeError(self._ended)

    def _drain(self) -> None:
        """What the process reported since the last call. A thread that finds another one draining goes on without
        waiting (a wait() loop per camera calls this every read)."""
        if not self._receiving.acquire(blocking=False):
            return
        try:
            while self._ended is None and self._conn.poll():
                self._take(*self._conn.recv())
        except (EOFError, OSError):
            self._ended = f"[{self.config.module_name}] the camera process ended (exit code {self._process.exitcode})"
            error(self._ended)
        finally:
            self._receiving.release()

    def _request(self, command: str, argument: Any, timeout: float = 10.0) -> Any:
        """Sends a request to the process and returns its result; its error is raised here. Each request has a number
        and its answer carries it: the late answer of a request that timed out is dropped, never taken for the next
        one's."""
        self._check()
        with self._receiving:  # the answer is read here, not by a thread draining reports meanwhile
            self._requests += 1
            number = self._requests
            with self._sending:
                self._conn.send((command, argument, number))
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not self._conn.poll(remaining):
                    raise TimeoutError(f"[{self.config.module_name}] no answer to {command!r} within {timeout} s")
                kind, name, payload = self._conn.recv()
                if kind in ("done", "error") and name != number:
                    continue  # an earlier request's, which timed out
                if kind == "done":
                    return payload
                if kind == "error":
                    raise RuntimeError(f"[{self.config.module_name}] {command}: {payload}")
                self._take(kind, name, payload)

    def _forget_faults(self, prefix: str) -> None:
        """Drops the faults of an output that has stopped (`recording:` or `stream:` names): they no longer apply.
        Holding the reader's lock: a report taken meanwhile changed the dict mid-iteration (a RuntimeError), or landed
        in the replaced one and was lost. The output's own reports came before its stop's answer, already taken."""
        with self._receiving:
            self._faults = {name: why for name, why in self._faults.items() if not name.startswith(prefix)}

    def _take(self, kind: str, name: str, payload: Any) -> None:
        """One report of the process: a fault (logged when it changes), a camera opened (its calibration), a camera
        delivering again."""
        module = self.config.module_name
        if kind == "fault":
            if self._faults.get(name) != payload:
                error(f"[{module}] {name}: {payload}")
            self._faults[name] = payload
        elif kind == "opened":
            self._calibrations[name] = payload
        elif kind == "recovered":
            self._faults.pop(name, None)
            info(f"[{module}] {name} delivers")

    def _wait_ready(self, timeout: float) -> Any:
        """The payload of the process's `ready`, taking each camera's first report (opened, or why not) on the way.
        An `error` instead, a process that ends first, or no `ready` within `timeout`, ends the process, removes its
        files and raises."""
        module = self.config.module_name
        deadline = time.monotonic() + timeout
        try:
            while True:
                if not self._conn.poll(max(0.0, deadline - time.monotonic())):
                    raise TimeoutError(f"[{module}] the camera process wasn't ready within {timeout} s")
                kind, name, payload = self._conn.recv()
                if kind == "ready":
                    return payload
                if kind == "error":
                    raise RuntimeError(f"[{module}] {payload}")
                self._take(kind, name, payload)
        except EOFError:
            self._abort_impl()
            raise RuntimeError(f"[{module}] the camera process ended (exit code {self._process.exitcode}) before it "
                               "was ready") from None
        except Exception:
            self._abort_impl()  # one cleanup for every failure
            raise

    def _abort_impl(self) -> None:
        """Ends the process (asked, then after 10 s terminated), unmaps the frames and removes their files."""
        if self._process.is_alive():
            try:
                with self._sending:
                    self._conn.send(("close", None, 0))
            except OSError:
                pass
            self._process.join(timeout=10.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=5.0)
        self._conn.close()
        for slot in self._slots.values():
            slot.close()
        self._slots = {}
        if self._imu is not None:
            self._imu.close()
            self._imu = None
        self._remove_files()

    def _remove_files(self) -> None:
        """Removes the frames' files, every one with this process's prefix (a start that timed out may have left some
        the process never reported)."""
        for path in glob.glob(f"{self._prefix}*"):
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass
