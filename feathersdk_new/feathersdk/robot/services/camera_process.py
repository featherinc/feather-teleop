"""The camera service's capture process: keeps every camera of the config open and grabbing, each on its own thread
into its shared-memory slot (`camera_frames.FrameSlot`). A camera that doesn't open, or stops delivering (grab errors,
or blank frames from a broken pipeline), is reported and retried with a growing wait until it delivers again; the
others never wait for it. Answers the service over a pipe. Spawned as its own interpreter, so the ZED SDK's grabs and
the colour conversions never share the GIL with the robot's control loops."""
import signal
import threading
import time

import numpy as np

from feathersdk.robot.services.camera_outputs import Recording, SharedEncoders, Stream, WebView
from feathersdk.robot.services.camera_frames import (
    STEREO, Calibration, FrameSlot, ImuSlot, imu_path, is_blank, pattern_image, sample_points,
)
from feathersdk.utils import currtime_ns
from feathersdk.utils.feathertypes import Any, Dict, NamedTuple, Optional, Tuple

FAILURES_BEFORE_REOPEN = 5
"""Consecutive bad grabs (an error or a blank frame) before the camera is closed and reopened."""
REOPEN_WAIT = (1.0, 30.0)
"""Seconds between reopen attempts: the first, doubling to the second."""
IMU_PERIOD = 0.0025
"""Seconds between looks at the stereo camera's IMU: its rate (the ZED 2i's is 400 Hz), so each reading is taken."""


class CameraInfo(NamedTuple):
    """What the service knows of a camera from the start, open or not: all of it from the config."""
    shape: Tuple[int, int, int]
    """(height, width, 3) of its frames."""
    fps: int
    path: str
    """The shared-memory file of its frames."""


def _pinhole(information: Any, width: int, height: int, baseline_m: float) -> Calibration:
    """The left (or only) eye's rectified model from a ZED's camera information: a stereo camera's parameters hold a
    `left_cam`, a ZED X One's are the eye's own."""
    parameters = information.camera_configuration.calibration_parameters
    eye = getattr(parameters, "left_cam", parameters)
    return Calibration(float(eye.fx), float(eye.fy), float(eye.cx), float(eye.cy), width, height, baseline_m)


class _Zed:
    """An open ZED, stereo (`sl.Camera`, with an IMU) or a ZED X One (`sl.CameraOne`): each `grab()` retrieves `view`
    as the ZED delivers it, BGRA (the frame slot keeps it so: no CPU colour conversion); `read_imu()` is the IMU's
    newest reading, from another thread (the ZED SDK reads its sensors beside a running grab)."""

    def __init__(self, camera: Any, view: Any, stereo: bool) -> None:
        import pyzed.sl as sl
        self._sl, self._camera, self._view = sl, camera, view
        self._mat = sl.Mat()
        # Stereo only: a CameraOne's grab() takes no runtime parameters and it has no IMU.
        self._grab_args = (sl.RuntimeParameters(),) if stereo else ()
        self._sensors = sl.SensorsData() if stereo else None
        self._closing = threading.Lock()  # read_imu() on the IMU thread never runs into close() on the capture's
        self._closed = False
        information = camera.get_camera_information()
        size = information.camera_configuration.resolution
        width, height = int(size.width), int(size.height)
        both = view == sl.VIEW.SIDE_BY_SIDE
        self.shape = (height, 2 * width if both else width, 3)
        parameters = information.camera_configuration.calibration_parameters
        baseline_m = float(parameters.get_camera_baseline()) if stereo else 0.0  # metres: _open_stereo sets the unit
        self.calibration = _pinhole(information, width, height, baseline_m)

    def grab(self) -> Tuple[np.ndarray, int]:
        """(BGRA image, the camera's capture time in ns). The image is the ZED's own buffer, valid until the next
        grab: the capture copies it into the frame slot first."""
        sl = self._sl
        result = self._camera.grab(*self._grab_args)
        if result != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"grab failed: {result}")
        result = self._camera.retrieve_image(self._mat, self._view)
        if result != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"retrieve_image failed: {result}")
        stamp = int(self._camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
        return self._mat.numpy(), stamp

    def read_imu(self) -> Optional[Tuple[int, Tuple[float, float, float], Tuple[float, float, float]]]:
        """(the IMU's time in ns, linear acceleration in m/s^2, angular velocity in rad/s), or None: no IMU (a ZED X
        One), closed, or no reading available."""
        sl = self._sl
        with self._closing:
            if self._sensors is None or self._closed or \
                    self._camera.get_sensors_data(self._sensors, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
                return None
            data = self._sensors.get_imu_data()
            if not data.is_available:
                return None
            ax, ay, az = data.get_linear_acceleration()
            wx, wy, wz = data.get_angular_velocity()  # deg/s from the ZED SDK
            stamp = int(data.timestamp.get_nanoseconds())
        return stamp, (float(ax), float(ay), float(az)), \
            (float(np.deg2rad(wx)), float(np.deg2rad(wy)), float(np.deg2rad(wz)))

    def close(self) -> None:
        with self._closing:
            self._closed = True
            self._camera.close()


def _open_stereo(config: Any) -> _Zed:
    """The stereo ZED (2i or X) of the service config: both eyes side by side, or one, as its view says."""
    import pyzed.sl as sl
    model = {"ZED_2i": "ZED2i", "ZED_X": "ZED_X"}[config.version.name]
    devices = list(sl.Camera.get_device_list())
    if devices and all(device.camera_model.name != model for device in devices):
        found = ", ".join(f"{device.camera_model.name} (serial {device.serial_number})" for device in devices)
        raise RuntimeError(f"Config requests {config.version.name}, but the connected camera is {found}.")
    camera = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = getattr(sl.RESOLUTION, config.resolution.name)
    params.camera_fps = config.fps
    params.depth_mode = sl.DEPTH_MODE.NONE
    params.coordinate_units = sl.UNIT.METER  # the baseline in metres (the default, millimetres, needed a guess)
    if config.serial is not None:
        params.set_from_serial_number(config.serial)
    result = camera.open(params)
    if result != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open the stereo ZED: {result}")
    camera.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, config.brightness)
    if config.whitebalance_temp is None:
        camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)
    else:  # a fixed temperature turns the automatic white balance off
        camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, config.whitebalance_temp)
    view = {"LEFT": sl.VIEW.LEFT, "RIGHT": sl.VIEW.RIGHT, "BOTH": sl.VIEW.SIDE_BY_SIDE}[config.view.name]
    return _Zed(camera, view, stereo=True)


def _open_mono(name: str, config: Any) -> _Zed:
    """A ZED X One by its serial."""
    import pyzed.sl as sl
    camera = sl.CameraOne()
    params = sl.InitParametersOne()
    params.set_from_serial_number(config.serial)
    params.camera_resolution = getattr(sl.RESOLUTION, config.resolution.name)
    params.camera_fps = config.fps
    result = camera.open(params)
    if result != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED X One {config.serial} ({name}): {result}")
    serial = camera.get_camera_information().serial_number
    if serial != config.serial:
        camera.close()
        raise RuntimeError(f"ZED X One {config.serial} ({name}): opened serial {serial} instead")
    return _Zed(camera, sl.VIEW.LEFT, stereo=False)


class _Pattern:
    """A moving test pattern of a camera's size, paced at its fps: the service without any ZED. Its calibration is a
    made-up pinhole (focal length the width, the centre in the middle); with `imu`, an IMU at rest, level (gravity
    along +Y, down)."""

    def __init__(self, shape: Tuple[int, int, int], fps: int, imu: bool) -> None:
        self.shape = shape
        self._imu = imu
        self.calibration = Calibration(float(shape[1]), float(shape[1]), shape[1] / 2, shape[0] / 2, shape[1],
                                       shape[0], 0.0)
        self._period = 1.0 / fps
        self._index = 0
        self._due = time.monotonic()

    def grab(self) -> Tuple[np.ndarray, int]:
        self._due = max(self._due + self._period, time.monotonic() - self._period)
        time.sleep(max(0.0, self._due - time.monotonic()))
        self._index += 1
        return pattern_image(self.shape[0], self.shape[1], self._index), currtime_ns()

    def read_imu(self) -> Optional[Tuple[int, Tuple[float, float, float], Tuple[float, float, float]]]:
        return (currtime_ns(), (0.0, 9.81, 0.0), (0.0, 0.0, 0.0)) if self._imu else None

    def close(self) -> None:
        pass


def _serve(command: str, argument: Any, captures: Dict[str, "_Capture"], slots: Dict[str, FrameSlot],
           fps: Dict[str, int], config: Any, outputs: Dict[str, Any], report) -> Any:
    """One request of the service; its result, or an exception the service raises."""
    if command == "reopen":
        captures[argument].reopen()
        return None
    if "shared" not in outputs:  # the encoders a camera's recording and stream have in common
        outputs["shared"] = SharedEncoders(slots, fps, config.encoder, config.stream_fps, config.stream_height,
                                           max(config.recording_bitrate, config.stream_bitrate),
                                           min(config.recording_idr_interval, config.stream_idr_interval))
    if command == "record":
        if "recording" in outputs:
            raise RuntimeError("already recording")
        outputs["recording"] = Recording(slots, fps, argument, config.encoder, config.recording_bitrate,
                                         config.recording_idr_interval, report, outputs["shared"])
        return None
    if command == "stop_recording":
        if "recording" not in outputs:
            raise RuntimeError("not recording")
        return outputs.pop("recording").stop()
    if command == "stream":
        if "stream" in outputs:
            raise RuntimeError("already streaming: stop_stream() first")
        host, names = argument
        outputs["stream"] = Stream(slots, fps, host, config.stream_port, config.encoder, config.stream_fps,
                                   config.stream_height, config.stream_bitrate, config.stream_idr_interval, report,
                                   names, outputs["shared"])
        return {name: outputs["stream"].ports[name] for name in names}
    if command == "show_stream":
        if "stream" not in outputs:
            raise RuntimeError("not streaming: start_stream() first")
        return outputs["stream"].show(argument)
    if command == "stop_stream":
        if "stream" not in outputs:
            raise RuntimeError("not streaming")
        return outputs.pop("stream").stop()
    raise ValueError(f"unknown request {command!r}")


def _fps(name: str, config: Any) -> int:
    return config.fps if name == STEREO else config.mono_cameras[name].fps


def _shape(name: str, config: Any) -> Tuple[int, int, int]:
    """(height, width, 3) of the camera's frames as the config sets its resolution and view: the slot exists before
    the camera opens, and a camera that opens at another size is refused."""
    if name == STEREO:
        width, height = config.eye_size()
        return height, 2 * width if config.view.name == "BOTH" else width, 3
    width, height = config.mono_cameras[name].eye_size()
    return height, width, 3


def _rgb(config: Any) -> bool:
    """Whether the slots' readers get RGB (else BGR); the slots themselves keep the ZED's BGRA either way."""
    return config.color_space.name == "RGB"


def _open(name: str, config: Any) -> Any:
    """The camera `name` of the service config, open: the stereo one, a mono one by name, or a pattern of its size
    when the config says so."""
    if config.pattern:
        return _Pattern(_shape(name, config), _fps(name, config), imu=name == STEREO)
    camera = _open_stereo(config) if name == STEREO else _open_mono(name, config.mono_cameras[name])
    if camera.shape != _shape(name, config):
        camera.close()
        raise RuntimeError(f"opened at {camera.shape[1]}x{camera.shape[0]}, the config says "
                           f"{_shape(name, config)[1]}x{_shape(name, config)[0]}")
    return camera


class _Capture:
    """One camera on its own thread until `stopping` is set: opens it, then grabs every frame into its slot. A camera
    that doesn't open, or fails `FAILURES_BEFORE_REOPEN` grabs in a row, is reported (`("fault", name, why)`) and
    opened again with a growing wait; each open reports `("opened", name, calibration)`, and the first frame after a
    fault `("recovered", name, None)`. `tried` is set once the first attempt to open has finished, either way."""

    def __init__(self, name: str, config: Any, slot: FrameSlot, report, stopping: threading.Event,
                 imu: Optional[ImuSlot] = None) -> None:
        self.camera: Optional[Any] = None
        self.fault: Optional[str] = None
        self.tried = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"capture_{name}", daemon=True)
        # The IMU on a thread of its own, at its rate: the old service read it fresh on each call, and once a frame
        # (30 Hz) is too slow for odometry integrating it at 100 Hz.
        self.imu_thread = threading.Thread(target=self._read_imu, args=(imu,), name=f"imu_{name}", daemon=True) \
            if imu is not None else None
        self._name, self._config, self._slot, self._report, self._stopping = name, config, slot, report, stopping
        self._points = sample_points(slot.shape[1], slot.shape[0])
        self._reopen = threading.Event()

    def reopen(self) -> None:
        """Asks the thread to close and reopen its camera."""
        self._reopen.set()

    def _read_imu(self, imu: ImuSlot) -> None:
        """Every new IMU reading of whichever camera is open into `imu`, until `stopping` is set."""
        last = None
        while not self._stopping.wait(IMU_PERIOD):
            camera = self.camera  # replaced by a reopen: then the old one's read_imu() returns None
            reading = camera.read_imu() if camera is not None else None
            if reading is not None and reading[0] != last:
                last = reading[0]
                imu.write(reading[0], currtime_ns(), reading[1], reading[2])

    def _run(self) -> None:
        self._open_camera("not open")
        failures = 0
        while not self._stopping.is_set():
            if self._reopen.is_set():
                self._reopen.clear()
                self._restart("reopen requested")
                failures = 0
                continue
            try:
                image, camera_ns = self.camera.grab()
                if is_blank(image, self._points):
                    raise RuntimeError("blank frame (a broken pipeline)")
            except Exception as error:
                failures += 1
                if failures >= FAILURES_BEFORE_REOPEN:
                    self._restart(str(error))
                    failures = 0
                continue
            failures = 0
            self._slot.write(image, camera_ns, currtime_ns())
            if self.fault is not None:
                self.fault = None
                self._report(("recovered", self._name, None))

    def _restart(self, why: str) -> None:
        """Reports why the camera stopped, closes it and opens it again."""
        self._set_fault(f"{why}; reopening")
        self.close()
        self._open_camera(why)

    def _open_camera(self, why: str) -> None:
        """Opens the camera, waiting longer after each failure, until it opens or `stopping` is set. Every outcome is
        reported before `tried` is set, so the service has heard of each camera by the time the process is ready."""
        wait = REOPEN_WAIT[0]
        while not self._stopping.is_set():
            try:
                self.camera = _open(self._name, self._config)
            except Exception as error:
                self._set_fault(f"{why}: opening failed: {type(error).__name__}: {error} (next try in {wait:.0f} s)")
                self.tried.set()
                self._stopping.wait(wait)
                wait = min(2 * wait, REOPEN_WAIT[1])
                continue
            self._report(("opened", self._name, self.camera.calibration))
            self.tried.set()
            return

    def _set_fault(self, why: str) -> None:
        self.fault = why
        self._report(("fault", self._name, why))

    def close(self) -> None:
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:  # a camera that broke may fail to close too; it is being dropped either way
                pass
            self.camera = None


def run(conn, lock, config: Any, prefix: str) -> None:
    """The process. Makes every camera's slot (`prefix` + its name) and the stereo camera's IMU file (`imu_path`; the
    service removes the files), and starts each
    camera's capture; once each camera's first attempt to open has finished (its `opened` or `fault` sent), reports
    `("ready", None, {name: CameraInfo})`, or `("error", None, text)` and ends if a slot can't be made. Then serves the
    service's requests (`close`, `reopen`, `record`, `stop_recording`, `stream`, `stop_stream`, each with the request's
    number), answering `("done", number, result)` or `("error", number, text)`, while the captures report."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # the robot's process handles Ctrl-C and tells this one to close
    send_lock = threading.Lock()

    def report(message) -> None:
        with send_lock:
            try:
                conn.send(message)
            except OSError:  # the robot's process is gone: this one is ending too
                pass

    names = [STEREO, *config.mono_cameras]
    slots: Dict[str, FrameSlot] = {}
    imu: Optional[ImuSlot] = None
    try:
        for name in names:
            slots[name] = FrameSlot(f"{prefix}{name}", _shape(name, config), lock, create=True, rgb=_rgb(config))
        imu = ImuSlot(imu_path(prefix), lock, create=True)
    except Exception as error:
        for slot in slots.values():
            slot.close()
        report(("error", None, f"{type(error).__name__}: {error}"))
        return
    stopping = threading.Event()
    captures = {name: _Capture(name, config, slots[name], report, stopping, imu if name == STEREO else None)
                for name in names}
    for capture in captures.values():
        capture.thread.start()
        if capture.imu_thread is not None:
            capture.imu_thread.start()
    for capture in captures.values():
        capture.tried.wait()
    fps = {name: _fps(name, config) for name in names}
    report(("ready", None, {name: CameraInfo(slots[name].shape, fps[name], slots[name].path) for name in names}))
    outputs: Dict[str, Any] = {}
    if config.web is not None:
        outputs["web"] = WebView(slots, config.web.port, config.web.fps, config.web.height, config.web.quality)
    try:
        while True:
            try:
                command, argument, number = conn.recv()
            except EOFError:  # the robot's process is gone
                return
            if command == "close":
                return
            try:
                result = _serve(command, argument, captures, slots, fps, config, outputs, report)
            except Exception as error:
                report(("error", number, f"{type(error).__name__}: {error}"))
            else:
                report(("done", number, result))
    finally:
        for output in outputs.values():
            output.stop()
        stopping.set()
        for capture in captures.values():
            capture.thread.join(timeout=5.0)
            if capture.imu_thread is not None:
                capture.imu_thread.join(timeout=1.0)
            capture.close()
        for slot in slots.values():
            slot.close()
        imu.close()
