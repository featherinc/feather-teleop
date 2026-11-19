"""The camera service's frames, without any camera: the shared-memory slot one capture process writes and the robot's
process reads (one camera's newest frame, two buffers and sequence numbers so neither side waits for the other and no
frame is ever torn), the frame itself, the stereo camera's IMU slot, the blank-frame check and a moving test pattern.
Testable with numpy alone."""
import mmap
import os
import struct
import time

import numpy as np

from feathersdk.utils.feathertypes import NamedTuple, Optional, Tuple

_TOP = struct.Struct("<qq")
"""The file's header: which of the two buffers holds the newest frame (-1 before the first) and its sequence number."""
_HEADER = struct.Struct("<qqq")
"""A buffer's header: the frame's sequence number (0 while it is being written), the camera's capture time (ns) and the
host's (ns)."""
_IMU = struct.Struct("<qqq6d")
"""The IMU file: the reading's number (0 before the first), the IMU's own time (ns), the host's (ns), the linear
acceleration (3, m/s^2) and the angular velocity (3, rad/s)."""
_HEADER_SIZE = 128
"""Bytes each header takes, padded; the image follows its buffer's header."""
_POLL = 0.001
"""Seconds between looks at the slot's sequence number while waiting for a frame."""
STEREO = "stereo"
"""The stereo camera's name in the service (`latest()`, `wait()` and the rest); the mono cameras have the config's."""


class Calibration(NamedTuple):
    """The rectified pinhole model of a camera's left (or only) eye, in pixels of the opened resolution."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    """Pixels across one eye."""
    height: int
    baseline_m: float
    """The stereo baseline in metres; 0 for a camera with one eye."""


class Frame(NamedTuple):
    """One captured frame, as plain values. The image is the caller's, nothing overwrites it: read-only when the
    service allocated it, the caller's own buffer when it passed `out`."""
    image: np.ndarray
    """(height, width, 3) uint8 in the config's colour space; both eyes side by side when the view is BOTH."""
    seq: int
    """1 for the first frame since the service started, then 2, 3, ... (every frame captured, read or not)."""
    camera_ns: int
    """The camera's own capture time, in nanoseconds of its clock."""
    host_ns: int
    """`currtime_ns()` (the SDK's monotonic clock, the one the log's stamps use) when the frame arrived."""


def _map_file(path: str, size: int, create: bool) -> mmap.mmap:
    """`path` mapped, `size` bytes; `create` makes the file (a new file reads as zeros), else it must exist."""
    fd = os.open(path, os.O_RDWR | (os.O_CREAT | os.O_EXCL if create else 0), 0o600)
    try:
        if create:
            os.ftruncate(fd, size)
        return mmap.mmap(fd, size)
    finally:
        os.close(fd)


class FrameSlot:
    """One camera's newest frame in shared memory: a file (under /dev/shm) mapped by the capture process, which
    `write()`s every frame into it, and by the robot's process, which `read()`s a copy. Two buffers, written in turn,
    and a sequence number per buffer: `lock` guards the headers (microseconds, and the memory fences), and a reader
    copies the newest buffer without it, then checks its sequence number again. A writer that came back to that
    buffer meanwhile (two frames later) spoils the copy, and the second copy is made holding the lock: the writer
    can't publish, so can't come back, until it is done. A reader always finishes in at most two copies; a writer
    waits for a copy only when a reader was that slow (seen at 60 fps with the CPU busy: lock-free retries alone
    starved a reader for good).

    Frames are kept as the ZED delivers them, BGRA (4 bytes a pixel): the capture copies the camera's buffer without
    converting it, and the encoders hand it to the Jetson's VIC (`nvvidconv`) as BGRx, with no CPU colour conversion
    anywhere. `read()`/`wait()` give the 3-channel image in the config's colour space; `read_raw()`/`wait_raw()` the
    BGRA frame itself."""

    def __init__(self, path: str, shape: Tuple[int, int, int], lock, create: bool = False, rgb: bool = False) -> None:
        """Maps `path`; `create` makes the file (the capture process does, sized for `shape`), else it must exist.
        `shape` is the (height, width, 3) image `read()` gives, in RGB order with `rgb`, else BGR."""
        if len(shape) != 3 or shape[2] != 3:
            raise ValueError(f"a frame slot's shape is (height, width, 3), got {shape}")
        self.path = path
        self.shape = shape
        self.raw_shape = (shape[0], shape[1], 4)
        """The stored BGRA frame's shape, what `read_raw()` gives."""
        self._channels = slice(2, None, -1) if rgb else slice(0, 3)  # BGRA -> RGB or BGR, a view (no copy)
        self._bytes = int(np.prod(self.raw_shape))
        self._map = _map_file(path, _HEADER_SIZE + 2 * (_HEADER_SIZE + self._bytes), create)
        if create:
            _TOP.pack_into(self._map, 0, -1, 0)  # no frame yet (a new file reads as zeros: buffer 0)
        self._lock = lock
        self._images = [np.ndarray(self.raw_shape, np.uint8, self._map, self._base(i) + _HEADER_SIZE) for i in (0, 1)]

    def _base(self, i: int) -> int:
        return _HEADER_SIZE + i * (_HEADER_SIZE + self._bytes)

    def write(self, image: np.ndarray, camera_ns: int, host_ns: int) -> int:
        """Stores the frame and returns its sequence number (the previous one plus 1). `image` is BGRA (the ZED's
        own), or BGR (the test pattern), whose fourth byte is left as it is (unused: encoders read BGRx)."""
        if image.shape[:2] != self.shape[:2] or image.shape[2] not in (3, 4):
            raise ValueError(f"frame {image.shape}, slot {self.raw_shape} (BGRA) or {self.shape} (BGR)")
        with self._lock:
            latest, seq = _TOP.unpack_from(self._map)
            i = 1 - latest if latest >= 0 else 0
            _HEADER.pack_into(self._map, self._base(i), 0, 0, 0)  # being written: no reader keeps a copy
        self._images[i][:, :, :image.shape[2]] = image
        with self._lock:
            _HEADER.pack_into(self._map, self._base(i), seq + 1, camera_ns, host_ns)
            _TOP.pack_into(self._map, 0, i, seq + 1)
        return seq + 1

    def seq(self) -> int:
        """The newest frame's sequence number, 0 before the first."""
        with self._lock:
            return _TOP.unpack_from(self._map)[1]

    def header(self) -> Tuple[int, int, int]:
        """(seq, camera_ns, host_ns) of the newest frame without copying it; seq 0 before the first."""
        with self._lock:
            latest, seq = _TOP.unpack_from(self._map)
            if latest < 0 or seq == 0:
                return 0, 0, 0
            return _HEADER.unpack_from(self._map, self._base(latest))

    def read(self, out: Optional[np.ndarray] = None) -> Optional[Frame]:
        """A copy of the newest frame, or None before the first. With `out`, an array of the frame's shape, the copy
        goes there and the frame's image is `out` itself (a consumer reusing its buffer skips a 5-7 MB allocation per
        frame, which costs more than the copy); without, the image is new and read-only."""
        return self._read(out, self._channels)

    def read_raw(self, out: Optional[np.ndarray] = None) -> Optional[Frame]:
        """`read()`, the stored BGRA frame (`raw_shape`): what the encoders take, copied without converting."""
        return self._read(out, slice(None))

    def _read(self, out: Optional[np.ndarray], channels: slice) -> Optional[Frame]:
        with self._lock:
            latest, seq = _TOP.unpack_from(self._map)
            if latest < 0 or seq == 0:
                return None
            seq, camera_ns, host_ns = _HEADER.unpack_from(self._map, self._base(latest))
        image = self._copy(latest, out, channels)
        with self._lock:
            if _HEADER.unpack_from(self._map, self._base(latest))[0] != seq:  # lapped: copy again, holding the lock
                latest = _TOP.unpack_from(self._map)[0]
                seq, camera_ns, host_ns = _HEADER.unpack_from(self._map, self._base(latest))
                image = self._copy(latest, out, channels)
        if out is None:
            image.flags.writeable = False
        return Frame(image, seq, camera_ns, host_ns)

    def _copy(self, buffer: int, out: Optional[np.ndarray], channels: slice) -> np.ndarray:
        image = self._images[buffer][:, :, channels]
        if out is None:
            return image.copy()  # contiguous
        np.copyto(out, image)
        return out

    def wait(self, after: int, timeout: float, out: Optional[np.ndarray] = None) -> Optional[Frame]:
        """The newest frame once its sequence number is above `after`, or None after `timeout` seconds without one;
        `out` as in `read()`."""
        return self.read(out) if self._wait(after, timeout) else None

    def wait_raw(self, after: int, timeout: float, out: Optional[np.ndarray] = None) -> Optional[Frame]:
        """`wait()`, the stored BGRA frame (`read_raw()`)."""
        return self.read_raw(out) if self._wait(after, timeout) else None

    def _wait(self, after: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while self.seq() <= after:
            if time.monotonic() >= deadline:
                return False
            time.sleep(_POLL)
        return True

    def close(self) -> None:
        """Unmaps the file; the service removes the files once the capture process has ended."""
        del self._images  # views keep the map open
        self._map.close()


class Imu(NamedTuple):
    """One reading of the stereo camera's IMU. Axes: +X right, +Y down, +Z forward along the optical axis."""
    seq: int
    """1 for the first reading since the service started, then 2, 3, ..."""
    camera_ns: int
    """The IMU's own time of the reading, in nanoseconds of the camera's clock."""
    host_ns: int
    """`currtime_ns()` when the capture process took it."""
    accel: Tuple[float, float, float]
    """Linear acceleration, m/s^2, gravity included."""
    gyro: Tuple[float, float, float]
    """Angular velocity, rad/s."""


def imu_path(prefix: str) -> str:
    """The stereo camera's IMU file for a service whose frames' files start with `prefix`."""
    return f"{prefix}{STEREO}.imu"


class ImuSlot:
    """The stereo camera's newest IMU reading in shared memory: the capture process `write()`s each new one (hundreds a
    second), the robot's process `read()`s it. 80 bytes, so both sides hold the shared lock for the whole access."""

    def __init__(self, path: str, lock, create: bool = False) -> None:
        self.path = path
        self._map = _map_file(path, _IMU.size, create)  # a new file reads as zeros: reading 0, none yet
        self._lock = lock

    def write(self, camera_ns: int, host_ns: int, accel: Tuple[float, float, float],
              gyro: Tuple[float, float, float]) -> None:
        with self._lock:
            seq = _IMU.unpack_from(self._map)[0]
            _IMU.pack_into(self._map, 0, seq + 1, camera_ns, host_ns, *accel, *gyro)

    def read(self) -> Optional[Imu]:
        """The newest reading, or None before the first."""
        with self._lock:
            seq, camera_ns, host_ns, ax, ay, az, wx, wy, wz = _IMU.unpack_from(self._map)
        return None if seq == 0 else Imu(seq, camera_ns, host_ns, (ax, ay, az), (wx, wy, wz))

    def close(self) -> None:
        self._map.close()


def sample_points(width: int, height: int, count: int = 10) -> Tuple[np.ndarray, np.ndarray]:
    """Fixed pixel coordinates (xs, ys) spread over an image of this size, for the blank-frame check."""
    rng = np.random.default_rng(123456)
    return rng.integers(0, width, size=count), rng.integers(0, height, size=count)


def is_blank(image: np.ndarray, points: Tuple[np.ndarray, np.ndarray]) -> bool:
    """Whether the sampled pixels are all one colour: a camera whose pipeline broke (a loose USB cable for a moment)
    delivers homogeneous frames until it is reopened."""
    pixels = image[points[1], points[0], :]
    return bool(np.all(pixels == pixels[0]))


_PATTERN_BASE = {}


def pattern_image(height: int, width: int, index: int) -> np.ndarray:
    """Frame `index` of a picture that changes every frame (a moving bar and line over fixed noise): the service
    without a camera."""
    base = _PATTERN_BASE.get((height, width))
    if base is None:
        base = _PATTERN_BASE[(height, width)] = np.random.default_rng(0).integers(32, 96, (height, width, 3), np.uint8)
    image = base.copy()
    x = (index * 8) % width
    image[:, x:x + max(1, width // 24)] = (0, 200, 255)
    image[(index * 3) % height, :] = 255
    return image
