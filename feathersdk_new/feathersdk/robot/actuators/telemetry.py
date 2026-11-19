import numpy as np
import numba

from feathersdk.quill_defs.python_qdefs.data_type_qdef import DataType
from feathersdk.utils.feathertypes import (
    Union, Tuple, TYPE_CHECKING, List, Type, overload, NumericType, IntLike, Protocol, Generic, TypeVar,
    runtime_checkable, Iterator, ensure, Dict, U, SENTINEL, Optional, SentinelType
)
from feathersdk.robot.services import perftime
from feathersdk.utils import currtime
from feathersdk.logging import log_json, NOTICE
from feathersdk.utils.constants import _HOT_PATH_SAMPLE_RATE

if TYPE_CHECKING:
    from feathersdk.robot.robot import FeatherRobot


@numba.njit(cache=True)
def _update_telemetry(telem_idx: int, n_elems: int, src_off: int, message: np.ndarray, data: np.ndarray, 
    offsets: np.ndarray, itemsizes: np.ndarray, capacities: np.ndarray, starts: np.ndarray, ends: np.ndarray, 
    counts: np.ndarray) -> None:
    """Update a single telemetry ring"""
    itemsize, capacity, base = itemsizes[telem_idx], capacities[telem_idx], offsets[telem_idx]
    start_idx, end_idx, count = starts[telem_idx], ends[telem_idx], counts[telem_idx]

    # Keep only the last `capacity` elements when the batch is oversized.
    if n_elems >= capacity:
        src_off = src_off + (n_elems - capacity) * itemsize
        n_elems = capacity
        nbytes = n_elems * itemsize
        data[base : base + nbytes] = message[src_off : src_off + nbytes]
        starts[telem_idx] = 0
        ends[telem_idx] = 0
        counts[telem_idx] = capacity
        return

    # First contiguous write from end_idx toward the end of the ring.
    space = capacity - end_idx
    first = n_elems if n_elems < space else space
    nbytes_first = first * itemsize
    dst0 = base + end_idx * itemsize
    data[dst0 : dst0 + nbytes_first] = message[src_off : src_off + nbytes_first]

    # Wrap: remaining elements land at the start of the ring.
    if first < n_elems:
        nbytes_rest = (n_elems - first) * itemsize
        src1 = src_off + nbytes_first
        data[base : base + nbytes_rest] = message[src1 : src1 + nbytes_rest]

    # Update the new count, start, and end indices
    new_count = count + n_elems
    if new_count > capacity:
        starts[telem_idx] = (start_idx + (new_count - capacity)) % capacity
        counts[telem_idx] = capacity
    else:
        counts[telem_idx] = new_count
    ends[telem_idx] = (end_idx + n_elems) % capacity


@numba.njit(cache=True)
def _update_motor_message(n_updates: int, message: np.ndarray, data: np.ndarray, offsets: np.ndarray, 
    itemsizes: np.ndarray, capacities: np.ndarray, starts: np.ndarray, ends: np.ndarray, counts: np.ndarray) -> None:
    """Apply a packed motor message to telemetry rings (nopython).

    Message format (Teensy ``GetTelemetrySingleTelemetryHeader`` x ``n_updates``):
        [u16le telem_idx][u16le n_elems][n_elems * itemsize bytes]
    """
    pos = 0

    for _ in range(n_updates):
        tidx = message[pos] | (message[pos + 1] << 8)
        nelem = message[pos + 2] | (message[pos + 3] << 8)
        payload_idx = pos + 4
        _update_telemetry(tidx, nelem, payload_idx, message, data, offsets, itemsizes, capacities, starts, ends, counts)
        pos = payload_idx + nelem * itemsizes[tidx]


def compile_kernels() -> None:
    """Compiles the telemetry kernels, or loads them from numba's cache, with the argument types `FeatherTelemetry`
    passes (a read-only message, as `np.frombuffer` gives). `FeatherRobot` calls it before connecting to the Teensy: a
    cold compile holds the GIL for seconds (about 5 on the Orin, after a code change or another environment's numba
    rewrote the cache), long enough for the Teensy to drop a client it hears nothing from for 5 s."""
    empty = np.zeros(1, np.uint64)
    _update_motor_message(0, np.frombuffer(b"", np.uint8), np.zeros(1, np.uint8), empty, empty, empty, empty, empty,
                          empty)


_TelemetryGetItemType = Union[IntLike, slice]
"""Type of index for a getitem call on telemetry."""


_DTYPE_MAP = {
    DataType.BOOL: np.bool_,
    DataType.U8: np.uint8,
    DataType.U16: np.uint16,
    DataType.U32: np.uint32,
    DataType.U64: np.uint64,
    DataType.I8: np.int8,
    DataType.I16: np.int16,
    DataType.I32: np.int32,
    DataType.I64: np.int64,
    DataType.F32: np.float32,
    DataType.F64: np.float64,
}


# Covariant: it is only read from `_value_type` and from `__getitem__`.
TTelemetryParamValue = TypeVar("TTelemetryParamValue", covariant=True, bound=NumericType)


@runtime_checkable
class TelemetryParam(Protocol, Generic[TTelemetryParamValue]):
    """Protocol for all telemetry parameters."""
    param_name: str
    """The name of the parameter."""
    pcode: int
    """The pcode of the parameter."""
    is_timestamp_param: bool
    """Whether the parameter is a timestamp parameter."""
    dtype: int
    """The data type of the parameter."""
    timestamp_pcode: int
    """The pcode of the timestamp parameter."""
    max_telemetry_size: int
    """The maximum telemetry size of the parameter."""

    @property
    def _value_type(self) -> TTelemetryParamValue:
        """The parameter's value type. Not stored; lets checkers recover `float` from `RobParam[float]`."""
        ...


TTelemetryParam = TypeVar("TTelemetryParam", bound=TelemetryParam[NumericType])
"""Typevar bound to TelemetryParam"""


class _TelemetryParameters(Protocol):
    """Protocol for all telemetry parameters."""
    def __iter__(self) -> Iterator[TelemetryParam[NumericType]]:
        """Iterate over the telemetry parameters."""
        ...
    
    def __len__(self) -> int:
        """Get the number of telemetry parameters."""
        ...
    
    def __getitem__(self, item: Union[str, int]) -> TelemetryParam[NumericType]:
        """Get a telemetry parameter by name or pcode."""
        ...

TTelemetryParameters = TypeVar("TTelemetryParameters", bound=_TelemetryParameters)
"""Typevar bound to all telemetry parameters for an actuator."""


class FeatherTelemetry:
    """Stores the telemetry data for all actuators in the system.

    Telemetries are static after calling `initialize()` and cannot be changed.
    """

    def __init__(self, robot: 'FeatherRobot'):
        self._initialized: bool = False
        self._telemetries: List[Tuple[str, int, Type[np.generic]]] = []
        self._data_types: List[DataType] = []  # per channel, for the C++ rings
        self._native = False  # the rings are the C++ link's (`use_cpp_comms`): its thread writes them
        self.robot: 'FeatherRobot' = robot
    
    def is_initialized(self) -> bool:
        """Check if the telemetry manager is initialized."""
        return self._initialized
    
    def add_telemetry(self, name: str, capacity: int, dtype: DataType) -> int:
        """Add a telemetry channel and return its index.

        Indices are assigned in call order, starting at 0. Channels cannot be added after ``initialize()``. `name` is
        `owner.parameter` (`drive_right.FB_VELOCITY`): the run's log records it per index, so the log parser names the
        channel (older recordings named them `channel_N`).
        """
        if self._initialized:
            raise ValueError("Telemetry manager already initialized")
        if dtype not in _DTYPE_MAP:
            raise ValueError(f"Unsupported dtype: {dtype}")

        self._telemetries.append((name, capacity, _DTYPE_MAP[dtype]))
        self._data_types.append(dtype)
        return len(self._telemetries) - 1

    def initialize(self) -> None:
        """Initialize the telemetry manager. Should be called after every object has registered its channels."""
        if self._initialized:
            raise ValueError("Telemetry manager already initialized")
        
        self._initialized = True
        
        if len(self._telemetries) == 0:
            raise ValueError("telemetries must be non-empty")

        # Build all the internal arrays for numba processing
        self._n_telemetries = len(self._telemetries)
        self._starts = np.zeros(self._n_telemetries, dtype=np.uint64)
        self._ends = np.zeros(self._n_telemetries, dtype=np.uint64)
        self._counts = np.zeros(self._n_telemetries, dtype=np.uint64)
        self._capacities = np.array([capacity for _, capacity, _ in self._telemetries], dtype=np.uint64)

        self._dtypes = [dtype for _, _, dtype in self._telemetries]
        self._itemsizes = np.array([dtype().itemsize for _, _, dtype in self._telemetries], dtype=np.uint64)

        byte_lens = self._itemsizes * self._capacities
        self._offsets = np.zeros(self._n_telemetries, dtype=np.uint64)
        if self._n_telemetries > 1:
            self._offsets[1:] = np.cumsum(byte_lens[:-1])
        
        # With the C++ link the rings are its memory, same layout, written by its thread: these arrays view it.
        comms = getattr(self.robot, "comms_manager", None)  # absent without a Teensy link (tests, a client robot)
        native = comms.native_telemetry if comms is not None else None
        if native is None:
            self._data = np.zeros(int(byte_lens.sum()), dtype=np.uint8)
        else:
            for capacity, data_type in zip(self._capacities, self._data_types):
                native.add(int(capacity), int(data_type))
            self._data, self._starts, self._ends, self._counts = native.initialize()
            if self._data.size != int(byte_lens.sum()):
                raise RuntimeError(f"C++ telemetry arena is {self._data.size} bytes, the Python layout "
                                   f"{int(byte_lens.sum())}")
            self._native = True

        # Typed views into `_data` (same memory as the numba uint8 arena).
        self._ring_buffers = [
            np.ndarray(
                int(self._capacities[i]),
                dtype=self._dtypes[i],
                buffer=self._data,
                offset=int(self._offsets[i]),
            )
            for i in range(self._n_telemetries)
        ]

        # Compile the numba update kernels before telemetry arrives on the comms thread.
        compile_kernels()  # done already when FeatherRobot built this: then instant

        log_json(NOTICE, "telemetry", {
            "n_telemetries": self._n_telemetries,
            "telemetries": [{"capacity": capacity, "dtype": dtype.__name__} for _, capacity, dtype in self._telemetries],
            "names": [name for name, _, _ in self._telemetries],
        })
    
    @perftime(prefix="FeatherTelemetry", sample_rate=_HOT_PATH_SAMPLE_RATE)
    def update_telemetry(self, n_updates: int, payload: bytes) -> None:
        """Update the telemetry with a motor message.

        Args:
            n_updates (int): The number of updates to apply.
            payload (bytes): Payload to update the telemetry with. Should be everything after the `num_updates` byte.
        """
        if not self._initialized:
            raise ValueError("Telemetry manager not initialized")
        if self._native:
            raise RuntimeError("update_telemetry() with the C++ link: its thread writes these rings, a second writer "
                               "would race it")
        
        motor_msg = np.frombuffer(payload, dtype=np.uint8)
        _update_motor_message(n_updates, motor_msg, self._data, self._offsets, self._itemsizes, self._capacities, 
                              self._starts, self._ends, self._counts)
        
        if n_updates > 0 and self.robot is not None and self.robot.data_monitor_service is not None \
            and self.robot.data_monitor_service.telemetry_writer is not None:
            self.robot.data_monitor_service.telemetry_writer.write_batch(currtime(), n_updates, payload)

    def _get(self, telem_index: IntLike, idx: _TelemetryGetItemType) -> Union[NumericType, np.ndarray]:
        """Read one channel. Accepts an ``int | slice`` index, unlike the overloaded ``__getitem__``."""
        if not self._initialized:
            raise ValueError("Telemetry manager not initialized")

        if not isinstance(telem_index, (int, np.integer)):
            raise TypeError(f"Telemetry index must be an integer, got {type(telem_index).__name__}")
        telem_i = int(telem_index)
        if telem_i < 0 or telem_i >= len(self._ring_buffers):
            raise IndexError(f"telemetry index {telem_i} out of range for {len(self._ring_buffers)} channels")

        buf = self._ring_buffers[telem_i]
        start = int(self._starts[telem_i])
        count = int(self._counts[telem_i])
        capacity = int(self._capacities[telem_i])

        # --- scalar index (fast path) ---
        if isinstance(idx, (int, np.integer)):
            j = int(idx) + (0 if idx >= 0 else count)
            if j < 0 or j >= count:
                raise IndexError(f"index `{idx}` out of range for telemetry {telem_i} with `{count}` elements")
            
            value: NumericType = buf[(start + j) % capacity]  # To make type checker happy
            return value

        # --- slice ---
        if isinstance(idx, slice):
            a, b, step = idx.indices(count)
            if step == 1:
                n = b - a
                if n <= 0:
                    return np.empty(0, dtype=buf.dtype)
                phys0 = (start + a) % capacity
                # Contiguous in physical memory → return a view (no copy).
                if phys0 + n <= capacity:
                    return buf[phys0 : phys0 + n]
                # Wraps once around the ring.
                first = capacity - phys0
                out = np.empty(n, dtype=buf.dtype)
                out[:first] = buf[phys0:]
                out[first:] = buf[: n - first]
                return out

            if step == 0:
                raise ValueError("slice step cannot be zero")

            # Non-unit step: gather via physical indices.
            logical = np.arange(a, b, step)
            if logical.size == 0:
                return np.empty(0, dtype=buf.dtype)
            gathered: np.ndarray = buf[(start + logical) % capacity]
            return gathered

        raise TypeError(f"Unsupported index type {type(idx).__name__}; expected int or slice")


class _TObjectWithTelemetry(Protocol, Generic[TTelemetryParameters]):
    """Protocol for all objects that have telemetry."""
    PARAMS: TTelemetryParameters
    """The telemetry parameters for this object."""

    @property
    def robot(self) -> Optional['FeatherRobot']:
        """The robot that this object belongs to. Must be set before telemetry is created."""
        ...


class ObjectTelemetry(Generic[TTelemetryParameters]):
    """Stores the telemetry information for an object."""

    PARAMS: TTelemetryParameters
    """The telemetry parameters for this object."""

    def __init__(self, object: _TObjectWithTelemetry[TTelemetryParameters], name: str) -> None:
        """`name` is the owner's name in the run's log: each channel is recorded as `name.PARAMETER`."""
        if object.PARAMS is None:
            raise ValueError("Object must define PARAMS before creating telemetry")
        assert object.robot is not None, "Object must be attached to a robot before creating telemetry"
        
        self.object = object
        """The object that this telemetry manager belongs to."""
        self.global_telemetry = ensure(object.robot.global_telemetry)
        """The global telemetry manager for this object."""
        self.param_inds: Dict[str, int] = {}
        """The global telemetry index for each parameter, keyed by ``param_name``."""
        self.telem_inds: List[int] = []
        """The global telemetry indices for the parameters, in pcode order."""
        self.PARAMS = object.PARAMS
        """The telemetry parameters for this object."""
        
        # Add telemetry entries for all parameters, and store the indices for each parameter.
        for param in sorted(self.PARAMS, key=lambda param: param.pcode):
            channel = f"{name}.{param.param_name.decode()}"
            telem_idx = self.global_telemetry.add_telemetry(channel, param.max_telemetry_size, DataType(param.dtype))
            self.param_inds[param.param_name] = telem_idx
            self.telem_inds.append(int(telem_idx))

    @overload
    def __getitem__(
        self, item: Tuple[TelemetryParam[TTelemetryParamValue], IntLike]
    ) -> TTelemetryParamValue: ...

    @overload
    def __getitem__(self, item: Tuple[TelemetryParam[TTelemetryParamValue], slice]) -> np.ndarray: ...

    def __getitem__(
        self, item: Tuple[TelemetryParam[TTelemetryParamValue], _TelemetryGetItemType]
    ) -> Union[NumericType, np.ndarray]:
        return self.global_telemetry._get(self.param_inds[item[0].param_name], item[1])

    @overload
    def getitem(
        self, key: TelemetryParam[TTelemetryParamValue], idx: IntLike
    ) -> TTelemetryParamValue: ...

    @overload
    def getitem(
        self, key: TelemetryParam[TTelemetryParamValue], idx: IntLike, default: SentinelType
    ) -> TTelemetryParamValue: ...

    @overload
    def getitem(
        self, key: TelemetryParam[TTelemetryParamValue], idx: IntLike, default: U
    ) -> Union[TTelemetryParamValue, U]: ...

    @overload
    def getitem(
        self, key: TelemetryParam[TTelemetryParamValue], idx: slice
    ) -> np.ndarray: ...

    @overload
    def getitem(
        self, key: TelemetryParam[TTelemetryParamValue], idx: slice, default: SentinelType
    ) -> np.ndarray: ...

    @overload
    def getitem(
        self, key: TelemetryParam[TTelemetryParamValue], idx: slice, default: U
    ) -> Union[np.ndarray, U]: ...

    def getitem(
        self, 
        key: TelemetryParam[TTelemetryParamValue], 
        idx: _TelemetryGetItemType, 
        default: Union[SentinelType, U] = SENTINEL,
    ) -> Union[NumericType, np.ndarray, U]:
        """Get telemetry value(s) for the given key.
        
        Allows scalar indexing and slicing. Returns the default value if given and the index is out of range.

        Args:
            key: The name of the telemetry channel.
            idx: The index or slice to get.
            default: The default value to return if `idx` is out of range. If no default is provided, error is raised.
        """
        try:
            return self.global_telemetry._get(self.param_inds[key.param_name], idx)
        except IndexError as e:
            if default is SENTINEL or isinstance(default, SentinelType):
                raise e
            return default
    
    @overload
    def get_latest_value(self, key: TelemetryParam[TTelemetryParamValue]) -> TTelemetryParamValue: ...

    @overload
    def get_latest_value(
        self, key: TelemetryParam[TTelemetryParamValue], default: SentinelType
    ) -> TTelemetryParamValue: ...

    @overload
    def get_latest_value(
        self, key: TelemetryParam[TTelemetryParamValue], default: U
    ) -> Union[TTelemetryParamValue, U]: ...

    def get_latest_value(
        self, key: TelemetryParam[TTelemetryParamValue], default: Union[SentinelType, U] = SENTINEL
    ) -> Union[TTelemetryParamValue, U]:
        """Get the latest value of a telemetry parameter by name. Can pass either a normal param or timestamp param."""
        if isinstance(default, SentinelType):
            return self.getitem(key, -1)
        return self.getitem(key, -1, default=default)

    @overload
    def get_latest_timestamp(self, key: TelemetryParam[TTelemetryParamValue]) -> float: ...

    @overload
    def get_latest_timestamp(self, key: TelemetryParam[TTelemetryParamValue], default: SentinelType) -> float: ...

    @overload
    def get_latest_timestamp(self, key: TelemetryParam[TTelemetryParamValue], default: U) -> Union[float, U]: ...

    def get_latest_timestamp(
        self, key: TelemetryParam[TTelemetryParamValue], default: Union[SentinelType, U] = SENTINEL
    ) -> Union[float, U]:
        """Get the latest timestamp of a telemetry parameter by name."""
        use_key = key if key.is_timestamp_param else self.PARAMS[key.timestamp_pcode]

        try:
            return float(self[use_key, -1])
        except IndexError as e:
            if default is SENTINEL or isinstance(default, SentinelType):
                raise e
            return default

    @overload
    def get_value_at(self, key: TelemetryParam[TTelemetryParamValue], index: int) -> TTelemetryParamValue: ...

    @overload
    def get_value_at(
        self, key: TelemetryParam[TTelemetryParamValue], index: int, default: SentinelType
    ) -> TTelemetryParamValue: ...

    @overload
    def get_value_at(
        self, key: TelemetryParam[TTelemetryParamValue], index: int, default: U
    ) -> Union[TTelemetryParamValue, U]: ...

    def get_value_at(
        self, key: TelemetryParam[TTelemetryParamValue], index: int, default: Union[SentinelType, U] = SENTINEL
    ) -> Union[TTelemetryParamValue, U]:
        """Get the value of a telemetry parameter at a given index."""
        if isinstance(default, SentinelType):
            return self.getitem(key, index)
        return self.getitem(key, index, default=default)

    @overload
    def get_timestamp_at(self, key: TelemetryParam[TTelemetryParamValue], index: int) -> TTelemetryParamValue: ...

    @overload
    def get_timestamp_at(
        self, key: TelemetryParam[TTelemetryParamValue], index: int, default: SentinelType
    ) -> float: ...

    @overload
    def get_timestamp_at(
        self, key: TelemetryParam[TTelemetryParamValue], index: int, default: U
    ) -> Union[float, U]: ...

    def get_timestamp_at(
        self, key: TelemetryParam[TTelemetryParamValue], index: int, default: Union[SentinelType, U] = SENTINEL
    ) -> Union[float, U]:
        """Get the timestamp of a telemetry parameter at a given index."""
        use_key = key if key.is_timestamp_param else self.PARAMS[key.timestamp_pcode]

        try:
            return float(self[use_key, index])
        except IndexError as e:
            if default is SENTINEL or isinstance(default, SentinelType):
                raise e
            return default

