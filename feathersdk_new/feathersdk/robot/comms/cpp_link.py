"""The Teensy link in C++ (`CommsManagerConfig.use_cpp_comms`): the backend's `TeensyManager` owns the socket and a
thread that receives, decodes and applies every reply outside the GIL, so a busy Python process (IK, a web server,
video) no longer delays the motors' feedback, the acks `write_param()` waits for, or the clock sync.

What stays in Python: rebuilding and flashing the firmware and checking its source hash (as `_TeensyHandler`), and
writing the data monitor's recordings, which the C++ thread queues (`RecordQueue`) and a Python thread here drains.
The telemetry rings are the C++ `Telemetry`'s memory; `FeatherTelemetry` views them as its numpy arrays, so readers
are unchanged. The library is `backend/build/libfeather_backend.so`, built by `make -B -C backend lib` when its
sources' content changed: not by file times, since the Orin has no working clock across boots (it has read a date
days behind), so every file looked newer than the library and make would rebuild it on every start.
"""
import ctypes
import fcntl
import hashlib
import struct
import subprocess
import threading
from pathlib import Path
from typing import TYPE_CHECKING, List, Optional, Tuple, Union

import numpy as np

from feathersdk.logging import NOTICE, error, log_json, notice, warning
from feathersdk.quill_defs.python_qdefs.teensy_commands_qdef import (
    GetBoardTimeResponsePayload, SendCanPayload, TeensyCommandOpcode,
)
from feathersdk.utils import Abortable, FeatherThread, currtime

if TYPE_CHECKING:
    from .comms_manager import CommsManager, TeensyResponseType

_BACKEND_DIR = Path(__file__).resolve().parents[2] / "backend"
_LIB_PATH = _BACKEND_DIR / "build" / "libfeather_backend.so"
_LIB_HASH_PATH = _LIB_PATH.with_name(_LIB_PATH.name + ".sourcehash")

_ERR_CAP = 1024
_MAX_RESPONSE_PAYLOAD = 512  # TEENSY_MAX_RESPONSE_PAYLOAD_SIZE
_RECORD_HEADER = struct.Struct("<BxHIqI4x")  # RecordHeader in teensy_manager.hpp
_RECORD_TELEMETRY, _RECORD_CANDUMP, _RECORD_RX_MESSAGE, _RECORD_ERROR, _RECORD_WARNING = range(5)
_DRAIN_CAP = 4 * 1024 * 1024  # RecordQueue's capacity: one drain takes everything queued
_RECORDS_HZ = 50.0
"""How often the recordings are written out. Lateness here delays only the files, never the link."""


class _FbTeensyConfig(ctypes.Structure):
    """`FbTeensyConfig` in backend/src/bindings/c_api.cpp."""
    _fields_ = [
        ("teensy_connect_timeout", ctypes.c_double),
        ("teensy_telemetry_update_frequency", ctypes.c_double),
        ("teensy_candump_update_frequency", ctypes.c_double),
        ("teensy_board_clock_sync_hz", ctypes.c_double),
        ("max_can_rate", ctypes.c_double),
        ("can_leniency", ctypes.c_double),
        ("enable_teensy_debug_logging", ctypes.c_int32),
        ("teensy_correct_clock_drift", ctypes.c_int32),
        ("teensy_expected_clock_drift_ns", ctypes.c_int32),
        ("teensy_max_clock_sync_error_ns", ctypes.c_int32),
        ("teensy_clock_sync_trials", ctypes.c_int32),
        ("use_emulated_teensy", ctypes.c_int32),
        ("server_run_time_nanos", ctypes.c_int32),
        ("can_read_run_time_nanos", ctypes.c_int32),
    ]


_lib: Optional[ctypes.CDLL] = None
_lib_lock = threading.Lock()


def load_library() -> ctypes.CDLL:
    """Build the backend library if its sources changed (by content), load it once, and declare its functions.

    Raises:
        RuntimeError: the build failed; the message holds the compiler's output.
    """
    global _lib
    with _lib_lock:
        if _lib is not None:
            return _lib
        _LIB_PATH.parent.mkdir(exist_ok=True)
        with open(_LIB_PATH.with_name(".build.lock"), "w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)  # another process starting at the same time builds once, not twice
            _build_if_changed()
        lib = ctypes.CDLL(str(_LIB_PATH))  # CDLL, not PyDLL: every call releases the GIL
        P, u64p = ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint64)
        sigs = {
            "fb_telemetry_new": ([], P),
            "fb_telemetry_add": ([P, ctypes.c_uint64, ctypes.c_int], ctypes.c_int),
            "fb_telemetry_initialize": ([P], ctypes.c_bool),
            "fb_telemetry_buffers": ([P, ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
                                      ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(u64p),
                                      ctypes.POINTER(u64p), ctypes.POINTER(u64p), ctypes.POINTER(ctypes.c_size_t)],
                                     ctypes.c_bool),
            "fb_teensy_new": ([ctypes.POINTER(_FbTeensyConfig), P, ctypes.c_char_p, ctypes.c_size_t], P),
            "fb_teensy_close": ([P], None),
            "fb_teensy_poll_err": ([P], ctypes.c_bool),
            "fb_teensy_num_can_lines": ([P], ctypes.c_int),
            "fb_teensy_clock_sync_bound_ns": ([P], ctypes.c_int64),
            "fb_teensy_src_hash_text": ([P, ctypes.c_char_p, ctypes.c_size_t], ctypes.c_long),
            "fb_teensy_send": ([P, ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_bool], ctypes.c_uint32),
            "fb_teensy_get_response": ([P, ctypes.c_uint32, ctypes.c_int64, ctypes.POINTER(ctypes.c_int),
                                        ctypes.c_char_p, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32),
                                        ctypes.POINTER(ctypes.c_int64)], ctypes.c_int),
            "fb_teensy_set_recording": ([P, ctypes.c_bool, ctypes.c_bool, ctypes.c_bool], None),
            "fb_teensy_drain_records": ([P, ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint64)],
                                        ctypes.c_size_t),
        }
        for name, (argtypes, restype) in sigs.items():
            fn = getattr(lib, name)
            fn.argtypes, fn.restype = argtypes, restype
        _lib = lib
        return lib


def _build_if_changed() -> None:
    """Run make when the sources' content differs from what the library was built from."""
    source_hash = _source_hash()
    built_hash = _LIB_HASH_PATH.read_text() if _LIB_HASH_PATH.exists() and _LIB_PATH.exists() else None
    if built_hash == source_hash:
        return
    notice(f"Building {_LIB_PATH} (its sources changed) ...")
    result = subprocess.run(["make", "-B", "-s", "-C", str(_BACKEND_DIR), "lib"], capture_output=True,
                            text=True)
    if result.returncode != 0:
        raise RuntimeError(f"Building {_LIB_PATH} failed (make -B -C {_BACKEND_DIR} lib):\n{result.stdout}"
                           f"{result.stderr}")
    _LIB_HASH_PATH.write_text(source_hash)


def _source_hash() -> str:
    """MD5 over the library's sources by content: the backend's own code (not the firmware copy under comms/teensy),
    its generated headers and the Makefile, in path order."""
    def is_source(path: Path) -> bool:
        rel = path.relative_to(_BACKEND_DIR).as_posix()
        return path.suffix in (".cpp", ".hpp") and ("comms/teensy/" not in rel or "comms/teensy/host/" in rel)
    files = sorted(p for p in (_BACKEND_DIR / "src").rglob("*") if is_source(p))
    files += sorted((_BACKEND_DIR / "qdefs").glob("*.hpp")) + [_BACKEND_DIR / "Makefile"]
    digest = hashlib.md5()
    for path in files:
        digest.update(path.relative_to(_BACKEND_DIR).as_posix().encode() + b"\0" + path.read_bytes())
    return digest.hexdigest()


class NativeTelemetry:
    """The C++ telemetry rings. `FeatherTelemetry` adds its channels here and views the arena as its own arrays.
    Never freed: those numpy views (and the C++ link writing into it) live as long as the process."""

    def __init__(self) -> None:
        self._lib = load_library()
        self.ptr = self._lib.fb_telemetry_new()
        if not self.ptr:
            raise MemoryError("fb_telemetry_new failed")

    def add(self, capacity: int, dtype: int) -> int:
        """Add a channel; returns its index, the same as the Python side's (both count from 0 in call order)."""
        index = self._lib.fb_telemetry_add(self.ptr, capacity, dtype)
        if index < 0:
            raise ValueError(f"C++ telemetry refused a channel: capacity {capacity}, dtype {dtype}")
        return index

    def initialize(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Allocate the rings; returns numpy views of (data, starts, ends, counts) over the C++ memory. The poll
        thread starts asking for telemetry from here on."""
        if not self._lib.fb_telemetry_initialize(self.ptr):
            raise ValueError("C++ telemetry initialize failed (already initialized, or no channels)")
        data, size = ctypes.POINTER(ctypes.c_uint8)(), ctypes.c_size_t()
        starts, ends, counts = (ctypes.POINTER(ctypes.c_uint64)() for _ in range(3))
        n = ctypes.c_size_t()
        if not self._lib.fb_telemetry_buffers(self.ptr, ctypes.byref(data), ctypes.byref(size), ctypes.byref(starts),
                                              ctypes.byref(ends), ctypes.byref(counts), ctypes.byref(n)):
            raise RuntimeError("C++ telemetry has no buffers after initialize")
        return (np.ctypeslib.as_array(data, (size.value,)), np.ctypeslib.as_array(starts, (n.value,)),
                np.ctypeslib.as_array(ends, (n.value,)), np.ctypeslib.as_array(counts, (n.value,)))


class CppTeensyHandler(Abortable):
    """`_TeensyHandler`'s interface over the C++ `TeensyManager`. See the module docstring."""

    def __init__(self, comms_manager: 'CommsManager'):
        from .comms_manager import (
            NUM_CAN_ENDPOINTS_MAX, TeensyError, _check_NIC_interrupt_coalescing, _LIB_BIN_EM, _LIB_BIN_REAL,
        )
        from .teensy.teensy_flash import resolve_teensy_host
        super().__init__()
        self.comms = comms_manager
        config = comms_manager.config
        self._lib = load_library()
        self._TeensyError = TeensyError

        if not config.use_emulated_teensy:
            host = resolve_teensy_host()
            if host is None:
                raise RuntimeError("feather-teensy.local did not resolve. Check the Ethernet link and libnss-mdns.")
            _check_NIC_interrupt_coalescing(host=host)

        self.telemetry = NativeTelemetry()
        c = _FbTeensyConfig(
            config.teensy_connect_timeout, config.teensy_telemetry_update_frequency,
            config.teensy_candump_update_frequency, config.teensy_board_clock_sync_hz, config.max_can_rate,
            config.can_leniency, config.enable_teensy_debug_logging, config.teensy_correct_clock_drift,
            config.teensy_expected_clock_drift_ns, config.teensy_max_clock_sync_error_ns,
            config.teensy_clock_sync_trials, config.use_emulated_teensy, config.server_run_time_nanos,
            config.can_read_run_time_nanos,
        )
        err = ctypes.create_string_buffer(_ERR_CAP)
        self._m = self._lib.fb_teensy_new(ctypes.byref(c), self.telemetry.ptr, err, _ERR_CAP)
        if not self._m:
            raise TeensyError(f"C++ Teensy link failed to start: {err.value.decode(errors='replace')}")

        # Same check as _TeensyHandler: a stale binary is deleted so the next run rebuilds it.
        text = ctypes.create_string_buffer(_MAX_RESPONSE_PAYLOAD)
        n = self._lib.fb_teensy_src_hash_text(self._m, text, _MAX_RESPONSE_PAYLOAD)
        src_hash = text.raw[:max(n, 0)].decode("utf-8", errors="replace")
        if src_hash != comms_manager.teensy_source_hash:
            (_LIB_BIN_EM if config.use_emulated_teensy else _LIB_BIN_REAL).unlink(missing_ok=True)
            self._close()
            raise TeensyError(f"Teensy source hash mismatch: {src_hash} != {comms_manager.teensy_source_hash}\n\t"
                              "Re-run the program to rebuild the Teensy library.")
        comms_manager.num_can_endpoints = num_can = self._lib.fb_teensy_num_can_lines(self._m)
        if not 1 <= num_can <= NUM_CAN_ENDPOINTS_MAX:
            self._close()
            raise TeensyError(f"Teensy reported {num_can} CAN lines; expected 1 to {NUM_CAN_ENDPOINTS_MAX}")
        log_json(NOTICE, "teensy_info", {"src_hash_str": src_hash, "num_can_endpoints": num_can})
        notice(f"C++ Teensy link up: {comms_manager.num_can_endpoints} CAN lines, clock sync bound "
               f"{self._lib.fb_teensy_clock_sync_bound_ns(self._m) / 1e6:.3f} ms.")

        # Hot path: bound once.
        self._send = self._lib.fb_teensy_send
        self._get = self._lib.fb_teensy_get_response
        self._drain_buf = ctypes.create_string_buffer(_DRAIN_CAP)
        self._recording: Tuple[bool, bool, bool] = (False, False, False)
        self._reported_poll_err = False
        self._records_thread = FeatherThread(step_target=self._records_step, operating_frequency=_RECORDS_HZ).start()

    def send_message(
        self, opcode: int, payload: bytes, block: bool = False, timeout: float = 0.1, want_response: bool = False,
    ) -> Union[int, Tuple[int, 'TeensyResponseType', float]]:
        """`_TeensyHandler.send_message`. A non-blocking send's reply is freed on arrival unless `want_response` (a
        caller that takes it later with `get_response()`): nothing else could take it.

        Raises:
            TeensyError: the send failed (socket closed, payload over 500 bytes, all 1024 reply slots in use).
        """
        message_id = self._send(self._m, opcode, payload, len(payload), block or want_response)
        if message_id == 0:
            raise self._TeensyError(f"C++ Teensy link could not send opcode {opcode} ({len(payload)} bytes): "
                                    f"{'socket closed' if self._lib.fb_teensy_poll_err(self._m) else 'see the log'}")
        dms = self.comms.robot.data_monitor_service
        if dms is not None and dms.teensy_message_writer is not None and opcode:
            dms.teensy_message_writer.write_message(currtime(), True, message_id, opcode, payload)
        if not block:
            return message_id
        response = self.get_response(message_id, timeout)
        if response is None:
            raise TimeoutError(f"Timeout waiting for response to message `{message_id}`")
        if response[0] == TeensyCommandOpcode.RESP_ERROR:
            raise self._TeensyError(f"Received error response for message `{message_id}`: {response[1]}")
        return response

    def send_message_await(
        self, opcode: int, payload: bytes, timeout: float = 0.1
    ) -> Tuple[int, 'TeensyResponseType', float]:
        """`_TeensyHandler.send_message_await`."""
        response = self.send_message(opcode, payload, block=True, timeout=timeout)
        assert not isinstance(response, int)
        return response

    def cansend(
        self, bus: int, extended: bool, can_id: int, data: bytes, block: bool = False, timeout: float = 0.1
    ) -> Union[int, Tuple[int, 'TeensyResponseType', float]]:
        """`_TeensyHandler.cansend`. The returned id's reply is kept for `get_response()`."""
        padded = bytes(data) + bytes(8 - len(data))
        payload = SendCanPayload._STRUCT.pack(bus, bool(extended), len(data), can_id, *padded)
        return self.send_message(TeensyCommandOpcode.CMD_SEND_CAN, payload, block=block, timeout=timeout,
                                 want_response=True)

    def get_response(self, message_id: int, timeout: float = 0.0
                     ) -> Optional[Tuple[int, 'TeensyResponseType', float]]:
        """`_TeensyHandler.get_response`: (opcode, value, receive time), or None by the timeout. Waits in C++."""
        opcode, size, received = ctypes.c_int(), ctypes.c_uint32(), ctypes.c_int64()
        payload = ctypes.create_string_buffer(_MAX_RESPONSE_PAYLOAD)
        if not self._get(self._m, message_id, int(timeout * 1e9), ctypes.byref(opcode), payload,
                         _MAX_RESPONSE_PAYLOAD, ctypes.byref(size), ctypes.byref(received)):
            return None
        code, body, t = opcode.value, payload.raw[:size.value], received.value * 1e-9
        if code in (TeensyCommandOpcode.RESP_ERROR, TeensyCommandOpcode.RESP_ECHO):
            return code, body.decode("utf-8", errors="replace"), t
        if code == TeensyCommandOpcode.RESP_BOARD_TIME:
            return code, GetBoardTimeResponsePayload._STRUCT.unpack(body)[0], t
        return TeensyCommandOpcode.RESP_SUCCESS, 1, t

    def _records_step(self) -> None:
        """Write what the C++ thread queued into the data monitor's files and the log."""
        if self._lib.fb_teensy_poll_err(self._m) and not self._reported_poll_err:
            self._reported_poll_err = True
            error("C++ Teensy link stopped: socket or framing error (see the backend log). Sends now raise.")

        dms = self.comms.robot.data_monitor_service
        telem_w = dms.telemetry_writer if dms is not None else None
        candump_w = dms.candump_writer if dms is not None else None
        msg_w = dms.teensy_message_writer if dms is not None else None
        recording = (telem_w is not None, candump_w is not None, msg_w is not None)
        if recording != self._recording:
            self._recording = recording
            self._lib.fb_teensy_set_recording(self._m, *recording)

        dropped = ctypes.c_uint64()
        n = self._lib.fb_teensy_drain_records(self._m, self._drain_buf, _DRAIN_CAP, ctypes.byref(dropped))
        if dropped.value:
            warning(f"C++ Teensy link dropped {dropped.value} records: the recording thread fell behind")
        view = memoryview(self._drain_buf)[:n]
        pos = 0
        while pos < n:
            kind, opcode, msg_id, ts_ns, size = _RECORD_HEADER.unpack_from(view, pos)
            pos += _RECORD_HEADER.size
            body = bytes(view[pos:pos + size])
            pos += size
            ts = ts_ns * 1e-9
            if kind == _RECORD_TELEMETRY and telem_w is not None:
                telem_w.write_batch(ts, opcode, body)
            elif kind == _RECORD_CANDUMP and candump_w is not None:
                candump_w.write(body)
            elif kind == _RECORD_RX_MESSAGE and msg_w is not None:
                msg_w.write_message(ts, False, msg_id, opcode, body)
            elif kind == _RECORD_ERROR:
                error(body.decode("utf-8", errors="replace"))
            elif kind == _RECORD_WARNING:
                warning(body.decode("utf-8", errors="replace"))

    def _close(self) -> None:
        # Stop the poll thread and shut the connection down, but never free the manager: other threads may still be
        # sending, and a send to a shut-down link fails (TeensyError) where one to freed memory would crash the process.
        self._lib.fb_teensy_close(self._m)

    def _abort_impl(self) -> None:
        try:
            self._records_thread.terminate(block=True)
            self._records_step()  # what was queued before the stop
        finally:
            self._close()
