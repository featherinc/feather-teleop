import socket
import threading
import time
from feathersdk.configs import PyDBaseModel, PyDField, Configurable, pyd_model_validator
from feathersdk.utils import Abortable, KB, FeatherThread, currtime, exponential_timeout, FeatherShellProcess
from feathersdk.utils.system import detect_ethernet_interface
from feathersdk.utils.constants import _HOT_PATH_SAMPLE_RATE
from feathersdk.utils.feathertypes import Union, Tuple, TYPE_CHECKING, Optional, Self, Dict, cast, ensure
from feathersdk.logging import error, warning, log_json, NOTICE, notice
from .teensy.recompilation import recompile_if_needed as recompile_teensy_if_needed, _LIB_BIN_REAL, _LIB_BIN_EM
from .teensy.teensy_flash import resolve_teensy_host
from feathersdk.robot.services import perftime
from feathersdk.quill_defs.python_qdefs.teensy_commands_qdef import (
    ClientCommandHeader, TeensyCommandOpcode, InitializeConfig, SetLoggingPayload, GetBoardTimeResponsePayload,
    GetNumCanLinesResponsePayload, SetTimestampOffsetPayload, SendCanPayload, TEENSY_PORT,
)
from feathersdk.quill_defs.python_qdefs.telemetry_qdef import GetTelemetryPayloadHeader
from feathersdk.utils.system import SystemBoard, get_system_board

if TYPE_CHECKING:
    from .cpp_link import CppTeensyHandler, NativeTelemetry
    from feathersdk.robot.actuators.telemetry import FeatherTelemetry
    from feathersdk.robot.robot import FeatherRobot


EMPTY_PAYLOAD = bytes()
"""Empty payload to send when a message doesn't have a payload."""


NUM_CAN_ENDPOINTS_MAX = 8
"""Absolute max CAN lines (candump bus field width / emulator ``MAX_NUM_CAN_LINES``).

Prefer ``CommsManager.num_can_endpoints``, queried from the Teensy at connect time.
"""

RESULT_BUFFER_SIZE = 100 * KB
"""The size of the result buffer for Teensy responses in bytes."""

HEADER_SIZE = ClientCommandHeader._STRUCT.size
"""The size of the header for a Teensy response message in bytes."""


TeensyResponseType = Union[int, str]
"""Possible payload types stored for a Teensy response message."""


class CommsManagerConfig(PyDBaseModel):
    """Configuration for the comms manager."""
    teensy_connect_timeout: float = PyDField(default=10.0, ge=0.1, le=60.0)
    """The timeout in seconds to wait for the Teensy to connect."""
    teensy_operating_frequency: float = PyDField(default=300.0, ge=1.0, le=1000.0)
    """The frequency at which the comms manager will attempt to process results from the teensy socket."""
    teensy_telemetry_update_frequency: float = PyDField(default=300.0, ge=1.0, le=1000.0)
    """The frequency at which the comms manager will attempt to request motor telemetry updates from the Teensy."""
    teensy_candump_update_frequency: float = PyDField(default=30.0, ge=1.0, le=1000.0)
    """The frequency at which the comms manager will attempt to request candump updates from the Teensy."""
    enable_teensy_debug_logging: bool = PyDField(default=False)
    """Whether to enable debug logging from the Teensy over serial."""
    teensy_correct_clock_drift: bool = PyDField(default=True)
    """Whether to correct the clock drift of the Teensy."""
    teensy_board_clock_sync_hz: float = PyDField(default=10.0, ge=1.0, le=1000.0)
    """The frequency at which the comms manager will attempt to sync the Teensy clock to the host clock."""
    teensy_expected_clock_drift_ns: int = PyDField(default=100_000, ge=1000, le=10_000_000)
    """The expected clock drift of the Teensy in nanoseconds per second. This is used to determine when we will update
       the clock offset on the Teensy to re-sync the clock to the jetson."""
    teensy_max_clock_sync_error_ns: int = PyDField(default=12_000_000, ge=1000, le=20_000_000)
    """The maximum error in nanoseconds between the Teensy and host clocks that is acceptable for clock sync."""
    teensy_clock_sync_trials: int = PyDField(default=30, ge=1, le=1000)
    """The number of trials to attempt to sync the Teensy clock to the host clock."""
    teensy_check_unused_messages_interval: float = PyDField(default=10.0, ge=0.1, le=60.0)
    """The interval in seconds to check for unused message responses."""
    teensy_max_rx_messages_per_tick: int = PyDField(default=32, ge=1, le=1000)
    """The maximum number of message responses to process per tick."""
    use_cpp_comms: bool = PyDField(default=False)
    """Run the Teensy link in C++ (`cpp_link.py`): its thread receives every reply and writes the telemetry outside the
       GIL, so a busy Python process no longer delays feedback and acks. Off: the Python thread here (`_TeensyHandler`).
       `teensy_operating_frequency`, `teensy_max_rx_messages_per_tick` and `teensy_check_unused_messages_interval` are
       the Python thread's only; the C++ one wakes on each arrival."""
    use_emulated_teensy: bool = PyDField(default=False)
    """Whether to use the emulated Teensy running on the host machine instead of a real connected Teensy 4.1 board.
       This requires you to have the can usb devices connected to the host machine."""

    max_can_rate: float = PyDField(default=3500.0, ge=10.0, le=3500.0)
    """The maximum messages per second that can be sent on a CAN interface. Sent to the Teensy on initialize and used
       for host-side CAN rate limiting."""
    can_leniency: float = PyDField(default=100.0, ge=1.0, le=3500.0)
    """Leniency for CAN bus overload checks. Must be <= `max_can_rate`. Sent to the Teensy on initialize."""

    server_run_time_nanos: int = PyDField(default=100_000, ge=1, le=30_000_000)
    """Max time in nanoseconds the Teensy may spend servicing the ethernet server loop per tick. Sent on initialize."""
    can_read_run_time_nanos: int = PyDField(default=100_000, ge=1, le=30_000_000)
    """Max time in nanoseconds the Teensy may spend polling CAN buses per tick. Sent on initialize."""

    result_operating_frequency: float = PyDField(default=200.0, ge=1.0, le=1000.0)
    """The frequency at which the comms manager result processing thread will attempt to process results from the 
       socket result buffer."""
    await_thread_start_timeout: float = PyDField(default=1.0, ge=0.1, le=60.0)
    """The timeout in seconds to wait for the processing thread to start."""
    processing_toggle_timeout: float = PyDField(default=0.3, ge=0.01, le=3.0)
    """The timeout in seconds to wait for the processing thread to toggle processing when making changes to the
       processing state."""

    @pyd_model_validator(mode='after')
    def validate_can_leniency_vs_rate(self) -> Self:
        if self.can_leniency > self.max_can_rate:
            raise ValueError(f"can_leniency ({self.can_leniency}) must be <= max_can_rate ({self.max_can_rate})")
        return self


class CommsManager(Configurable[CommsManagerConfig], Abortable):
    """Handles communication"""

    def __init__(self, config: CommsManagerConfig, robot: 'FeatherRobot'):
        super().__init__(config)
        self.robot = robot
        self.teensy_bin_path, self.teensy_source_hash = recompile_teensy_if_needed(self.config.use_emulated_teensy)
        # Filled by ``_TeensyHandler`` via ``CMD_GET_NUM_CAN_LINES`` before any CAN traffic.
        self.num_can_endpoints: int = 0

        # Launch the emulated teensy handler if using
        if self.config.use_emulated_teensy:
            self._emulated_teensy_proc = FeatherShellProcess([str(self.teensy_bin_path)], name="EmulatedTeensy").start()
            time.sleep(0.3)  # Wait for the emulated teensy to start up

        self._teensy_handler: Union[_TeensyHandler, 'CppTeensyHandler']
        if self.config.use_cpp_comms:
            from .cpp_link import CppTeensyHandler
            self._teensy_handler = CppTeensyHandler(self)
        else:
            self._teensy_handler = _TeensyHandler(self)

    @property
    def native_telemetry(self) -> Optional['NativeTelemetry']:
        """The C++ telemetry rings `FeatherTelemetry` must view, with `use_cpp_comms`; None otherwise."""
        handler = self._teensy_handler
        return None if isinstance(handler, _TeensyHandler) else handler.telemetry

    @perftime(prefix="CommsManager", sample_rate=_HOT_PATH_SAMPLE_RATE)
    def send_teensy_message(self, opcode: int, payload: bytes, block: bool = False, timeout: float = 0.1) -> None:
        """Send a message to the Teensy. See ``_TeensyHandler.send_message``."""
        self._teensy_handler.send_message(opcode, payload, block=block, timeout=timeout)

    def cansend(
        self, bus: int, extended: bool, can_id: int, data: bytes, block: bool = False, timeout: float = 0.1
    ) -> Union[int, Tuple[int, TeensyResponseType, float]]:
        """Send a plain CAN frame via the Teensy. See ``_TeensyHandler.cansend``."""
        return self._teensy_handler.cansend(bus, extended, can_id, data, block=block, timeout=timeout)

    def _abort_impl(self) -> None:
        self._teensy_handler.abort()


_UNTRACKED_TEENSY_TX_OPCODES: 'frozenset[TeensyCommandOpcode]' = frozenset()
"""Send opcodes to not track by data monitor service."""

_UNTRACKED_TEENSY_RX_OPCODES = frozenset({
    TeensyCommandOpcode.RESP_ACTUATOR_TELEMETRY,
    TeensyCommandOpcode.RESP_CANDUMP,
})
"""Response opcodes to not track by data monitor service."""

_EMPTY_BYTES = bytes()
"""Empty bytes to send when a message is not tracked by the data monitor service."""


class _TeensyHandler(Abortable):
    """Handles communication with the Teensy."""

    def __init__(self, comms_manager: 'CommsManager'):
        super().__init__()
        self.comms = comms_manager
        self.config = comms_manager.config
        self._global_telemetry: Optional["FeatherTelemetry"] = None

        # The emulator listens on localhost. A real board is found by mDNS (feather-teensy.local).
        host: Optional[str] = "127.0.0.1"
        if not self.config.use_emulated_teensy:
            host = resolve_teensy_host()
            if host is None:
                raise RuntimeError("feather-teensy.local did not resolve. Check the Ethernet link and libnss-mdns.")
            notice(f"Connecting to Teensy at {host}.")

        _check_NIC_interrupt_coalescing(host=host)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm.
        self._socket.settimeout(self.config.teensy_connect_timeout)
        self._socket.connect((host, TEENSY_PORT))
        self._socket.setblocking(False)

        # Holds messages we have sent to the Teensy and when, so we can check for delays.
        # Callers on other threads register sends and take responses while this thread records replies.
        self._messages_lock = threading.Lock()
        self._next_message_id = 0
        self._sent_messages: Dict[int, float] = {}

        # Hold responses from the Teensy, to be processed by caller
        self._responses: Dict[int, Tuple[int, TeensyResponseType, float]] = {}  

        # Receive buffer for teensy responses.
        self._buffer = bytearray(RESULT_BUFFER_SIZE)
        self._buf_memview = memoryview(self._buffer)
        self._buf_nbytes = 0

        # In-progress message state (partial header / payload across reads).
        self._cur_msg_id: Optional[int] = None  # None means no message is in progress.
        self._cur_opcode = 0
        self._cur_pyld_size = 0

        # Keep track of the last time we requested a telemetry update.
        self._last_telemetry_update = 0.0
        self._last_candump_update = 0.0

        # Keep track of the last time we checked for unused message responses.
        self._last_response_check = 0.0

        # Keep track of last time we requested a board time, and last time we commanded a clock sync.
        self._last_board_time_request = 0.0
        self._last_clock_sync = 0.0
        self._last_clock_sync_bound = float('inf')
        self._last_clock_sync_offset = 0
        # Set in flight. Recorded locally only once the Teensy accepts it. The id can be 0.
        self._pending_clock_offset = 0
        self._clock_offset_msg_id: Optional[int] = None
        self._clock_offset_accepted_at = 0.0

        self._clocks_have_been_synced = False

        self._thread = FeatherThread(
            step_target=self._thread_step, 
            operating_frequency=self.config.teensy_operating_frequency,
        ).start()

        # Check that the Teensy has the correct source hash
        src_hash = self.send_message_await(TeensyCommandOpcode.CMD_GET_SRC_HASH, EMPTY_PAYLOAD)[1]
        if src_hash != self.comms.teensy_source_hash:
            if self.config.use_emulated_teensy:
                _LIB_BIN_EM.unlink(missing_ok=True)
            else:
                _LIB_BIN_REAL.unlink(missing_ok=True)
            raise TeensyError(f"Teensy source hash mismatch: {src_hash} != {self.comms.teensy_source_hash}\n\t"
                "Re-run the program to rebuild the Teensy library.")

        # Discover CAN topology before any host-initiated CAN traffic.
        num_can = self.send_message_await(TeensyCommandOpcode.CMD_GET_NUM_CAN_LINES, EMPTY_PAYLOAD)[1]
        self.comms.num_can_endpoints = int(num_can)
        if self.comms.num_can_endpoints < 1:
            raise TeensyError("Teensy reported 0 CAN lines")
        if self.comms.num_can_endpoints > NUM_CAN_ENDPOINTS_MAX:
            raise TeensyError(f"Teensy reported {self.comms.num_can_endpoints} CAN lines, max: {NUM_CAN_ENDPOINTS_MAX}")
        
        log_json(NOTICE, "teensy_info", {"src_hash_str": src_hash, "num_can_endpoints": self.comms.num_can_endpoints})

        # Reset Teensy actuators/config so reconnects can re-init motors cleanly.
        init_payload = InitializeConfig._STRUCT.pack(
            self.config.server_run_time_nanos,
            self.config.can_read_run_time_nanos,
            self.config.max_can_rate,
            self.config.can_leniency,
        )
        self.send_message(TeensyCommandOpcode.CMD_INITIALIZE, init_payload, block=True)

        self._sync_clocks()

    def _sync_clocks(self) -> None:
        """One-shot Cristian sync of the Teensy clock to the host monotonic clock.

        Always runs on connect, even when ``teensy_correct_clock_drift`` is False.
        Disables Teensy logging for fast responses during the trials.
        """
        self.send_message(TeensyCommandOpcode.CMD_SET_LOGGING, SetLoggingPayload._STRUCT.pack(False), block=True)
        for _ in range(self.config.teensy_clock_sync_trials):
            self.send_message(TeensyCommandOpcode.CMD_GET_BOARD_TIME, EMPTY_PAYLOAD, block=True)
        
        max_sync_err = self.config.teensy_max_clock_sync_error_ns * 1e-9
        if self._last_clock_sync_bound > max_sync_err:
            raise TeensyError(f"Failed to sync clocks: {self._last_clock_sync_bound}s > {max_sync_err}s")
        
        if self.config.enable_teensy_debug_logging:
            self.send_message(TeensyCommandOpcode.CMD_SET_LOGGING, SetLoggingPayload._STRUCT.pack(True), block=True)
        
        self._clocks_have_been_synced = True

    def _recv_upto(self, upto: int) -> bool:
        """Read available socket bytes into ``_result_buffer``. Reads until ``n`` bytes are available in the buffer.

        Returns:
            True if ``n`` bytes are available in the buffer, False otherwise.
        """
        if upto > RESULT_BUFFER_SIZE:
            raise ValueError(f"Requested to read up to {upto} bytes, but buffer size is {RESULT_BUFFER_SIZE}")
        elif self._buf_nbytes > upto:
            raise RuntimeError(f"Buffer has {self._buf_nbytes} bytes, but requested to read up to {upto}")
        elif self._buf_nbytes == upto:
            return True
        
        try:
            self._buf_nbytes += self._socket.recv_into(self._buf_memview[self._buf_nbytes:], upto - self._buf_nbytes)
            return self._buf_nbytes >= upto
        except BlockingIOError:
            return False

    @perftime(prefix="TeensyHandler", sample_rate=_HOT_PATH_SAMPLE_RATE)
    def _thread_step(self) -> None:
        """Process any results from the Teensy, and request any new telemetry updates."""
        # Drain as many complete messages as are already available. One-message-per-tick
        # is too slow when the host is flooding GET_ACTUATOR_TELEMETRY during blocking waits.
        for _ in range(self.config.teensy_max_rx_messages_per_tick):
            if not self._try_process_one_message():
                break
        
        func_time = currtime()
        
        # Continuous drift correction: poll board time and maybe push a new offset.
        # Initial sync is handled separately in ``_sync_clocks``; skip when disabled.
        if (
            self.config.teensy_correct_clock_drift
            and func_time - self._last_board_time_request > 1.0 / self.config.teensy_board_clock_sync_hz
        ):
            self._last_board_time_request = currtime()
            self.send_message(TeensyCommandOpcode.CMD_GET_BOARD_TIME, EMPTY_PAYLOAD, block=False)

        # Latch global telemetry once it is initialized, then poll Teensy for updates.
        if self._global_telemetry is None:
            if self.comms.robot.global_telemetry is not None and self.comms.robot.global_telemetry._initialized:
                self._global_telemetry = self.comms.robot.global_telemetry
            else:
                return

        if func_time - self._last_telemetry_update > 1.0 / self.config.teensy_telemetry_update_frequency:
            self._last_telemetry_update = func_time
            self.comms.send_teensy_message(TeensyCommandOpcode.CMD_GET_ACTUATOR_TELEMETRY, EMPTY_PAYLOAD, block=False)

        dms = self.comms.robot.data_monitor_service
        if dms is not None and dms.candump_writer is not None \
            and func_time - self._last_candump_update > 1.0 / self.config.teensy_candump_update_frequency:
            self._last_candump_update = func_time
            self.comms.send_teensy_message(TeensyCommandOpcode.CMD_GET_CANDUMP, EMPTY_PAYLOAD, block=False)
        
        # Occasionally check for unhandled messages
        msg_interval = self.config.teensy_check_unused_messages_interval
        if func_time - self._last_response_check > msg_interval:
            self._last_response_check = func_time

            # Check for sent messages that haven't been responded to in a while, and responses
            # the caller never collected. Snapshot under the lock; other threads mutate both dicts.
            with self._messages_lock:
                sent_remove_ids = [
                    msg_id for msg_id, sent_time in self._sent_messages.items()
                    if func_time - sent_time > msg_interval
                ]
                for msg_id in sent_remove_ids:
                    del self._sent_messages[msg_id]
                response_remove_ids = [
                    msg_id for msg_id, response in self._responses.items()
                    if func_time - response[2] > msg_interval
                ]
                for msg_id in response_remove_ids:
                    del self._responses[msg_id]
            for msg_id in sent_remove_ids:
                warning(f"Message `{msg_id}` not responded to within timeout ({msg_interval}s), dropping message")

    @perftime(prefix="TeensyHandler", sample_rate=_HOT_PATH_SAMPLE_RATE)
    def _try_process_one_message(self) -> bool:
        """Try to read and handle a single Teensy response. Returns True if one was handled."""
        # Try to parse a full header from the buffer.
        if self._cur_msg_id is None:
            if not self._recv_upto(ClientCommandHeader._STRUCT.size):
                return False
            self._cur_msg_id, self._cur_opcode, self._cur_pyld_size = \
                ClientCommandHeader._STRUCT.unpack_from(self._buffer)

        # Try to parse a full message from the buffer (header already read).
        if not self._recv_upto(HEADER_SIZE + self._cur_pyld_size):
            return False

        # Handle the message and reset the current state.
        payload = bytes(self._buf_memview[HEADER_SIZE:HEADER_SIZE + self._cur_pyld_size])
        self._handle_message(ensure(self._cur_msg_id), self._cur_opcode, payload)
        self._cur_msg_id = None
        self._buf_nbytes = 0
        return True

    @perftime(prefix="TeensyHandler", sample_rate=_HOT_PATH_SAMPLE_RATE)
    def _handle_message(self, msg_id: int, opcode: int, payload: bytes) -> None:
        """Handle a fully-received Teensy response message."""
        # Make sure it is a known message, then remove it from the sent messages dictionary.
        with self._messages_lock:
            sent_time = self._sent_messages.get(msg_id)
        if sent_time is None:
            warning(f"Received message with ID `{msg_id}` (opcode={opcode}) but it was not in sent messages; dropping")
            return

        # Parse the response outside the lock. Clock sync sends another message, which takes the lock itself.
        stored: Optional[Tuple[int, TeensyResponseType, float]] = None
        if opcode in (TeensyCommandOpcode.RESP_ERROR, TeensyCommandOpcode.RESP_ECHO, TeensyCommandOpcode.RESP_SRC_HASH):
            response = payload.decode("utf-8")

            # Manually log the error in cause user doesn't handle it themselves.
            if opcode == TeensyCommandOpcode.RESP_ERROR:
                error(f"Received error response for message `{msg_id}`: {response}")
                if msg_id == self._clock_offset_msg_id:
                    # Rejected, so the next sample can try this correction again.
                    self._clock_offset_msg_id = None
                    self._last_clock_sync_bound = float("inf")

            stored = (opcode, response, currtime())

        elif opcode == TeensyCommandOpcode.RESP_SUCCESS:
            if msg_id == self._clock_offset_msg_id:
                self._last_clock_sync_offset = self._pending_clock_offset
                self._clock_offset_msg_id = None
                self._clock_offset_accepted_at = currtime()
            stored = (TeensyCommandOpcode.RESP_SUCCESS, 1, currtime())

        elif opcode == TeensyCommandOpcode.RESP_BOARD_TIME:
            # Update the clock before storing the response.
            curr_time = currtime()
            board_time_ns = GetBoardTimeResponsePayload._STRUCT.unpack(payload)[0]
            self._maybe_update_clock_offset(curr_time, sent_time, board_time_ns)
            stored = (TeensyCommandOpcode.RESP_BOARD_TIME, board_time_ns, curr_time)

        elif opcode == TeensyCommandOpcode.RESP_NUM_CAN_LINES:
            stored = (
                TeensyCommandOpcode.RESP_NUM_CAN_LINES,
                GetNumCanLinesResponsePayload._STRUCT.unpack(payload)[0],
                currtime(),
            )

        elif opcode == TeensyCommandOpcode.RESP_ACTUATOR_TELEMETRY:
            n_updates = GetTelemetryPayloadHeader._STRUCT.unpack_from(payload)[0]
            if self.comms.robot.global_telemetry is not None:
                self.comms.robot.global_telemetry.update_telemetry(
                    n_updates, payload[GetTelemetryPayloadHeader._STRUCT.size:]
                )
            else:
                error(f"Received actuator telemetry response but global telemetry is not initialized")

        elif opcode == TeensyCommandOpcode.RESP_CANDUMP:
            dms = self.comms.robot.data_monitor_service
            writer = dms.candump_writer if dms is not None else None
            if writer is not None and len(payload) > 0:  # Teensy payload is 0 when no new messages
                writer.write(payload)

        else:
            stored = (TeensyCommandOpcode.RESP_ERROR, f"Unknown response code: `{opcode}`", currtime())
            error(stored)

        with self._messages_lock:
            if stored is not None:
                self._responses[msg_id] = stored
            self._sent_messages.pop(msg_id, None)
        
        # Log the response unless it is already recorded by a dedicated monitor.
        dms = self.comms.robot.data_monitor_service
        if dms is not None and dms.teensy_message_writer is not None and opcode:
            mon_payload = _EMPTY_BYTES if opcode in _UNTRACKED_TEENSY_RX_OPCODES else payload
            dms.teensy_message_writer.write_message(currtime(), False, msg_id, opcode, mon_payload)
    
    def _maybe_update_clock_offset(self, curr_time: float, sent_time: float, board_time_ns: int) -> None:
        """Maybe push a Cristian clock offset to the Teensy from a board-time RTT sample.

        Used for both the initial ``_sync_clocks`` trials (while ``_clocks_have_been_synced``
        is False) and continuous background correction afterward. After the initial sync,
        updates are skipped when ``teensy_correct_clock_drift`` is False.

        Accuracy bound is ±RTT/2. A new offset is applied only when that bound is tighter
        than the previous bound plus assumed drift since the last update
        (``teensy_expected_clock_drift_ns`` per second).

        The local offset updates when the Teensy accepts the set. A board-time sample sent
        before that still shows the old clock and is not applied again.
        """
        # Allow updates during initial sync; block only continuous correction when disabled.
        if self._clocks_have_been_synced and not self.config.teensy_correct_clock_drift:
            return
        if self._clock_offset_msg_id is not None or sent_time < self._clock_offset_accepted_at:
            return

        bound = (curr_time - sent_time) / 2.0
        total_drift = (currtime() - self._last_clock_sync) * self.config.teensy_expected_clock_drift_ns / 1e9
        if bound >= total_drift + self._last_clock_sync_bound:
            return

        self._last_clock_sync_bound = bound
        self._last_clock_sync = currtime()
        new_offset = self._last_clock_sync_offset + int((sent_time + bound) * 1e9) - board_time_ns
        self._pending_clock_offset = new_offset
        # block=False returns the message id. The signature also allows the blocking response tuple.
        self._clock_offset_msg_id = cast(int, self.send_message(
            TeensyCommandOpcode.CMD_SET_TIMESTAMP_OFFSET,
            SetTimestampOffsetPayload._STRUCT.pack(new_offset),
            block=False,
        ))

    @perftime(prefix="TeensyHandler", sample_rate=_HOT_PATH_SAMPLE_RATE)
    def cansend(
        self, bus: int, extended: bool, can_id: int, data: bytes, block: bool = False, timeout: float = 0.1
    ) -> Union[int, Tuple[int, TeensyResponseType, float]]:
        """Send a plain CAN frame on the given Teensy bus.

        Args:
            bus: CAN bus index in ``[0, CommsManager.num_can_endpoints)``.
            extended: Whether the frame uses a 29-bit extended ID (else 11-bit standard).
            can_id: CAN ID of the frame.
            data: Payload bytes (0-8).
            block: Whether to wait for the Teensy's ack/error response. See ``send_message``.
            timeout: Timeout in seconds when ``block`` is True.

        Returns:
            The message ID if ``block`` is False, or the Teensy response tuple if ``block`` is True.
        """
        padded = bytes(data) + bytes(8 - len(data))
        payload = SendCanPayload._STRUCT.pack(bus, bool(extended), len(data), can_id, *padded)
        return self.send_message(TeensyCommandOpcode.CMD_SEND_CAN, payload, block=block, timeout=timeout)

    def send_message(
        self, opcode: int, payload: bytes, block: bool = False, timeout: float = 0.1
    ) -> Union[int, Tuple[int, TeensyResponseType, float]]:
        """Send a message to the Teensy.

        Args:
            opcode: The opcode of the message to send.
            payload: The payload of the message to send.
            block: Whether to block until a response is received. This is the response from the Teensy, not from any
                CAN communication that the Teensy is doing. If False, the return value is the message ID of the message
                sent (to be later retrieved with `get_response()`). Otherwise, the return value is the Teensy response.
            timeout: The timeout in seconds to wait for a response from the Teensy. Will raise an error if no response 
                is received within the timeout.
        
        Returns:
            The message ID if `block` is False, or tuple of (response code, response, timestamp) if `block` is True.
        """
        # Register before sendall so a fast Teensy Success/telemetry reply cannot race the thread
        with self._messages_lock:
            message_id = self._next_message_id
            self._next_message_id += 1
            self._sent_messages[message_id] = currtime()
        self._socket.sendall(ClientCommandHeader._STRUCT.pack(message_id, opcode, len(payload)) + payload)

        # Log the message unless it is already recorded by a dedicated monitor.
        dms = self.comms.robot.data_monitor_service
        if dms is not None and dms.teensy_message_writer is not None and opcode:
            mon_payload = _EMPTY_BYTES if opcode in _UNTRACKED_TEENSY_TX_OPCODES else payload
            dms.teensy_message_writer.write_message(currtime(), True, message_id, opcode, mon_payload)

        if not block:
            return message_id

        # Otherwise, block until a response is received, and return the response.
        response = self.get_response(message_id, timeout)
        if response is None:
            raise TimeoutError(f"Timeout waiting for response to message `{message_id}`")
        elif response[0] == TeensyCommandOpcode.RESP_ERROR:
            raise TeensyError(f"Received error response for message `{message_id}`: {response[1]}")
        return response
    
    def send_message_await(
        self, opcode: int, payload: bytes, timeout: float = 0.1
    ) -> Tuple[int, TeensyResponseType, float]:
        """Send a message to the Teensy and wait for a response.
        
        Args:
            opcode: The opcode of the message to send.
            payload: The payload of the message to send.
            timeout: The timeout in seconds to wait for a response. If <= 0, the function will check and return
                immediately without blocking.
        """
        return_value = self.send_message(opcode, payload, block=True, timeout=timeout)
        assert not isinstance(return_value, int)
        return return_value
        
    def get_response(self, message_id: int, timeout: float = 0.0) -> Optional[Tuple[int, TeensyResponseType, float]]:
        """Get the response for a message from the Teensy.

        NOTE: On a successful call, the response will be removed from the responses dictionary, so further calls
            will return None.

        Args:
            message_id: The ID of the message to get the response for.
            timeout: The timeout in seconds to wait for a response. If <= 0, the function will check and return
                immediately without blocking.
        
        Returns:
            Tuple of (response code, response, timestamp), or None if no response is received within the timeout.
        """
        if timeout > 0 and not self._has_response(message_id):
            exponential_timeout(await_true=lambda: self._has_response(message_id), timeout=timeout)

        with self._messages_lock:
            return self._responses.pop(message_id, None)

    def _has_response(self, message_id: int) -> bool:
        with self._messages_lock:
            return message_id in self._responses
        
    def _abort_impl(self) -> None:
        try:
            self._thread.terminate(block=True)
        finally:
            self._socket.close()


class TeensyError(Exception):
    """An error occurred while communicating with the Teensy."""


_NIC_INTERRUPT_COALESCING_EXPECTED = {
    "rx-usecs": 6,
    "rx-frames": 1,
    "tx-usecs": 32,
    "tx-frames": 1,
}
"Expected NIC interrupt coalescing settings."

_NIC_SET_INTERRUPT_COALESCING_COMMAND = """```
    sudo ip link set {iface} down && \\
    sudo ethtool -C {iface} {param_str} && \\
    sudo ip link set {iface} up
```"""
"""Command to set the NIC interrupt coalescing settings to the optimal values."""


def _check_NIC_interrupt_coalescing(
    interface: Optional[str] = None,
    host: Optional[str] = None,
    expected_params: Dict[str, int] = _NIC_INTERRUPT_COALESCING_EXPECTED,
) -> None:
    """Check NIC coalescing settings and make sure they are optimal, logging a WARNING if not.
    
    NICs are allowed to group incomming ethernet frames together, waiting for either a max wait time or max number of
    frames, before triggering a CPU interrupt to handle them. This is done to reduce the number of interrupts and
    context switches required which in theory reduces CPU usage.

    However for us, we really want to minimize latency, especially when we are trying to sync clocks with the Teensy.
    So, we check to make sure we reduce this latency as the kernel/hardware allows.

    Default params - rx-usecs: 512, rx-frames: 64, tx-usecs: 256, tx-frames: 16
    Minimal params - rx-usecs: 6, rx-frames: 1, tx-usecs: 32, tx-frames: 1

    Changing these params requires sudo, so we don't change them, just warn about them.

    `ethtool -c <interface>` outputs lines like: `rx-usecs: 512`, `rx-frames: 64`, `tx-usecs: 256`, `tx-frames: 16`, ...

    ``interface`` defaults to the NIC used to reach ``host``. That name is ``eno1`` on the Orin and a different
    predictable name on the Thor.
    """
    if get_system_board() != SystemBoard.AGX_ORIN:
        return
    if interface is None:
        if host is None:
            raise RuntimeError("Need a host or interface to check NIC coalescing.")
        interface = detect_ethernet_interface(host)
    if interface == "lo":
        return

    proc = FeatherShellProcess(command=f"ethtool -c {interface}".split(" "), name="check_ethtool_settings").start()
    result_lines = proc.wait().get_result().assert_success().stdout.decode("utf-8").split("\n")

    # Read the actual params from the output.
    params = {}
    for line in result_lines:
        for k in expected_params:
            if line.startswith(k + ":"):
                params[k] = int(line.split(":")[1].strip())
                break
    log_json(NOTICE, "nic_interrupt_coalescing_settings", {
        "interface": interface, "params": params, "expected": expected_params,
    })

    # Check if the actual params are not optimal.
    for k, v in expected_params.items():
        if params.get(k, None) != v:
            fmt_params = ' '.join([f"{k} {v}" for k, v in expected_params.items()])
            warning(f"NIC interrupt coalescing settings are not optimal: `{params}`, expected "
                    f"`{expected_params}`.\n Run the following command to set the optimal settings:\n"
                    f"{_NIC_SET_INTERRUPT_COALESCING_COMMAND.format(iface=interface, param_str=fmt_params)}")
            break
