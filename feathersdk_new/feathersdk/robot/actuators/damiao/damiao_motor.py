import struct
from contextlib import suppress
from feathersdk.robot.actuators.motor import Motor, MotorConfig
from feathersdk.utils import exponential_timeout, currtime
from feathersdk.utils.feathertypes import TYPE_CHECKING, Any, NamedTuple, Optional, Generic, TypeVar, TNum, ensure
from feathersdk.configs import PyDField
from feathersdk.quill_defs.python_qdefs.damiao_qdef import (
    DAMIAO_PARAMS, DamiaoCommandPayload, DamiaoControlMode, DamiaoMITControlPayload, DamiaoMotorInitPayload,
    DamiaoParam, DamiaoReadParamPayload, DamiaoStatus, DamiaoWriteParamPayload, _DamiaoParameters, DAMIAO_KD_MAX,
    DAMIAO_KP_MAX, DAMIAO_MASTER_ID_OFFSET,
)
from feathersdk.quill_defs.python_qdefs.data_type_qdef import DataType
from feathersdk.quill_defs.python_qdefs.teensy_commands_qdef import TeensyCommandOpcode

if TYPE_CHECKING:
    from feathersdk.robot.actuator_systems.actuator_system import ActuatorSystem, ActuatorSystemConfig


DAMIAO_TIMEOUT_UNIT = 50e-6
"""Seconds per unit of the Damiao `TIMEOUT` register."""


_FEEDBACK_READ_ATTEMPTS = 3
"""`read_feedback()` rereads when a new frame lands mid-read. Frames arrive tens of milliseconds apart and a read takes
microseconds, so a second attempt almost always succeeds."""


class DamiaoFeedback(NamedTuple):
    """One feedback frame from a Damiao motor. Every value came from the same frame."""
    position: float
    """Output position in radians."""
    velocity: float
    """Output velocity in radians per second."""
    torque: float
    """Output torque in Nm."""
    status: DamiaoStatus
    """Motor status."""
    temperature_mos: int
    """Driver MOSFET temperature in degrees Celsius."""
    temperature_rotor: int
    """Rotor temperature in degrees Celsius."""
    timestamp: float
    """When the Teensy received the frame, in seconds (`FEEDBACK_TS`)."""


class DamiaoMotorConfig(MotorConfig):
    """The config for a Damiao motor.

    The motor's encoding ranges (PMAX/VMAX/TMAX) are not configured here: they are read from the motor's registers when
    it is found. The limits below are safety limits only, and the firmware enforces whichever is tighter.
    """
    requires_calibration: bool = PyDField(default=False)
    """Damiao motors keep their zero across power cycles, so the bare motor needs no calibration."""
    max_torque: float = PyDField(gt=0.0, le=60.0, default=3.0)
    """Safety limit: largest absolute torque the firmware will command (Nm). Default is the DM4310 rated torque."""
    max_kp: float = PyDField(ge=0.0, le=DAMIAO_KP_MAX, default=DAMIAO_KP_MAX)
    """Safety limit: largest kp the firmware will command."""
    max_kd: float = PyDField(ge=0.0, le=DAMIAO_KD_MAX, default=DAMIAO_KD_MAX)
    """Safety limit: largest kd the firmware will command."""
    teensy_telemetry_max_size: int = PyDField(gt=0, le=0xFFFF, default=300)
    """Cap sent to the Teensy for each parameter ring. Actual size might be smaller."""
    message_block_timeout: float = PyDField(gt=0.0, le=10.0, default=0.3)
    """The timeout in seconds to wait for a telemetry update when blocking."""
    message_send_attempts: int = PyDField(gt=0, default=2)
    """The number of attempts to send a message to the motor when awaiting a response."""


TDamiaoMotorConfig = TypeVar("TDamiaoMotorConfig", bound=DamiaoMotorConfig)
"""Type variable for a Damiao motor configuration, so mechanisms built on one motor can extend its config."""


class DamiaoMotor(
    Motor[TDamiaoMotorConfig, _DamiaoParameters], Generic[TDamiaoMotorConfig]
):
    """A Damiao motor driven in MIT mode.

    The motor only reports feedback in reply to a command, and disables itself if no command arrives within its
    `TIMEOUT` watchdog (see `watchdog_timeout`). Anything holding it enabled must keep sending commands.
    """

    PARAMS = DAMIAO_PARAMS
    """The telemetry and read_param() parameters for a Damiao motor."""

    def __init__(self, config: TDamiaoMotorConfig, system: 'ActuatorSystem[ActuatorSystemConfig, Any]'):
        super().__init__(config, system)

        # Scaling (pmax/vmax/tmax) is sent as 0: the firmware learns it from the motor's registers.
        pyld = DamiaoMotorInitPayload._STRUCT.pack(
            self.id, False, 0.0, 0.0, 0.0, self.config.max_torque, self.config.max_kp, self.config.max_kd,
            *self.telemetry.telem_inds, self.config.teensy_telemetry_max_size,
        )
        self.comms.send_teensy_message(TeensyCommandOpcode.CMD_DAMIAO_INITIALIZE_MOTOR, pyld, block=True, timeout=1)

    def _on_endpoint_found(self) -> None:
        """Read the registers the firmware and SDK depend on, and check the motor is set up the way they expect."""
        master_id = self.read_param_await(DAMIAO_PARAMS.MASTER_ID)
        if master_id != self.id + DAMIAO_MASTER_ID_OFFSET:
            raise RuntimeError(f"[{self.config.name}] MASTER_ID is 0x{int(master_id):X}, expected "
                               f"0x{self.id + DAMIAO_MASTER_ID_OFFSET:X}. Replies would not be routed to this motor.")

        ctrl_mode = self.read_param_await(DAMIAO_PARAMS.CTRL_MODE)
        if ctrl_mode != DamiaoControlMode.MIT:
            raise RuntimeError(f"[{self.config.name}] Motor is in control mode {int(ctrl_mode)}, only MIT is supported")

        for param in (DAMIAO_PARAMS.PMAX, DAMIAO_PARAMS.VMAX, DAMIAO_PARAMS.TMAX, DAMIAO_PARAMS.TIMEOUT):
            self.read_param(param)

    def _calibrate(self, load_calibration: Optional[bool] = None, only_load: bool = False, home: bool = True) -> None:
        """Does nothing: the bare motor keeps its zero across power cycles. Mechanisms built on it may override."""
        if only_load and load_calibration is False:
            raise ValueError(f"[{self.config.name}] Cannot only load calibration state without allowing load")

    def _send(self, opcode: TeensyCommandOpcode, payload: bytes, await_param: Optional[DamiaoParam[TNum]] = None) -> None:
        """Send a message to the motor, and if `await_param` is given, wait for a telemetry update of it.

        Will attempt to send the message `self.config.message_send_attempts` times.
        """
        if await_param is None:
            self.comms.send_teensy_message(opcode, payload)
            return

        # Compare against the param's previous Teensy-stamped value, not the host clock: the two clocks are only synced to
        # within a few milliseconds, which could make a fresh reply look old (or an old one look fresh).
        start_stamp = self.telemetry.get_latest_timestamp(await_param, default=0.0)

        def _done() -> bool:
            return self.telemetry.get_latest_timestamp(await_param, default=start_stamp) > start_stamp

        for _attempt in range(self.config.message_send_attempts):
            self.comms.send_teensy_message(opcode, payload)
            try:
                exponential_timeout(self.config.message_block_timeout, await_true=_done, start_delay=0.001)
                return
            except TimeoutError:
                continue

        raise TimeoutError(f"[{self.config.name}] No reply to {opcode.name} after "
                           f"{self.config.message_send_attempts} attempts")

    def _send_command(self, opcode: TeensyCommandOpcode, block: bool) -> None:
        """Send a command that only addresses the motor. Every such command is answered with a feedback frame."""
        payload = DamiaoCommandPayload._STRUCT.pack(self.id, self.config.endpoint)
        self._send(opcode, payload, DAMIAO_PARAMS.FB_STATUS if block else None)

    def enable(self, block: bool = False, **kwargs: Any) -> None:
        """Enable the motor. Clear any fault first with `clear_error()`, or the motor stays disabled."""
        self._send_command(TeensyCommandOpcode.CMD_DAMIAO_ENABLE, block)

    def disable(self, block: bool = False, **kwargs: Any) -> None:
        """Disable the motor."""
        self._send_command(TeensyCommandOpcode.CMD_DAMIAO_DISABLE, block)

    def clear_error(self, block: bool = False) -> None:
        """Clear the motor's fault, e.g. after its communication watchdog tripped."""
        self._send_command(TeensyCommandOpcode.CMD_DAMIAO_CLEAR_ERROR, block)

    def save_zero(self, block: bool = False) -> None:
        """Save the current position as the motor's zero. Persists across power cycles."""
        self._save_zero(block)

    def _save_zero(self, block: bool = False) -> None:
        """Implementation of `save_zero`, kept callable for mechanisms that block the public method."""
        self._send_command(TeensyCommandOpcode.CMD_DAMIAO_SAVE_ZERO, block)

    def mit_command(
        self, position: float = 0.0, velocity: float = 0.0, kp: float = 0.0, kd: float = 0.0, torque: float = 0.0,
        clamp: bool = False,
    ) -> None:
        """Send an MIT-mode command: torque = kp * (position - pos) + kd * (velocity - vel) + torque.

        Args:
            position: The target position in radians.
            velocity: The target velocity in radians per second.
            kp: The position gain.
            kd: The velocity gain.
            torque: The feedforward torque in Newton-meters.
            clamp: If True, clamp out-of-range values to the limits. Otherwise the firmware rejects the command.
        """
        self._mit_command(position, velocity, kp, kd, torque, clamp)

    def _mit_command(
        self, position: float = 0.0, velocity: float = 0.0, kp: float = 0.0, kd: float = 0.0, torque: float = 0.0,
        clamp: bool = False,
    ) -> None:
        """Implementation of `mit_command`, kept callable for mechanisms that block the public method."""
        payload = DamiaoMITControlPayload._STRUCT.pack(
            self.id, self.config.endpoint, position, velocity, kp, kd, torque, clamp
        )
        self._send(TeensyCommandOpcode.CMD_DAMIAO_MIT_CONTROL, payload)

    def read_param(self, param: DamiaoParam[TNum], block: bool = True) -> Optional[TNum]:
        """Send a request to read a register from the motor.

        Feedback values (position, velocity, torque, status, temperatures) are not registers. They update on every
        command reply; use `telemetry.get_latest_value()` for them.

        Args:
            param: The register parameter to read.
            block: Whether to block until we get a response from the motor.
        """
        if not param.readable:
            raise ValueError(f"[{self.config.name}] {param.param_name} is not a register; read it from telemetry")

        payload = DamiaoReadParamPayload._STRUCT.pack(self.id, self.config.endpoint, param.pcode)
        self._send(TeensyCommandOpcode.CMD_DAMIAO_READ_PARAM, payload, param if block else None)
        return self.telemetry[param, -1] if block else None

    def read_param_await(self, param: DamiaoParam[TNum]) -> TNum:
        """Read a register from the motor and wait for a response."""
        return ensure(self.read_param(param, block=True))

    def write_param(self, param: DamiaoParam[TNum], value: TNum, block: bool = True) -> None:
        """Write a writable register (`MASTER_ID`, `ESC_ID`). Takes effect at once; lost at power off unless saved.

        Args:
            param: The register to write. Must be writable.
            value: The value to write.
            block: Wait for the motor's acknowledgement, which echoes the value. Don't block when changing `ESC_ID`:
                the motor answers from its new id at once, so the acknowledgement lands on the new id's slot.
        """
        if not param.writable:
            raise ValueError(f"[{self.config.name}] {param.param_name!r} is not a writable register")
        if param.dtype == DataType.F32:
            raw = struct.unpack("<I", struct.pack("<f", float(value)))[0]
        else:
            raw = int(value)
            if not 0 <= raw <= 0xFFFFFFFF:
                raise ValueError(f"[{self.config.name}] {param.param_name!r} value {value} does not fit in 32 bits")
        payload = DamiaoWriteParamPayload._STRUCT.pack(self.id, self.config.endpoint, param.pcode, raw)
        self._send(TeensyCommandOpcode.CMD_DAMIAO_WRITE_PARAM, payload, param if block else None)

    def save_params(self) -> None:
        """Save the motor's registers to flash so they survive a power cycle. Disable the motor first.

        The motor sends no acknowledgement. Read the registers back after a power cycle to confirm.
        """
        self._send_command(TeensyCommandOpcode.CMD_DAMIAO_SAVE_PARAMS, block=False)

    def read_feedback(self) -> DamiaoFeedback:
        """Read position, velocity, torque, status and temperatures from one feedback frame.

        The Teensy writes `FEEDBACK_TS` after a frame's values, so an unchanged timestamp across the read means every
        value is from that frame. Prefer this over the separate getters when using values together.

        Raises:
            ValueError: The motor has sent no scaled feedback yet (its scaling registers are not read).
            RuntimeError: The feedback changed during every attempt to read it (telemetry is broken).
        """
        telemetry, params = self.telemetry, DAMIAO_PARAMS
        try:
            for _ in range(_FEEDBACK_READ_ATTEMPTS):
                stamp = telemetry[params.FEEDBACK_TS, -1]
                feedback = DamiaoFeedback(
                    position=float(telemetry[params.FB_POSITION, -1]),
                    velocity=float(telemetry[params.FB_VELOCITY, -1]),
                    torque=float(telemetry[params.FB_TORQUE, -1]),
                    status=DamiaoStatus(int(telemetry[params.FB_STATUS, -1])),
                    temperature_mos=int(telemetry[params.FB_TEMPERATURE_MOS, -1]),
                    temperature_rotor=int(telemetry[params.FB_TEMPERATURE_ROTOR, -1]),
                    timestamp=float(stamp),
                )
                if telemetry[params.FEEDBACK_TS, -1] == stamp:
                    return feedback
        except IndexError:  # an empty telemetry ring
            raise ValueError(f"[{self.config.name}] No feedback yet; is the motor found and initialized?") from None
        raise RuntimeError(f"[{self.config.name}] Feedback changed during each of {_FEEDBACK_READ_ATTEMPTS} reads")

    def get_status(self) -> DamiaoStatus:
        """Get the motor status from the latest feedback frame."""
        return DamiaoStatus(int(self.telemetry.get_latest_value(DAMIAO_PARAMS.FB_STATUS)))

    def get_position(self) -> float:
        """Get the position in radians from the latest feedback frame."""
        return float(self.telemetry.get_latest_value(DAMIAO_PARAMS.FB_POSITION))

    def get_velocity(self) -> float:
        """Get the velocity in radians per second from the latest feedback frame."""
        return float(self.telemetry.get_latest_value(DAMIAO_PARAMS.FB_VELOCITY))

    def get_torque(self) -> float:
        """Get the torque in Newton-meters from the latest feedback frame."""
        return float(self.telemetry.get_latest_value(DAMIAO_PARAMS.FB_TORQUE))

    @property
    def watchdog_timeout(self) -> float:
        """Seconds without a command before the motor disables itself, or 0 if its watchdog is off."""
        return float(self.telemetry.get_latest_value(DAMIAO_PARAMS.TIMEOUT)) * DAMIAO_TIMEOUT_UNIT

    def _probe_endpoint(self, endpoint: int, timeout: float = 0.25) -> bool:
        """Probe the CAN bus index for this Damiao motor by reading its ID register.

        Only accepts a reply that the firmware recorded as arriving on `endpoint`. Otherwise a slow reply to the probe
        of the previous bus would be credited to this one.
        """
        old_timeout, old_send_attempts = self.config.message_block_timeout, self.config.message_send_attempts
        try:
            object.__setattr__(self.config, "message_block_timeout", 0.1)
            object.__setattr__(self.config, "message_send_attempts", 2)
            self.config.set_endpoint(endpoint)

            def _found() -> bool:
                with suppress(TimeoutError):
                    self.read_param(DAMIAO_PARAMS.ESC_ID)
                    return int(self.telemetry.get_latest_value(DAMIAO_PARAMS.BUS)) == endpoint
                return False

            with suppress(TimeoutError):
                exponential_timeout(timeout, await_true=_found)
                return True

            self.config.clear_endpoint()
            return False
        finally:
            object.__setattr__(self.config, "message_block_timeout", old_timeout)
            object.__setattr__(self.config, "message_send_attempts", old_send_attempts)
