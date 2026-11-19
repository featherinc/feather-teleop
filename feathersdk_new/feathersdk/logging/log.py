import os
import sys
import threading
import json
from feathersdk.utils.time import timediff, currtime
from feathersdk.utils.feathertypes import (
    TYPE_CHECKING, Final, Optional, Iterable, Any, FrameType, Union, TypeAlias, Dict,
)

from .levels import _LONGEST_LVL_LEN, LoggingLevel, TRACE, DEBUG, INFO, WARNING, ERROR, CRITICAL, NOTICE
from .loggers import ROSLogger

if TYPE_CHECKING:
    from feathersdk.logging.init import _GlobalLoggingState


_FMT_TIME_DIGITS: Final[int] = 6
"""The number of digits to display in the decimal part of the time delta."""

JSON_PREFIX: Final[str] = "__JSON_DATA__"
"""Prefix to use for JSON data in a log message. Anything after this in a message line will be treated as JSON data."""

_Outputs: TypeAlias = Optional[Union[str, Iterable[str]]]
"""Type of the `outputs` argument on the logging functions. See `log()`."""


def _GL_STATE() -> '_GlobalLoggingState':
    """Get the global logging state. Only called in non-critical path."""
    from feathersdk.logging.init import _GL_STATE  # circular import
    return _GL_STATE()


def log(
    level: LoggingLevel, 
    *fmt_args: Any, 
    fmt_sep: str = ' ',
    debug_tags: Optional[Iterable[str]] = None,
    assert_initialized: bool = False,
    outputs: _Outputs = None,
    _logged_from: Optional[str] = None,
    _call_chain: int = 2,
) -> bool:
    """Log a message with the specified level.

    Args:
        level: The level to log at.
        *fmt_args: The arguments to format the message with. Formats similar to print().
        fmt_sep: The separator to use between the arguments. Defaults to a single space.
        debug_tags: Tags to attach to this log message. If we are currently filtering messages by tags, then these
            are used to determine if the message should be logged. See `set_logging_only_tags()` for more information.
        assert_initialized: If True, will raise a RuntimeError if logging has not been initialized.
        outputs: Name(s) of the loggers to send this message to (EG: "console", "file", "ros"). Can be a single name
            or an iterable of names. None (default) sends it to every enabled logger. This only narrows the outputs:
            the configured logging levels and debug tags still apply. Names that don't match an enabled logger are
            ignored.
        _logged_from: Allows you to spoof logging from a different location. Used for testing.
        _call_chain: Do not set! Used internally.

    Returns:
        True if the message was logged, False otherwise.
    """
    __GL_STATE = _GL_STATE()
    if len(__GL_STATE.loggers) == 0:
        if __GL_STATE.initialized:
            return False  # Logging has been initialized, but all loggers are disabled
        if assert_initialized:
            raise RuntimeError("Logging has not been initialized. Call init_logging() first.")
        return False
    
    stack_loc = _logged_from if _logged_from is not None else _get_message_stack_loc(_call_chain)

    # Find/create node, then do early return instead of checking throttling if we are for sure not logging at this level
    node = __GL_STATE.find_or_create_node(stack_loc)
    assert node.config.console_level is not None and node.config.file_level is not None
    if node.config.console_level > level and node.config.file_level > level:
        return False
    
    # Get all the loggers that are allowed to log this message. Return early if we don't have any.
    filtered_loggers = node.get_filtered_loggers(level, debug_tags, __GL_STATE.loggers.copy())
    if outputs is not None:
        outputs = (outputs,) if isinstance(outputs, str) else set(outputs)
        filtered_loggers = {k: v for k, v in filtered_loggers.items() if k in outputs}
    if len(filtered_loggers) == 0:
        return False

    # Construct and log the message
    time_fmt = fmt_time_delta(timediff(__GL_STATE.logging_init_time))
    log_loc = f"{os.getpid()}:{threading.current_thread().name}:{stack_loc}"
    fmt_msg = fmt_sep.join(list(map(str, fmt_args))).replace("\n", "\n\t")
    log_msg_nl = "\n\t" if __GL_STATE.config.log_message_multiline else " "
    msg = f"[{time_fmt}] {level.name:>{_LONGEST_LVL_LEN}} ({log_loc}):{log_msg_nl}{fmt_msg}\n"
    for logger in filtered_loggers.values():
        if isinstance(logger, ROSLogger):  # ROSLogger needs to know the level to log at to convert to ros's logger()
            logger.log(fmt_msg, level=level)
        else:
            logger.log(msg)

    return True


def trace(
    *args: Any, debug_tags: Optional[Iterable[str]] = None, outputs: _Outputs = None, _logged_from: Optional[str] = None
) -> bool:
    """Log a trace message. Args work similarly to print(). Returns True if message was logged, False otherwise."""
    return log(TRACE, *args, debug_tags=debug_tags, outputs=outputs, _logged_from=_logged_from, _call_chain=3)

def debug(
    *args: Any, debug_tags: Optional[Iterable[str]] = None, outputs: _Outputs = None, _logged_from: Optional[str] = None
) -> bool:
    """Log a debug message. Args work similarly to print(). Returns True if message was logged, False otherwise."""
    return log(DEBUG, *args, debug_tags=debug_tags, outputs=outputs, _logged_from=_logged_from, _call_chain=3)

def info(
    *args: Any, debug_tags: Optional[Iterable[str]] = None, outputs: _Outputs = None, _logged_from: Optional[str] = None
) -> bool:
    """Log an info message. Args work similarly to print(). Returns True if message was logged, False otherwise."""
    return log(INFO, *args, debug_tags=debug_tags, outputs=outputs, _logged_from=_logged_from, _call_chain=3)

def warning(
    *args: Any, debug_tags: Optional[Iterable[str]] = None, outputs: _Outputs = None, _logged_from: Optional[str] = None
) -> bool:
    """Log a warning message. Args work similarly to print(). Returns True if message was logged, False otherwise."""
    return log(WARNING, *args, debug_tags=debug_tags, outputs=outputs, _logged_from=_logged_from, _call_chain=3)

def error(
    *args: Any, debug_tags: Optional[Iterable[str]] = None, outputs: _Outputs = None, _logged_from: Optional[str] = None
) -> bool:
    """Log an error message. Args work similarly to print(). Returns True if message was logged, False otherwise."""
    return log(ERROR, *args, debug_tags=debug_tags, outputs=outputs, _logged_from=_logged_from, _call_chain=3)

def critical(
    *args: Any, debug_tags: Optional[Iterable[str]] = None, outputs: _Outputs = None, _logged_from: Optional[str] = None
) -> bool:
    """Log a critical message. Args work similarly to print(). Returns True if message was logged, False otherwise."""
    return log(CRITICAL, *args, debug_tags=debug_tags, outputs=outputs, _logged_from=_logged_from, _call_chain=3)

def notice(
    *args: Any, debug_tags: Optional[Iterable[str]] = None, outputs: _Outputs = None, _logged_from: Optional[str] = None
) -> bool:
    """Log a notice message. This is a high-importance informational message that is not a warning or error.
       Args work similarly to print(). Returns True if message was logged, False otherwise."""
    return log(NOTICE, *args, debug_tags=debug_tags, outputs=outputs, _logged_from=_logged_from, _call_chain=3)

def log_json(
    level: LoggingLevel, 
    info_name: str, 
    data: Dict[Any, Any], 
    debug_tags: Optional[Iterable[str]] = None, 
    outputs: _Outputs = "file",
    _logged_from: Optional[str] = None,
    _call_chain: int = 3,
) -> bool:
    """Log a message with structured JSON data. Only logs to the file by default, see `log()` for `outputs`."""
    msg = "%s{\"%s\": %s}" % (JSON_PREFIX, info_name, json.dumps(data))
    return log(level, msg, debug_tags=debug_tags, outputs=outputs, _logged_from=_logged_from, _call_chain=_call_chain)


EVENT_KEY = "event"
"""The JSON key of an event line: the one key `log_parsing` collects into a list instead of a single value."""


def _plain(value: Any) -> Any:
    """`value` as JSON can hold it: numpy arrays and scalars as lists and floats, dicts and sequences recursed, keys
    as strings (a dict keyed by motor objects is keyed by their names)."""
    if hasattr(value, "tolist"):  # numpy arrays and scalars
        return value.tolist()
    if isinstance(value, dict):
        return {(k if isinstance(k, str) else getattr(k, "name", str(k))): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


def log_event(kind: str, system: str, **fields: Any) -> bool:
    """Record what a system was asked to do, or did for safety, as one structured line of the run's log (file only):
    `{"event": {"t": currtime(), "kind": kind, "system": system, ...fields}}`. `t` is the clock the telemetry and
    Teensy recordings use, so an event lines up with them. For goals, trajectories, stops and faults, once per call:
    never for a per-cycle stream (the data monitor records every command on the wire). `log_parsing` collects the
    events of a run in order."""
    data = {"t": currtime(), "kind": kind, "system": system}
    data.update(_plain(fields))
    return log_json(INFO, EVENT_KEY, data, _call_chain=4)

if sys.version_info >= (3, 11):
    def _get_frame_local_name(frame: FrameType) -> str:
        """Post-3.11 has co_qualname, use that to get the name of the frame"""
        return frame.f_code.co_qualname
else:
    def _get_frame_local_name(frame: FrameType) -> str:
        """Pre-3.11 doesn't have co_qualname, so default to just the function name"""
        return frame.f_code.co_name


def _get_message_stack_loc(depth: int) -> str:
    """Returns the qualified name of the location `depth` steps upwards on the stack"""
    try:
        frame = sys._getframe(depth)
        module_name = frame.f_globals.get("__name__", "unknown_module")
        local_name = _get_frame_local_name(frame)
        return module_name if local_name == "<module>" else f"{module_name}.{local_name}"
    except Exception:
        return "error_getting_stack_loc"


def fmt_time_delta(seconds: float, decimal_digits: int = _FMT_TIME_DIGITS) -> str:
    """Format a time delta in a human-readable format. Returns a string like '00_00:00:00.000000000'. 
       `decimal_digits` will truncate the decimal part to the given number of digits."""
    neg = seconds < 0
    total_s, ns = divmod(int(round(abs(seconds) * 1_000_000_000)), 1_000_000_000)
    total_m, s = divmod(total_s, 60)
    total_h, m = divmod(total_m, 60)
    d, h = divmod(total_h, 24)

    sign = "-" if neg else ""
    decimal_time = str(ns // 10**(9 - decimal_digits)).zfill(decimal_digits)
    return f"{sign}{d:02d}_{h:02d}:{m:02d}:{s:02d}.{decimal_time}"
