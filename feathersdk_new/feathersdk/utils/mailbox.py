"""Latest-wins mailbox: a one-value channel between threads where the newest value wins, and `OwnedSlot`, a mailbox
only its current owner may write."""
import threading

from .feathertypes import Generic, Optional, Sequence, Tuple, TypeVar, T

TOwner = TypeVar("TOwner")


class NotOwnerError(RuntimeError):
    """Raised when something other than the current owner writes to an `OwnedSlot`."""


class LatestWins(Generic[T]):
    """A one-value channel between threads: `write()` replaces the value and `read()` returns the newest one.

    Neither side waits on a lock. `write()` stores one reference to `(version, value)` and `read()` loads it, each an
    atomic step in CPython, so a reader always gets one whole value, as long as values are never modified after they're
    written (e.g. a `NamedTuple` of read-only arrays). A value overwritten before anyone reads it is dropped, which is
    what a control loop wants. Use a queue when every item must arrive.
    """

    __slots__ = ("_entry",)

    def __init__(self) -> None:
        self._entry: Tuple[int, Optional[T]] = (0, None)

    def write(self, value: T) -> int:
        """Replace the value and return its version (1 for the first write, then 2, 3, ...). One writer at a time: two
        threads writing at once could get the same version."""
        version = self._entry[0] + 1
        self._entry = (version, value)
        return version

    def read(self) -> Optional[T]:
        """The newest value, or None before the first write."""
        return self._entry[1]

    def read_versioned(self) -> Tuple[int, Optional[T]]:
        """The newest value and its version, `(0, None)` before the first write. A reader that keeps the version can
        tell whether the value is new since its last read."""
        return self._entry


class OwnedSlot(Generic[TOwner, T]):
    """A `LatestWins` mailbox only its owner may write; `switch_owner()` changes the owner (ros2_control's one
    controller per command interface).

    Writes and switches share a short lock so checking the owner and writing are one step; reads never wait. After a
    switch, the previous owner's last value stays until the new owner writes.
    """

    __slots__ = ("_owner", "_box", "_lock")

    def __init__(self, owner: TOwner) -> None:
        self._owner = owner
        self._box: LatestWins[T] = LatestWins()
        self._lock = threading.Lock()

    @property
    def owner(self) -> TOwner:
        """The current owner."""
        return self._owner

    def switch_owner(self, owner: TOwner) -> TOwner:
        """Make `owner` the only writer and return the previous owner."""
        with self._lock:
            previous, self._owner = self._owner, owner
            return previous

    def write(self, owner: TOwner, value: T) -> int:
        """Write `value` as `owner` and return its version. Raises NotOwnerError if `owner` is not the current owner."""
        with self._lock:
            if owner != self._owner:
                raise NotOwnerError(f"{owner!r} can't write: {self._owner!r} owns this slot")
            return self._box.write(value)

    def read(self) -> Optional[T]:
        """The newest value, or None before the first write."""
        return self._box.read()

    def read_versioned(self) -> Tuple[int, Optional[T]]:
        """The newest value and its version, `(0, None)` before the first write."""
        return self._box.read_versioned()

    @staticmethod
    def write_together(owner: TOwner, writes: Sequence[Tuple['OwnedSlot[TOwner, T]', T]]) -> None:
        """Write each value to its slot as `owner`, all or none. Every slot's lock is held while all owners are checked
        and all values written, so an owner switch can't land in between. Raises NotOwnerError, writing nothing, unless
        `owner` owns every slot."""
        slots = sorted({id(slot): slot for slot, _ in writes}.values(), key=id)  # one lock order: no deadlocks
        for slot in slots:
            slot._lock.acquire()
        try:
            for slot, _ in writes:
                if slot._owner != owner:
                    raise NotOwnerError(f"{owner!r} can't write: {slot._owner!r} owns one of the slots")
            for slot, value in writes:
                slot._box.write(value)
        finally:
            for slot in slots:
                slot._lock.release()
