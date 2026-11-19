# The headset set on a table facing the operator, read over adb from the oculus_reader app's log.
#
# python -m feather_teleop.headsets.table                  # print what the headset sends
# python -m feather_teleop.headsets.table --to <robot>     # stream it over UDP (unauthenticated) to TableLink
import argparse
import json
import socket
import subprocess
import threading
import time
from typing import Optional

import numpy as np

from feather_teleop import config
from feather_teleop.headsets import Reading

APP = "com.rail.oculus.teleop"   # from rail-berkeley/oculus_reader; logs every frame
LOG_TAG = "wE9ryARX"   # the tag the app logs under
FLAGS = {"r": ("A", "B", "RThU", "RJ", "RG", "RTr"), "l": ("X", "Y", "LThU", "LJ", "LG", "LTr")}   # listed while held
# Power manager broadcasts: behave as if worn, and go back to the real wear sensor.
STAY_AWAKE = "com.oculus.vrpowermanager.prox_close"
USE_WEAR_SENSOR = "com.oculus.vrpowermanager.automation_disable"
SAME_POSE = 1e-6        # both controllers this close to the same pose means neither is tracked
FRESH = 0.5             # s, an older sample counts as no data
PRINT_EVERY = 0.25      # s between printed lines
SEND_RATE = 100.0       # Hz
REPORT_EVERY = 5.0      # s between the lines that print the send rate
RECEIVE_TIMEOUT = 0.5   # s the receiver waits for a datagram before checking whether it was closed
MAX_DATAGRAM = 65536    # bytes, the largest UDP datagram


def parse(line: str) -> Optional[tuple]:
    # Parse one log line into (poses, buttons), or None if it is not one of the app's lines.
    prefix = LOG_TAG + ": "
    if prefix not in line or "&" not in line:
        return None
    poses_text, buttons_text = line.split(prefix, 1)[1].split("&", 1)
    return _parse_poses(poses_text), _parse_buttons(buttons_text)


def _parse_poses(text: str) -> dict:
    # The controllers' 4x4 poses by side, or none if the headset does not see them.
    poses = {}
    for part in text.split("|"):
        side, _, numbers = part.partition(":")
        side, values = side.strip(), numbers.split()
        if side in ("l", "r") and len(values) == 16:
            poses[side] = np.array(values, dtype=float).reshape(4, 4)
    if len(poses) == 2 and np.allclose(poses["l"], poses["r"], atol=SAME_POSE):
        return {}   # untracked controllers both report one pose; with none, everything downstream holds
    return poses


def _parse_buttons(text: str) -> dict:
    # The buttons by name: True or False, or a tuple for an analog input.
    items = [item.strip() for item in text.split(",")]
    buttons = {}
    for side, connected in (("r", "R"), ("l", "L")):
        if connected in items:   # the app lists "R" / "L" while that controller is connected
            buttons.update({name: name in items for name in FLAGS[side]})
    for item in items:
        name, *values = item.split(" ")
        if values:   # e.g. "rightTrig 0.35" or "rightJS 0.1 -0.2"
            buttons[name] = tuple(float(value) for value in values)
    return buttons


class _Newest:
    # The newest sample, written by a receiving thread and read by the loop.

    def __init__(self):
        self._lock = threading.Lock()
        self._poses, self._buttons = {}, {}
        self._taken: Optional[float] = None   # on this computer's monotonic clock

    def put(self, poses: dict, buttons: dict, taken: float) -> None:
        with self._lock:
            self._poses, self._buttons, self._taken = poses, buttons, taken

    def latest(self) -> Reading:
        # Return the newest sample; its age is infinite before the first.
        with self._lock:
            age = float("inf") if self._taken is None else time.monotonic() - self._taken
            return Reading(self._poses, self._buttons, age)


class TableHeadset(_Newest):
    # Starts the app on the headset plugged in here, and keeps the newest sample from its log.
    #
    # Args:
    #     serial: The headset's adb serial, needed when more than one device is connected.
    #
    # Raises:
    #     RuntimeError: adb is not installed or does not see the headset.

    def __init__(self, serial: Optional[str] = None):
        super().__init__()
        self._adb = ["adb"] + (["-s", serial] if serial else [])
        self._check_connected()
        self._shell("am", "broadcast", "-a", STAY_AWAKE)   # nobody wears it, so its wear sensor would sleep it
        self._shell("input", "keyevent", "KEYCODE_WAKEUP")
        self._shell("am", "start", "-n", f"{APP}/{APP}.MainActivity")
        self._log = subprocess.Popen(self._adb + ["logcat", "-T", "1", "-s", LOG_TAG], stdout=subprocess.PIPE, text=True, errors="ignore")
        threading.Thread(target=self._follow_log, daemon=True).start()

    def _check_connected(self) -> None:
        try:
            state = subprocess.run(self._adb + ["get-state"], capture_output=True, text=True).stdout.strip()
            devices = subprocess.run(["adb", "devices"], capture_output=True, text=True).stdout.strip()
        except FileNotFoundError:
            raise RuntimeError("adb is not installed (Android platform tools)") from None
        if state != "device":
            raise RuntimeError(f"adb does not see the headset.\n  1. Switch the headset on and plug its USB-C cable into this computer.\n  2. Put it on once and accept \"Allow USB debugging\".\n  3. `adb devices` must then list it as `device`.\n`adb devices` says:\n{devices}")

    def _shell(self, *command: str) -> None:
        subprocess.run(self._adb + ["shell", *command], check=True, capture_output=True)

    def _follow_log(self) -> None:
        for line in self._log.stdout:
            parsed = parse(line)
            if parsed:
                self.put(*parsed, time.monotonic())

    def close(self) -> None:
        self._log.terminate()
        # Not _shell(): closing must not raise if the headset is already unplugged.
        subprocess.run(self._adb + ["shell", "am", "broadcast", "-a", USE_WEAR_SENSOR], capture_output=True)


def encode(reading: Reading) -> bytes:
    # Pack one sample into a JSON datagram.
    poses = {side: np.asarray(pose).ravel().tolist() for side, pose in reading.poses.items()}
    return json.dumps({"poses": poses, "buttons": reading.buttons, "age": reading.age}).encode()


def decode(data: bytes) -> Reading:
    # Unpack a datagram made by `encode()`.
    message = json.loads(data)
    poses = {side: np.array(values, float).reshape(4, 4) for side, values in message["poses"].items()}
    # JSON turns the analog values' tuples into lists.
    buttons = {name: tuple(value) if isinstance(value, list) else value for name, value in message["buttons"].items()}
    return Reading(poses, buttons, float(message["age"]))


class TableLink(_Newest):
    # Receives the laptop's stream on the robot and keeps the newest sample.
    #
    # Args:
    #     port: The UDP port to listen on.
    #     address: The local address to listen on; every interface by default.

    def __init__(self, port: int, address: str = "0.0.0.0"):
        super().__init__()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((address, port))
        self._socket.settimeout(RECEIVE_TIMEOUT)
        self._open = True
        threading.Thread(target=self._listen, daemon=True).start()

    def _listen(self) -> None:
        while self._open:
            try:
                data, _ = self._socket.recvfrom(MAX_DATAGRAM)
            except socket.timeout:
                continue
            except OSError:   # the socket was closed
                return
            try:
                reading = decode(data)
            except Exception:   # a stray packet must not stop the receiver
                continue
            self.put(reading.poses, reading.buttons, time.monotonic() - reading.age)

    def close(self) -> None:
        self._open = False
        self._socket.close()


def stream(robot: str, port: int) -> None:
    # Stream the headset plugged into this computer to `robot`, until Ctrl+C.
    headset = TableHeadset()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"streaming the controllers to {robot}:{port}")
    sent = skipped = 0
    reported = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            reading = headset.latest()
            if reading.age < FRESH and reading.poses:
                sender.sendto(encode(reading), (robot, port))
                sent += 1
            else:
                skipped += 1   # silence makes the robot hold
            if now - reported >= REPORT_EVERY:
                print(f"{sent / (now - reported):.0f} packets/s")
                if skipped > sent:
                    print("not tracking: the headset is asleep or does not see the controllers")
                sent = skipped = 0
                reported = now
            time.sleep(max(0.0, 1.0 / SEND_RATE - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        headset.close()
        sender.close()


def show() -> None:
    # Print what the headset sends, until Ctrl+C.
    headset = TableHeadset()
    np.set_printoptions(precision=3, suppress=True)
    try:
        while True:
            time.sleep(PRINT_EVERY)
            reading = headset.latest()
            if reading.age > FRESH:
                print("no data: the headset is asleep or the app is not in the foreground")
            elif not reading.poses:
                print("the headset is awake but does not see the controllers")
            else:
                held = [name for name, value in reading.buttons.items() if value is True]
                positions = "  ".join(f"{side} {pose[:3, 3]}" for side, pose in sorted(reading.poses.items()))
                print(positions, "| held:", held)
    except KeyboardInterrupt:
        pass
    finally:
        headset.close()


def main(arguments: Optional[list] = None) -> None:
    # Command line: print the samples, or stream them with --to.
    parser = argparse.ArgumentParser(description="Print the table headset's samples, or stream them to the robot.")
    parser.add_argument("--to", metavar="ROBOT", help="the robot computer's address; without it, print")
    robot = parser.parse_args(arguments).to
    if robot:
        stream(robot, config.load()["table"]["port"])
    else:
        show()


if __name__ == "__main__":
    main()
