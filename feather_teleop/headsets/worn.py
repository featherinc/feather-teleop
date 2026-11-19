# The headset worn by the operator, through a WebXR page the robot serves over one WebSocket.
#
# Poses come up in WebXR's axes and are turned into the robot's here. Status (JSON) and the cameras' H.264 go down: on the same socket, or with `video_port` from a process of its own (video_relay.py) on a socket of its own, so a frame never holds a pose up. A browser grants an immersive session only over https, so the page needs a certificate (see the README).
import asyncio
import json
import select
import socket
import ssl
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from feather_teleop.headsets import HEAD, Reading

PAGE = Path(__file__).resolve().parent / "web"

# Up, every frame: type, tracked bits (head, left, right), padding, sequence, epoch, page clock (ms); the head, left and right poses as x y z qx qy qz qw; per hand (left, right): button bits, padding, trigger, squeeze, stick x, y.
POSE = struct.Struct("<BBHIId" + "7f" * 3 + "H2x4f" * 2)
POSE_VALUES = 7          # x y z and a quaternion
POSES_AT = 6             # index of the first pose value among the unpacked fields
HANDS_AT = POSES_AT + 3 * POSE_VALUES
HAND_FIELDS = 5          # bits, trigger, squeeze, stick x, stick y
# Down, before each H.264 access unit: type, keyframe (1 or 0), stream, frame number, age when sent (ms), and where the camera looked when it was taken (up, left in rad, in the operator's tracking frame; NaN if unknown).
VIDEO = struct.Struct("<BBHIfff")
TYPE_POSE, TYPE_VIDEO = 1, 2
STREAMS = {"stereo": 0, "left": 1, "right": 2}
UINT32 = 0xFFFFFFFF      # sequence and frame numbers wrap at 32 bits

TRACKED = (HEAD, "l", "r")   # the poses in message order
HANDS = (("L", "left", "X", "Y"), ("R", "right", "A", "B"))   # in message order: prefix, side, face buttons
STICK, LOWER, UPPER = 4, 8, 16   # a hand's button bits: the stick clicked, A or X, B or Y
SQUEEZE_HELD = 0.5        # of the grip's travel: further in counts as held
QUATERNION_SLACK = 0.02   # a tracked pose's quaternion must be this close to unit length
TO_ROBOT = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # WebXR's axes to the robot's

TAKEN_OVER = 4001         # close code for a page another headset replaced; it does not reconnect
TAKEN_OVER_GRACE = 1.0    # s that page gets to hear why before its connection is cut
BACKLOG = 262_144         # bytes queued on the socket past which video waits for the next keyframe
HEARTBEAT = 5.0           # s between WebSocket pings
MAX_MESSAGE = 4096        # bytes; the page only ever sends poses
START_TIMEOUT = 10.0      # s to wait for the server to start
REDIRECT_PORT = 8080      # plain http here sends a browser on to the https page
CLOCK_DRIFT = 1e-4        # s/s the page's clock may drift from the robot's (100 ppm; crystals drift under 50)

NAL_IDR, NAL_SPS = 5, 7
NAL_START = b"\x00\x00\x01"
NAL_START_LONG = b"\x00\x00\x00\x01"
NAL_TYPE = 0x1F
RTP_HEADER = struct.Struct("!BBHII")   # RFC 3550: flags, marker and payload type, sequence, timestamp, source
RTP_MARKER = 0x80                      # set on an access unit's last packet
RTP_SEQUENCE = 0xFFFF                  # RTP sequence numbers wrap at 16 bits
SINGLE_NAL = (1, 23)                   # NAL types sent whole in one packet
RELAY_LOOK, RELAY_RESET = b"L", b"R"    # the video relay's control datagrams: where the camera looks, the streams start again
FU_A, FU_HEADER, FU_START, FU_END = 28, 2, 0x80, 0x40   # a NAL fragmented over packets (RFC 6184)


def _rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    # The rotation matrix of a unit quaternion.
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_matrix(values) -> np.ndarray:
    # (x, y, z, qx, qy, qz, qw) in WebXR's axes as a 4x4 pose in the robot's.
    x, y, z, w = np.asarray(values[3:], float) / np.linalg.norm(values[3:])
    pose = np.eye(4)
    pose[:3, :3] = TO_ROBOT @ _rotation(x, y, z, w) @ TO_ROBOT.T
    pose[:3, 3] = TO_ROBOT @ np.asarray(values[:3], float)
    return pose


def parse(data: bytes) -> dict:
    # Unpack one POSE message into its sequence, epoch, poses, buttons and the page's clock (s, as "sent").
    #
    # Raises:
    #     ValueError: Not a pose message, or one holding an impossible value.
    if len(data) != POSE.size or data[0] != TYPE_POSE:
        raise ValueError(f"{len(data)} bytes of type {data[0] if data else None}; a pose is {POSE.size} bytes of type {TYPE_POSE}")
    fields = POSE.unpack(data)
    tracked, sequence, epoch, sent_ms = fields[1], fields[3], fields[4], fields[5]
    numbers = np.array(fields[POSES_AT:], float)
    if not np.isfinite(numbers).all():
        raise ValueError("a pose message holding a number that is not finite")
    poses = _poses(numbers, tracked)
    buttons = _buttons(fields)
    return {"sequence": sequence, "epoch": epoch, "poses": poses, "buttons": buttons, "sent": sent_ms / 1e3}


def _poses(numbers: np.ndarray, tracked: int) -> dict:
    # The tracked poses, by name, from the message's numbers.
    poses = {}
    for i, name in enumerate(TRACKED):
        values = numbers[POSE_VALUES * i:POSE_VALUES * (i + 1)]
        if tracked & (1 << i):
            if abs(np.linalg.norm(values[3:]) - 1.0) > QUATERNION_SLACK:
                raise ValueError(f"{name}: a tracked pose whose quaternion is not unit length")
            poses[name] = pose_matrix(values)
    return poses


def _buttons(fields: tuple) -> dict:
    # Both hands' buttons, under the table headset's names.
    buttons = {}
    for i, (hand, side, lower, upper) in enumerate(HANDS):
        at = HANDS_AT + HAND_FIELDS * i
        bits, trigger, squeeze, stick_x, stick_y = fields[at:at + HAND_FIELDS]
        if not all(-1.0 <= value <= 1.0 for value in (trigger, squeeze, stick_x, stick_y)):
            raise ValueError(f"{side} controller: an input outside its travel")
        buttons.update({lower: bool(bits & LOWER), upper: bool(bits & UPPER), hand + "J": bool(bits & STICK), hand + "G": squeeze > SQUEEZE_HELD, f"{side}Trig": (trigger,), f"{side}JS": (stick_x, stick_y)})
    return buttons


def _is_older(sequence: int, than: int) -> bool:
    # Whether `sequence` comes before `than`, allowing for the 32-bit wrap.
    return (sequence - than) & UINT32 > UINT32 // 2


def is_keyframe(unit: bytes) -> bool:
    # Whether an Annex B access unit decodes by itself: it carries the parameter sets and an IDR picture.
    kinds = set()
    at = unit.find(NAL_START)
    while 0 <= at < len(unit) - len(NAL_START):
        kinds.add(unit[at + len(NAL_START)] & NAL_TYPE)
        at = unit.find(NAL_START, at + len(NAL_START))
    return NAL_IDR in kinds and NAL_SPS in kinds


def tls_context(cert, key) -> ssl.SSLContext:
    # A server TLS context from the certificate's files.
    cert, key = Path(cert or "").expanduser(), Path(key or "").expanduser()
    missing = [str(path) for path in (cert, key) if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"no {' or '.join(missing)}: make the certificate as the README says")
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(str(cert), str(key))
    return context


class WornHeadset:
    # Serves the page and reads the headset that sends poses to it.
    #
    # An open page that sends nothing is only told the status. The first page to send a pose is the headset; when another starts sending, it takes over and the first is closed for good.
    #
    # Attributes:
    #     epoch: Changes when the headset's tracking frame does (recentred, or the session entered again).
    #     look: Where the robot's camera looks, (up, left) in rad in the operator's tracking frame, or None. Set by the neck and sent with each video frame, so the page shows the picture where the camera was looking.
    #
    # Args:
    #     address, port: Where to serve the page.
    #     cert, key: The certificate's files, or None for plain http.
    #     say: Called with each message for the operator.
    #     clock: Returns the time in seconds. Replaced in tests.

    def __init__(self, address: str, port: int, cert=None, key=None, say=print, clock=time.monotonic):
        from aiohttp import web  # only the worn headset needs it
        self._web = web
        self.say = say
        self.clock = clock
        self.epoch = None
        self.look_sink = None  # called with each new `look`, or None: the video relay, which carries it with each frame
        self.look = None
        self.peer = None       # the headset's address
        self.address = address
        self.cert, self.key = (cert, key) if cert or key else (None, None)   # as served: the video relay serves the same way
        self.rejected = None   # why the last message was not taken
        self._lock = threading.Lock()
        self._packet = None
        self._stamp = None     # when the newest pose was sent, on this computer's clock
        self._lead = None      # the least (arrival - the page's clock) seen from this headset
        self._lead_at = 0.0    # when it was taken
        self._arrived = None   # (the page's clock in the newest pose, when it arrived here), for the debug readout's round trip
        self._status = {"phase": "ready", "camera": None, "overlay": {"left": None, "right": None}, "video_port": None, "debug": None}
        self._client = None
        self._transport = None
        self._wait_for_keyframes()
        self._sockets = set()   # every open page's WebSocket
        self._sends = set()
        self._ssl = tls_context(cert, key) if cert or key else None
        self._loop = asyncio.new_event_loop()
        self._ready = threading.Event()
        self._failed = None
        self._thread = threading.Thread(target=self._serve, args=(address, port), name="worn_headset", daemon=True)
        self._thread.start()
        if not self._ready.wait(START_TIMEOUT) or self._failed:
            raise RuntimeError(f"the headset's page could not be served on {address}:{port}: {self._failed or 'no answer'}")

    @property
    def look(self):
        return self._look

    @look.setter
    def look(self, value) -> None:
        self._look = value
        if self.look_sink is not None:
            self.look_sink(value)

    # The server, on its own thread and event loop.

    def _serve(self, address: str, port: int) -> None:
        asyncio.set_event_loop(self._loop)
        runners = []
        try:
            app = self._web.Application()
            app.router.add_get("/", self._page)
            app.router.add_get("/ws", self._socket)
            app.router.add_get("/{script:[a-z_]+[.]js}", self._script)
            app.router.add_static("/", PAGE)
            runner = self._web.AppRunner(app, access_log=None)
            self._loop.run_until_complete(runner.setup())
            runners.append(runner)
            self._loop.run_until_complete(self._web.TCPSite(runner, address, port, ssl_context=self._ssl).start())
            self.port = runner.addresses[0][1]   # the port bound, when `port` was 0
            if self._ssl is not None:
                redirect = self._loop.run_until_complete(self._redirect(address))
                if redirect is not None:
                    runners.append(redirect)
        except Exception as error:
            self._failed = f"{type(error).__name__}: {error}"
            self._ready.set()
            return
        self._ready.set()
        self._loop.run_forever()
        for runner in runners:
            self._loop.run_until_complete(runner.cleanup())
        self._loop.close()

    async def _redirect(self, address: str):
        # Serve plain http on REDIRECT_PORT, sending every request on to the https page.
        #
        # A browser given only `<address>:<port>` asks for http, which the https port cannot answer.
        async def onward(request):
            raise self._web.HTTPFound(f"https://{request.url.host}:{self.port}/")
        app = self._web.Application()
        app.router.add_get("/{anything:.*}", onward)
        runner = self._web.AppRunner(app, access_log=None)
        await runner.setup()
        try:
            await self._web.TCPSite(runner, address, REDIRECT_PORT).start()
        except OSError as error:   # the port is taken; the page itself still works
            await runner.cleanup()
            self.say(f"headset: http on port {REDIRECT_PORT} does not lead to the page ({error})")
            return None
        return runner

    async def _page(self, request):
        return self._web.FileResponse(PAGE / "index.html", headers={"Cache-Control": "no-store"})

    async def _script(self, request):
        # Never from the browser's cache: a page that kept an older script does not understand what the status now says.
        path = PAGE / request.match_info["script"]
        if not path.is_file():
            raise self._web.HTTPNotFound()
        return self._web.FileResponse(path, headers={"Cache-Control": "no-store"})

    async def _socket(self, request):
        # Not compressed: the video is H.264 already, and this thread also receives the poses.
        client = self._web.WebSocketResponse(heartbeat=HEARTBEAT, max_msg_size=MAX_MESSAGE, compress=False)
        await client.prepare(request)
        self._sockets.add(client)
        self._send(client, client.send_str(json.dumps(self._status)))
        try:
            async for message in client:
                if message.type != self._web.WSMsgType.BINARY:
                    continue
                if client is not self._client and not self._take_over(client, request, message.data):
                    continue
                self.take(message.data)
        finally:
            self._sockets.discard(client)
            if self._client is client:
                self._client = self._transport = None
        return client

    def _take_over(self, client, request, data: bytes) -> bool:
        # Make the page that sent `data` the headset, if `data` is a valid pose. The previous headset is closed.
        try:
            parse(data)
        except (ValueError, struct.error) as error:
            self.rejected = str(error)
            return False
        previous, previous_transport = self._client, self._transport
        self._client, self._transport = client, request.transport
        self._wait_for_keyframes()
        self.peer = request.remote
        with self._lock:
            self._packet = self._stamp = self._lead = None   # keep nothing of the previous headset's
        if previous is not None:
            self._send(previous, previous.close(code=TAKEN_OVER, message=b"another headset took over"))
            if previous_transport is not None:
                self._loop.call_later(TAKEN_OVER_GRACE, previous_transport.abort)
        return True

    def _send(self, client, sending) -> None:
        # Run a send without waiting for it. If it fails (the page left), video restarts from a keyframe.
        async def run():
            try:
                await sending
            except (ConnectionError, RuntimeError, asyncio.CancelledError):
                if client is self._client:
                    self._wait_for_keyframes()
        task = asyncio.ensure_future(run())
        self._sends.add(task)
        task.add_done_callback(self._sends.discard)

    def _wait_for_keyframes(self) -> None:
        # Hold every video stream until its next keyframe.
        self._waiting_for_key = dict.fromkeys(STREAMS.values(), True)

    # Poses from the headset.

    def take(self, data: bytes) -> bool:
        # Take one message from the headset's page. Returns False if it was not taken (`rejected` says why).
        #
        # The page's clock is offset from this one by an unknown amount, so a pose's age counts from the quickest trip seen so far: a pose held up by a stalled link does not arrive looking new.
        try:
            packet = parse(data)
        except (ValueError, struct.error) as error:
            self.rejected = str(error)
            return False
        now = self.clock()
        with self._lock:
            newest = self._packet
            if newest is not None and newest["epoch"] != packet["epoch"]:
                self._lead = None   # the page was opened again, and its clock restarted
            elif newest is not None and _is_older(packet["sequence"], newest["sequence"]):
                return False
            lead = now - packet["sent"]
            quickest = lead
            if self._lead is not None:   # the quickest trip so far ages, so a drifting clock is followed
                quickest = min(lead, self._lead + CLOCK_DRIFT * (now - self._lead_at))
            self._lead = quickest
            self._lead_at = now
            self._packet = packet
            self._stamp = now - (lead - quickest)
            self._arrived = (packet["sent"], now)
        return True

    def latest(self) -> Reading:
        # Return the newest sample, and set `epoch` to its tracking frame's.
        with self._lock:
            packet, stamp = self._packet, self._stamp
        if packet is None:
            return Reading({}, {}, float("inf"))
        self.epoch = packet["epoch"]
        return Reading(packet["poses"], packet["buttons"], self.clock() - stamp)

    # Status and video to the pages.

    def set_status(self, **fields) -> None:
        # Change entries of the status and send it to every open page. Callable from any thread.
        #
        # phase: "active" while a demonstration runs (the page shows the camera), or "ready".
        # camera: The picture's pinhole model, or None while there is no picture.
        # overlay: {side: the wrist camera's {width, height}, or None to hide it}; merged per side.
        # video_port: The port the page gets the video from, on a socket of its own; None: this one.
        # debug: What the page's debug readout shows of the robot's side (see set_debug), or None for no readout.
        with self._lock:
            if "overlay" in fields:
                fields["overlay"] = {**self._status["overlay"], **fields["overlay"]}
            self._status = {**self._status, **fields}
        try:
            self._loop.call_soon_threadsafe(self._send_status)
        except RuntimeError:   # the loop is closed
            pass

    def _send_status(self) -> None:
        with self._lock:   # read on the loop's thread, so pages get the statuses in the order they were set
            text = json.dumps(self._status)
        for client in list(self._sockets):
            if not client.closed:
                self._send(client, client.send_str(text))

    def set_debug(self, info: Optional[dict]) -> None:
        # Tell the page what its debug readout shows: {"faults": [text], "hz": the loop's rate}, or None for no readout.
        #
        # Added here: `echo`, the page's own clock in the newest pose and how long ago it arrived (both ms), from which the page works out the round trip.
        if info is not None:
            with self._lock:
                arrived = self._arrived
            echo = None if arrived is None else [arrived[0] * 1e3, (self.clock() - arrived[1]) * 1e3]
            info = {**info, "echo": echo}
        self.set_status(debug=info)

    def send_video(self, unit: bytes, number: int, age: float, stream: int) -> None:
        # Send one access unit of `stream` (see STREAMS) to the headset. Callable from any thread.
        #
        # Dropped while there is no headset, and from when its socket falls behind until the next keyframe.
        up, left = self.look or (float("nan"), float("nan"))
        key = is_keyframe(unit)
        message = VIDEO.pack(TYPE_VIDEO, key, stream, number & UINT32, age * 1e3, up, left) + unit
        try:
            self._loop.call_soon_threadsafe(self._send_video, message, key, stream)
        except RuntimeError:   # the loop is closed
            pass

    def _send_video(self, message: bytes, key: bool, stream: int) -> None:
        client, transport = self._client, self._transport
        connected = client is not None and not client.closed and transport is not None
        if not connected or transport.get_write_buffer_size() > BACKLOG:
            self._waiting_for_key[stream] = True   # a late picture is worse than a missing one
            return
        if self._waiting_for_key[stream] and not key:
            return
        self._waiting_for_key[stream] = False
        self._send(client, client.send_bytes(message))

    def close(self) -> None:
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=START_TIMEOUT)


class Depacketizer:
    # Reassembles one camera's RTP/H.264 packets (RFC 6184) into Annex B access units.
    #
    # A unit with a packet missing is dropped whole, so the headset waits for the next keyframe rather than decoding a broken picture.

    def __init__(self):
        self._nals = []
        self._fragment: Optional[bytearray] = None
        self._next_sequence: Optional[int] = None
        self._broken = False

    def take(self, packet: bytes) -> Optional[bytes]:
        # Take one packet. Returns the access unit it completes, or None.
        if len(packet) <= RTP_HEADER.size:
            return None
        _, marker, sequence, _, _ = RTP_HEADER.unpack_from(packet)
        payload = packet[RTP_HEADER.size:]
        if self._next_sequence is not None and sequence != self._next_sequence:
            self._broken = True   # a packet was lost or came out of order
        self._next_sequence = (sequence + 1) & RTP_SEQUENCE
        kind = payload[0] & NAL_TYPE
        if kind == FU_A and len(payload) > FU_HEADER:
            self._take_fragment(payload)
        elif SINGLE_NAL[0] <= kind <= SINGLE_NAL[1]:
            self._nals.append(payload)
        if not marker & RTP_MARKER:
            return None
        return self._finish_unit()

    def _take_fragment(self, payload: bytes) -> None:
        # Add one FU-A fragment to the NAL being reassembled.
        header = payload[1]
        if header & FU_START:   # the NAL's own header, rebuilt from the indicator and the FU header
            self._fragment = bytearray([(payload[0] & ~NAL_TYPE & 0xFF) | (header & NAL_TYPE)])
        if self._fragment is None:
            self._broken = True
            return
        self._fragment += payload[FU_HEADER:]
        if header & FU_END:
            self._nals.append(bytes(self._fragment))
            self._fragment = None

    def _finish_unit(self) -> Optional[bytes]:
        # Join the NALs gathered into an access unit (None if one was lost), and start the next.
        unit = None if self._broken or not self._nals else b"".join(NAL_START_LONG + nal for nal in self._nals)
        self._nals, self._fragment, self._broken = [], None, False
        return unit


class VideoRelay:
    # video_relay.py as a process of its own: takes the camera service's streams on this computer's UDP ports and serves them to the page on (`address`, `port`), with the page's certificate.
    #
    # It ends with close(), and by itself when this process does: it waits on its standard input.
    #
    # Args:
    #     address, port: Where to serve the video socket; `port` after the start is the one bound.
    #     streams: {UDP port on this computer: stream number (see STREAMS)}.
    #     cert, key: The page's certificate's files, or None for plain http.
    START_TIMEOUT = 15.0   # s for the process to say it serves
    STOP_TIMEOUT = 3.0     # s for it to end once told to
    _running = {}          # {port asked for: the relay on it}: a new one on the same port (the notebook cell run again) ends the old one first

    def __init__(self, address: str, port: int, streams: dict, cert=None, key=None):
        before = VideoRelay._running.pop(port, None)
        if before is not None:
            before.close()
        command = [sys.executable, "-m", "feather_teleop.headsets.video_relay", "--address", address, "--port", str(port), "--streams", ",".join(f"{udp}:{stream}" for udp, stream in streams.items())]
        if cert is not None:
            command += ["--cert", str(cert), "--key", str(key)]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        ready = select.select([self.process.stdout], [], [], self.START_TIMEOUT)[0]
        line = self.process.stdout.readline().split() if ready else []
        if len(line) != 3 or line[0] != "ready":
            self.process.kill()
            why = self.process.communicate()[1].strip().splitlines()[-1:] or ["no answer"]
            raise RuntimeError(f"the headset's video could not be served on {address}:{port}: {why[0]}")
        self.port = int(line[1])
        self._control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._control.connect(("127.0.0.1", int(line[2])))
        if port:
            VideoRelay._running[port] = self

    def look(self, value) -> None:
        # Where the robot's camera looks from now on, (up, left) in rad or None; carried by each frame.
        self._tell(RELAY_LOOK + struct.pack("<ff", *(value or (float("nan"), float("nan")))))

    def reset(self) -> None:
        # The streams start again: each one's next unit is a new encoder's first.
        self._tell(RELAY_RESET)

    def _tell(self, data: bytes) -> None:
        try:
            self._control.send(data)
        except OSError:   # the process is gone: the picture stops, and the stream's fault says so
            pass

    def close(self) -> None:
        if self.process.stdin.closed:
            return
        VideoRelay._running = {port: relay for port, relay in VideoRelay._running.items() if relay is not self}
        self._control.close()
        self.process.stdin.close()   # it ends when this closes
        try:
            self.process.wait(timeout=self.STOP_TIMEOUT)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdout.close()
        self.process.stderr.close()


class ServiceVideo:
    # Forwards the camera service's live stream to the worn headset, without encoding anything in this process.
    #
    # The stereo camera always streams, a wrist camera only while it is shown: the service encodes just those (show_stream), at its config's `stream_*` settings, so a hidden wrist costs the encoders nothing.
    #
    # Args:
    #     cameras: The SDK camera service.
    #     headset: The WornHeadset to send to.
    #     address: The local address the service streams to.
    #     video_port: Serve the video from a VideoRelay on this port, so not a byte of it passes through this process; None: received here and sent down the page's pose socket.

    RECEIVE_TIMEOUT = 0.2             # s to wait for a packet before checking whether it was stopped
    RECEIVE_BUFFER = 4 * 1024 * 1024  # bytes of socket buffer per camera
    MAX_PACKET = 65536                # bytes, the largest UDP datagram
    STOP_TIMEOUT = 2.0                # s to wait for the receiving thread to end

    def __init__(self, cameras, headset: WornHeadset, address: str = "127.0.0.1", video_port: Optional[int] = None):
        from feathersdk.robot.services.camera_frames import STEREO
        self.cameras = cameras
        self.headset = headset
        self.address = address
        self.stereo = STEREO
        self.streams = {STEREO: STREAMS["stereo"], "left_wrist": STREAMS["left"], "right_wrist": STREAMS["right"]}
        self.shown = {"left": False, "right": False}
        self._sockets = {}
        self._thread = None
        self._open = False
        self.relay = None
        if video_port is not None:
            ports = {self._port(name): stream for name, stream in self.streams.items() if name in cameras.names}
            self.relay = VideoRelay(headset.address, video_port, ports, headset.cert, headset.key)
            headset.look_sink = self.relay.look
            headset.set_status(video_port=self.relay.port)

    def _port(self, name: str) -> int:
        # The UDP port the service streams camera `name` to: `stream_port` plus its place among the cameras.
        return self.cameras.config.stream_port + self.cameras.names.index(name)

    def failure(self) -> Optional[str]:
        # Why the service's stream stopped, or None.
        faults = [f"{name}: {why}" for name, why in self.cameras.faults.items() if name.startswith("stream:")]
        return "; ".join(faults) or None

    def size(self, name: str) -> Tuple[int, int]:
        # A camera's (width, height) as the service streams it.
        from feathersdk.robot.services.camera_encoding import scaled_size
        height, width, _ = self.cameras.shape(name)
        return scaled_size((width, height), self.cameras.config.stream_height)

    def model(self) -> dict:
        # The pinhole model of one eye of the stereo picture as streamed.
        c = self.cameras.calibration(self.stereo)
        width, height = self.size(self.stereo)
        width //= 2   # the eyes are side by side
        sx, sy = width / c.width, height / c.height
        return {"fx": c.fx * sx, "fy": c.fy * sy, "cx": c.cx * sx, "cy": c.cy * sy, "width": width, "height": height, "baseline": c.baseline_m}

    def start(self) -> None:
        # Have the service stream the stereo camera here, and forward it to the headset from now on.
        #
        # Raises:
        #     ValueError: The service refuses: its encoders would be over their budget.
        #     RuntimeError: The service streams to another port than the one listened on.
        if self.relay is not None:
            self.relay.reset()   # it listens on the ports already
        streaming = False
        try:
            if self.relay is None:
                for name in self.streams:   # listen before the first packet comes
                    if name in self.cameras.names:
                        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        self._sockets[name] = receiver
                        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.RECEIVE_BUFFER)
                        receiver.bind((self.address, self._port(name)))
            streamed = self.cameras.start_stream(self.address, [self.stereo])
            streaming = True
            if streamed != {self.stereo: self._port(self.stereo)}:
                raise RuntimeError(f"the camera service streams to ports {streamed}, not {self._port(self.stereo)}")
            self.headset.set_status(camera=self.model())
        except BaseException:
            if streaming:
                self.cameras.stop_stream()
            self._close_sockets()
            raise
        self._open = True
        if self.relay is None:
            self._thread = threading.Thread(target=self._receive, name="service_video", daemon=True)
            self._thread.start()

    def show(self, side: str, on: bool) -> None:
        # Show or hide that side's wrist camera in its corner of the headset's view, starting or ending its stream.
        #
        # Raises:
        #     ValueError: The service refuses: its encoders would be over their budget. The wrist stays hidden.
        name = f"{side}_wrist"
        if self.shown[side] == on or not self._open or name not in self.cameras.names:
            return
        wanted = {**self.shown, side: on}
        self.cameras.show_stream([self.stereo] + [f"{s}_wrist" for s, shown in wanted.items() if shown])
        self.shown[side] = on   # only once the service took it
        model = dict(zip(("width", "height"), self.size(name))) if on else None
        self.headset.set_status(overlay={side: model})

    def _forwarded(self, name: str) -> bool:
        # Whether a camera's video goes to the headset: the stereo camera's always, a wrist camera's while shown.
        if name not in self.streams:
            return False
        if not name.endswith("_wrist"):
            return True
        return bool(self.shown.get(name[:-len("_wrist")]))

    def _receive(self) -> None:
        names = {receiver.fileno(): name for name, receiver in self._sockets.items()}
        units = {name: Depacketizer() for name in self._sockets}
        numbers = dict.fromkeys(self._sockets, 0)
        receivers = list(self._sockets.values())
        while self._open:
            try:
                ready = select.select(receivers, [], [], self.RECEIVE_TIMEOUT)[0]
            except (OSError, ValueError):   # closed under it
                return
            for receiver in ready:
                try:
                    packet = receiver.recv(self.MAX_PACKET)
                except OSError:
                    continue
                name = names[receiver.fileno()]
                unit = units[name].take(packet)
                if unit is None or not self._forwarded(name):
                    continue
                numbers[name] += 1
                stream = self.streams[name]
                self.headset.send_video(unit, numbers[name], float("nan"), stream)

    def restart(self) -> None:
        # Start the service's stream again (every encoder new) after one stopped; the wrist cameras shown stay shown.
        shown = [side for side, on in self.shown.items() if on]
        self.stop()
        self.start()
        for side in shown:
            self.show(side, True)

    def stop(self) -> None:
        if not self._open:
            return
        self._open = False
        if self._thread is not None:
            self._thread.join(timeout=self.STOP_TIMEOUT)
            self._thread = None
        for side, on in self.shown.items():   # every overlay hidden; stop_stream() below ends every camera
            if on:
                self.shown[side] = False
                self.headset.set_status(overlay={side: None})
        try:
            self.cameras.stop_stream()
        finally:
            self._close_sockets()
            self.headset.set_status(camera=None)

    def _close_sockets(self) -> None:
        for receiver in self._sockets.values():
            receiver.close()
        self._sockets = {}

    def close(self) -> None:
        self.stop()
        if self.relay is not None:
            self.headset.look_sink = None
            self.headset.set_status(video_port=None)
            self.relay.close()
            self.relay = None
