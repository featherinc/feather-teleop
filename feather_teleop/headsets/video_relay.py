# The worn headset's video in a process of its own: the camera service's RTP/H.264 streams in on this computer's UDP ports, whole access units out to the page on a WebSocket of its own (`/video` on `--port`).
#
#     camera service's process --RTP/UDP, 127.0.0.1--> this process --wss /video--> the headset's page
#
# Started and ended by worn.VideoRelay (quest.yaml's `worn.video_port`). The teleop's own process also runs the SDK, the solver and the loop, so a frame forwarded there waits its turn among them and holds the poses' socket up; here nothing else runs.
#
# Control, one datagram each on the UDP port the ready line names: RELAY_LOOK + up, left (two floats, rad: where the robot's camera looks, carried by every frame after it), RELAY_RESET (the streams start again). The first line printed is `ready <the WebSocket's port> <the control port>`. It ends when its standard input closes, so it never outlives the process that started it.
import argparse
import asyncio
import os
import socket
import struct
import sys
import threading

from aiohttp import web

from feather_teleop.headsets.worn import BACKLOG, HEARTBEAT, RELAY_LOOK, RELAY_RESET, TYPE_VIDEO, UINT32, VIDEO, Depacketizer, tls_context, is_keyframe

NOT_KNOWN = (float("nan"), float("nan"))
RECEIVE_BUFFER = 4 * 1024 * 1024   # bytes of socket buffer per camera
MAX_MESSAGE = 4096                 # bytes; the page sends nothing on this socket


class Relay:
    # The streams' packets in (`packet`), their whole access units out to the newest page that opened `/video`.

    def __init__(self, streams):
        self.streams = list(streams)   # the stream numbers (worn.STREAMS)
        self.client = None
        self.transport = None
        self.look = NOT_KNOWN
        self._sends = set()
        self.reset()

    def reset(self) -> None:
        self.units = {stream: Depacketizer() for stream in self.streams}
        self.numbers = dict.fromkeys(self.streams, 0)
        self.waiting_for_key = dict.fromkeys(self.streams, True)

    def control(self, data: bytes) -> None:
        if data[:1] == RELAY_LOOK and len(data) == 9:
            self.look = struct.unpack("<ff", data[1:])
        elif data[:1] == RELAY_RESET:
            self.reset()

    def packet(self, stream: int, data: bytes) -> None:
        unit = self.units[stream].take(data)
        if unit is None:
            return
        self.numbers[stream] += 1
        client, transport = self.client, self.transport
        if client is None or client.closed or transport is None or transport.get_write_buffer_size() > BACKLOG:
            self.waiting_for_key[stream] = True   # the link is behind: a late picture is worse than a missing one
            return
        key = is_keyframe(unit)
        if self.waiting_for_key[stream] and not key:
            return
        self.waiting_for_key[stream] = False
        message = VIDEO.pack(TYPE_VIDEO, key, stream, self.numbers[stream] & UINT32, float("nan"), *self.look) + unit
        task = asyncio.ensure_future(self._send(client, message))
        self._sends.add(task)
        task.add_done_callback(self._sends.discard)

    async def _send(self, client, message: bytes) -> None:
        try:
            await client.send_bytes(message)
        except (ConnectionError, RuntimeError, asyncio.CancelledError):
            if client is self.client:
                self.waiting_for_key = dict.fromkeys(self.streams, True)

    async def socket(self, request):
        # A page's video socket: the newest one gets the video, from each stream's next keyframe.
        client = web.WebSocketResponse(heartbeat=HEARTBEAT, max_msg_size=MAX_MESSAGE, compress=False)
        await client.prepare(request)
        self.client, self.transport = client, request.transport
        self.waiting_for_key = dict.fromkeys(self.streams, True)
        try:
            async for _ in client:   # the page sends nothing here; this ends when it leaves
                pass
        finally:
            if self.client is client:
                self.client = self.transport = None
        return client


class _Datagrams(asyncio.DatagramProtocol):
    def __init__(self, take):
        self.take = take

    def datagram_received(self, data, addr):
        self.take(data)


def _udp(port: int) -> socket.socket:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER)
    receiver.bind(("127.0.0.1", port))
    receiver.setblocking(False)
    return receiver


async def serve(address: str, port: int, context, ports: dict) -> None:
    # Serve until cancelled. `ports`: {UDP port on this computer: stream number}.
    loop = asyncio.get_running_loop()
    relay = Relay(ports.values())
    for udp_port, stream in ports.items():
        await loop.create_datagram_endpoint(lambda stream=stream: _Datagrams(lambda data: relay.packet(stream, data)), sock=_udp(udp_port))
    control = _udp(0)
    await loop.create_datagram_endpoint(lambda: _Datagrams(relay.control), sock=control)

    async def hello(request):   # a browser opened here by hand accepts the certificate for this port
        return web.Response(text="The headset's video port. Go back to the headset's page.")
    app = web.Application()
    app.router.add_get("/video", relay.socket)
    app.router.add_get("/", hello)
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    await web.TCPSite(runner, address, port, ssl_context=context).start()
    print(f"ready {runner.addresses[0][1]} {control.getsockname()[1]}", flush=True)
    await asyncio.Event().wait()


def _end_with_parent() -> None:
    sys.stdin.buffer.read()   # returns when the other end closes: the process that started this is gone
    os._exit(0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the worn headset's video from the camera service's streams.")
    parser.add_argument("--address", required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--streams", required=True, help="<udp port>:<stream number>, comma separated")
    parser.add_argument("--cert")
    parser.add_argument("--key")
    args = parser.parse_args()
    context = tls_context(args.cert, args.key) if args.cert or args.key else None
    ports = {int(port): int(stream) for port, stream in (item.split(":") for item in args.streams.split(","))}
    threading.Thread(target=_end_with_parent, daemon=True).start()
    asyncio.run(serve(args.address, args.port, context, ports))


if __name__ == "__main__":
    main()
