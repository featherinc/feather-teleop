# Both headsets: the table one's log and laptop link, and the worn one's messages, web server and video.
import asyncio
import json
import socket
import struct
import subprocess
import time
from types import SimpleNamespace

import numpy as np
import pytest

from feather_teleop.control.maths import to_matrix
from feather_teleop.headsets import Reading, table, worn
from feather_teleop.headsets.worn import STREAMS, VIDEO, Depacketizer, ServiceVideo, VideoRelay, WornHeadset, is_keyframe, parse

aiohttp = pytest.importorskip("aiohttp")
STILL = (0.0, 0.0, 0.0, 1.0)   # a quaternion that turns nothing
NO_INPUT = (0, 0.0, 0.0, 0.0, 0.0)
NOTHING = Reading({}, {}, float("inf"))


def free_port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET, kind) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def wait_until(condition, seconds=2.0):
    deadline = time.monotonic() + seconds
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.01)
    return condition()


# The table headset.

def test_table_log_line():
    pose = " ".join(str(float(i)) for i in range(16))
    other = " ".join(str(float(i + 1)) for i in range(16))
    poses, buttons = table.parse(f"I {table.LOG_TAG}: l:{pose}|r:{other}&R,A,RG,rightTrig 0.5,rightJS 0.1 -0.2")
    assert poses["l"][0, 3] == 3.0 and poses["r"][3, 3] == 16.0
    assert buttons["A"] and buttons["RG"] and not buttons["B"] and "X" not in buttons   # the left one isn't on
    assert buttons["rightTrig"] == (0.5,) and buttons["rightJS"] == (0.1, -0.2)
    assert table.parse("an unrelated line") is None
    poses, buttons = table.parse(f"{table.LOG_TAG}: l:{pose}|r:{pose}&R,L,RG")
    assert poses == {} and buttons["RG"]   # the same pose for both: the controllers aren't tracked


def test_table_link():
    reading = Reading({"r": np.arange(16.0).reshape(4, 4)}, {"A": True, "rightJS": (0.25, -0.5)}, 0.012)
    assert table.decode(table.encode(reading)).buttons == reading.buttons
    assert np.array_equal(table.decode(table.encode(reading)).poses["r"], reading.poses["r"])

    port = free_port(socket.SOCK_DGRAM)
    receiver, sender = table.TableLink(port, "127.0.0.1"), socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        assert receiver.latest() == NOTHING
        sender.sendto(b"not a sample", ("127.0.0.1", port))   # ignored; the receiver keeps listening
        sender.sendto(table.encode(Reading({"r": 2 * np.eye(4)}, {"A": True}, 0.2)), ("127.0.0.1", port))
        assert wait_until(lambda: receiver.latest().buttons.get("A"))
        latest = receiver.latest()
        assert latest.poses["r"][0, 0] == 2.0 and 0.2 <= latest.age < 1.0   # the age the laptop measured counts
    finally:
        sender.close()
        receiver.close()


# The worn headset.

def message(sequence=1, epoch=5, head=((0, 1.6, 0), STILL), right=((0.2, 1.1, -0.3), STILL), lost=(), left_input=NO_INPUT, right_input=NO_INPUT, sent=1234.5):
    # A POSE message as main.js writes it, in WebXR's axes. A pose: ((x, y, z), quaternion).
    tracked = sum(1 << i for i, name in enumerate(("head", "l", "r")) if name not in lost)
    data = struct.pack("<BBHIId", 1, tracked, 0, sequence, epoch, sent)
    for place, turn in (head, ((-0.2, 1.1, -0.3), STILL), right):
        data += struct.pack("<7f", *place, *turn)
    for bits, *analog in (left_input, right_input):
        data += struct.pack("<H2x4f", bits, *analog)
    return data


def unit(*kinds):
    # An Annex B access unit with NAL units of these types.
    return b"".join(b"\x00\x00\x00\x01" + bytes([kind]) + b"\xaa\xbb" for kind in kinds)


KEYFRAME, DELTA = unit(9, 7, 8, 5), unit(9, 1)


@pytest.fixture
def headset():
    clock = [100.0]
    served = WornHeadset("127.0.0.1", 0, clock=lambda: clock[0])
    served.clock_now = clock
    yield served
    served.close()


def test_worn_message():
    facing_left = (0.0, np.sin(np.pi / 4), 0.0, np.cos(np.pi / 4))   # a quarter turn about WebXR's up
    packet = parse(message(right=((0.2, 1.1, -0.3), facing_left), lost=("l",), left_input=(worn.STICK | worn.UPPER, 0.25, 0.2, -1.0, 0.5), right_input=(worn.LOWER, 0.75, 0.9, 0.0, 0.0)))
    poses, b = packet["poses"], packet["buttons"]
    assert sorted(poses) == ["head", "r"]   # robot axes: x ahead, y left, z up
    assert poses["r"][:3, 3] == pytest.approx([0.3, -0.2, 1.1])
    assert poses["r"][:3, :3] == pytest.approx(to_matrix([0, 0, np.pi / 2]), abs=1e-6)
    assert (b["LG"], b["RG"]) == (False, True)   # a grip counts as held past half its travel
    assert (b["A"], b["B"], b["X"], b["Y"], b["LJ"], b["RJ"]) == (True, False, False, True, True, False)
    assert b["leftTrig"] == (0.25,) and b["rightTrig"] == (0.75,) and b["leftJS"] == (-1.0, 0.5)
    assert is_keyframe(KEYFRAME) and not is_keyframe(DELTA) and not is_keyframe(unit(9, 5)) and not is_keyframe(b"")


@pytest.mark.parametrize("spoil, why", [(lambda d: d[:-4], "140 bytes"), (lambda d: bytes([9]) + d[1:], "of type 9"), (lambda d: d[:20] + struct.pack("<f", float("nan")) + d[24:], "not finite"), (lambda d: d[:32] + struct.pack("<4f", 0.5, 0.5, 0.5, 0.9) + d[48:], "head: .* not unit length"), (lambda d: d[:-16] + struct.pack("<f", 1.5) + d[-12:], "right controller: an input outside its travel")])
def test_worn_message_refused(headset, spoil, why):
    with pytest.raises(ValueError, match=why):
        parse(spoil(message()))
    assert not headset.take(spoil(message())) and headset.latest() == NOTHING   # nothing taken: the arms hold


def test_worn_pose_age(headset):
    clock = headset.clock_now
    assert headset.take(message(sequence=10, right=((0.5, 1, 0), STILL)))
    assert not headset.take(message(sequence=9))   # older than the one in hand
    clock[0] += 0.03
    assert headset.latest().age == pytest.approx(0.03) and headset.epoch == 5
    assert headset.take(message(sequence=1, epoch=6)) and headset.epoch == 5   # a new frame, its epoch handed out
    headset.latest()                                                          # with its poses
    assert headset.epoch == 6

    headset.take(message(sequence=2, epoch=6, sent=5000.0))   # the quickest trip so far: taken as just sent
    page_started = clock[0] - 5.0                             # the robot's time when the page's clock read 0
    clock[0] += 0.5                                           # the link stalls, then delivers a pose sent 20 ms on
    headset.take(message(sequence=3, epoch=6, sent=5020.0))
    assert headset.latest().age == pytest.approx(0.48, abs=1e-3)

    for i in range(3600):   # an hour, the page's clock 50 ppm slow: poses don't grow ever older
        clock[0] += 1.0
        headset.take(message(sequence=4 + i, epoch=6, sent=(clock[0] - page_started) * (1.0 - 50e-6) * 1e3))
    assert headset.latest().age < 0.01


def test_worn_server():
    served = WornHeadset("127.0.0.1", 0)
    url = f"http://127.0.0.1:{served.port}"

    async def pages():
        async with aiohttp.ClientSession() as session:
            async with session.get(url + "/") as response:
                assert response.status == 200 and "main.js" in await response.text()
            async with session.get(url + "/main.js") as response:   # never from the browser's cache
                assert response.status == 200 and "requestSession" in await response.text()
                assert response.headers["Cache-Control"] == "no-store"

            first = await session.ws_connect(url + "/ws")
            assert json.loads((await first.receive()).data)["phase"] == "ready"
            await first.send_bytes(message(epoch=11, right_input=(0, 0.0, 1.0, 0.0, 0.0)))
            await asyncio.to_thread(wait_until, lambda: served.latest().age < 1.0)
            assert served.latest().buttons["RG"] and served.peer == "127.0.0.1"
            served.set_debug({"faults": ["left held back: J3"], "hz": 49.5})
            readout = json.loads((await first.receive()).data)["debug"]
            assert readout["faults"] == ["left held back: J3"] and readout["echo"][0] == 1234.5   # the page's clock in its pose

            watching = await session.ws_connect(url + "/ws")   # a page that only watches gets the status, no video
            await watching.receive()
            served.set_status(phase="active")
            for page in (first, watching):
                assert json.loads((await page.receive()).data)["phase"] == "active"

            served.look = (0.1, -0.2)
            served.send_video(DELTA, 1, 0.02, STREAMS["stereo"])   # mid-stream: the page can't start from it
            served.send_video(KEYFRAME, 2, 0.03, STREAMS["stereo"])
            served.send_video(DELTA, 3, 0.02, STREAMS["stereo"])
            served.send_video(KEYFRAME, 1, 0.01, STREAMS["right"])
            served.set_status(overlay={"right": {"width": 640, "height": 400}})
            stereo, more, wrist = [(await first.receive()).data for _ in range(3)]
            status = json.loads((await first.receive()).data)
            assert VIDEO.unpack_from(stereo)[:4] == (2, 1, STREAMS["stereo"], 2)
            assert VIDEO.unpack_from(stereo)[5:] == pytest.approx((0.1, -0.2)) and stereo[VIDEO.size:] == KEYFRAME
            assert VIDEO.unpack_from(more)[3] == 3 and VIDEO.unpack_from(wrist)[2] == STREAMS["right"]
            assert status["overlay"] == {"left": None, "right": {"width": 640, "height": 400}}
            assert status["phase"] == "active"

            second = await session.ws_connect(url + "/ws")   # another headset takes over for good
            await second.receive()
            await second.send_bytes(message(epoch=13, right=((0.5, 1.0, -0.3), STILL)))
            closed = await first.receive()
            assert closed.type == aiohttp.WSMsgType.CLOSE and closed.data == worn.TAKEN_OVER
            await asyncio.to_thread(wait_until, lambda: "r" in served.latest().poses)
            assert served.epoch == 13 and served.latest().poses["r"][1, 3] == pytest.approx(-0.5)
            await watching.close()
            await second.close()

    try:
        asyncio.run(pages())
    finally:
        served.close()
    served.set_status(phase="ready")   # nothing raises once closed
    served.send_video(KEYFRAME, 1, 0.0, STREAMS["stereo"])


def test_worn_https(tmp_path, monkeypatch):
    with pytest.raises(FileNotFoundError, match="cert.pem"):
        WornHeadset("127.0.0.1", 0, tmp_path / "cert.pem", tmp_path / "key.pem")
    cert, key = tmp_path / "cert.pem", tmp_path / "key.pem"
    subprocess.run(["openssl", "req", "-x509", "-nodes", "-days", "1", "-newkey", "rsa:2048", "-keyout", str(key), "-out", str(cert), "-subj", "/CN=test"], check=True, capture_output=True)
    redirect = free_port()
    monkeypatch.setattr(worn, "REDIRECT_PORT", redirect)
    served = WornHeadset("127.0.0.1", 0, cert, key)

    async def ask():
        async with aiohttp.ClientSession() as session:
            async with session.get(f"http://127.0.0.1:{redirect}/", allow_redirects=False) as response:
                return response.status, response.headers["Location"]

    try:
        assert asyncio.run(ask()) == (302, f"https://127.0.0.1:{served.port}/")   # plain http leads to the page
    finally:
        served.close()


def rtp(sequence, payload, last):
    return struct.pack("!BBHII", 0x80, (0x80 if last else 0) | 96, sequence, 0, 1) + payload


def fragments(nal, size):
    # A NAL unit as FU-A payloads carrying `size` bytes each.
    indicator = (nal[0] & 0xE0) | worn.FU_A
    pieces = [nal[1:][at:at + size] for at in range(0, len(nal) - 1, size)]
    return [bytes([indicator, (0x80 if i == 0 else 0) | (0x40 if i == len(pieces) - 1 else 0) | (nal[0] & 0x1F)]) + piece for i, piece in enumerate(pieces)]


def test_depacketizer():
    sps, idr = bytes([0x67, 1, 2, 3]), bytes([0x65]) + bytes(range(50))
    packets = [rtp(1, sps, False)] + [rtp(2 + i, piece, i == 3) for i, piece in enumerate(fragments(idr, 13))]
    whole = Depacketizer()
    assert [whole.take(p) for p in packets] == [None] * 4 + [b"\x00\x00\x00\x01" + sps + b"\x00\x00\x00\x01" + idr]
    lossy = Depacketizer()
    assert [lossy.take(p) for i, p in enumerate(packets) if i != 2] == [None] * 4   # a lost fragment drops the unit
    assert lossy.take(rtp(6, bytes([0x41, 9]), True)) == b"\x00\x00\x00\x01" + bytes([0x41, 9])
    assert lossy.take(b"\x80") is None   # too short to be a packet


def fake_cameras(first, names, refuse):
    # A camera service streaming `names` from UDP port `first` on; it refuses past the budget while refuse[0].
    shown = []

    def start_stream(host, which):
        if refuse[0]:
            raise ValueError("over the encoders' budget")
        return {name: first + names.index(name) for name in which}

    def show_stream(which):
        if refuse[0]:
            raise ValueError("over the encoders' budget")
        shown.append(list(which))
    calibration = SimpleNamespace(fx=1000.0, fy=1000.0, cx=640.0, cy=360.0, width=1280, height=720, baseline_m=0.12)
    cameras = SimpleNamespace(names=names, faults={}, config=SimpleNamespace(stream_port=first, stream_height=360), shape=lambda name: (720, 2560 if name == "stereo" else 1280, 3), start_stream=start_stream, show_stream=show_stream, stop_stream=lambda: None, calibration=lambda name: calibration)
    return cameras, shown


def test_service_video():
    pytest.importorskip("feathersdk.robot.services.camera_encoding")
    first = free_port(socket.SOCK_DGRAM)
    names = ["stereo", "right_wrist", "rear"]   # "rear": a camera the headset never shows
    sent, statuses, refuse = [], [], [True]
    cameras, shown = fake_cameras(first, names, refuse)
    headset = SimpleNamespace(send_video=lambda unit, number, age, stream: sent.append((stream, number)), set_status=lambda **fields: statuses.append(fields))
    video, sender = ServiceVideo(cameras, headset), socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def stream(camera, sequence):
        sender.sendto(rtp(sequence, bytes([0x41, sequence]), True), ("127.0.0.1", first + names.index(camera)))

    try:
        with pytest.raises(ValueError):
            video.start()   # refused: its ports are freed, so it can start again
        refuse[0] = False
        video.start()
        assert statuses[0]["camera"]["width"] == 640 and statuses[0]["camera"]["fx"] == pytest.approx(500.0)
        stream("right_wrist", 1)   # not shown: dropped
        stream("rear", 1)          # never forwarded
        stream("stereo", 1)
        assert wait_until(lambda: len(sent) == 1)
        video.show("right", True)
        assert shown == [["stereo", "right_wrist"]] and statuses[-1] == {"overlay": {"right": {"width": 640, "height": 360}}}
        stream("right_wrist", 2)
        assert wait_until(lambda: len(sent) == 2)
        assert sent == [(STREAMS["stereo"], 1), (STREAMS["right"], 1)]
        video.show("right", False)   # hiding is a show_stream too
        assert shown[-1] == ["stereo"]
        refuse[0] = True
        with pytest.raises(ValueError):
            video.show("right", True)
        assert video.shown["right"] is False   # refused: stays hidden
        refuse[0] = False
        video.show("right", True)
        video.restart()
        assert video.shown["right"] and shown[-1] == ["stereo", "right_wrist"]   # shown again after the restart
    finally:
        sender.close()
        video.close()
    assert statuses[-1] == {"camera": None} and {"overlay": {"right": None}} in statuses


def test_video_relay():
    pytest.importorskip("feathersdk.robot.services.camera_encoding")
    first = free_port(socket.SOCK_DGRAM)
    cameras, _ = fake_cameras(first, ["stereo", "right_wrist"], [False])
    statuses = []
    headset = SimpleNamespace(address="127.0.0.1", cert=None, key=None, look_sink=None, set_status=lambda **fields: statuses.append(fields))
    video = ServiceVideo(cameras, headset, video_port=0)   # the video from a process of its own
    relay = video.relay
    assert statuses == [{"video_port": relay.port}]
    headset.look_sink((0.1, -0.2))
    video.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sps, idr = bytes([0x67, 1, 2]), bytes([0x65, 3, 4])

    async def watch():
        async with aiohttp.ClientSession() as session:
            page = await session.ws_connect(f"http://127.0.0.1:{relay.port}/video")
            for sequence in range(0, 200, 2):   # until the relay has the page: what came before is dropped
                sender.sendto(rtp(sequence, sps, False), ("127.0.0.1", first))
                sender.sendto(rtp(sequence + 1, idr, True), ("127.0.0.1", first))
                try:
                    return (await page.receive(timeout=0.05)).data
                except asyncio.TimeoutError:
                    continue
            return None

    try:
        data = asyncio.run(watch())
        assert data is not None
        assert VIDEO.unpack_from(data)[:3] == (worn.TYPE_VIDEO, 1, STREAMS["stereo"]) and VIDEO.unpack_from(data)[5:] == pytest.approx((0.1, -0.2))
        assert data[VIDEO.size:] == b"\x00\x00\x00\x01" + sps + b"\x00\x00\x00\x01" + idr
    finally:
        sender.close()
        video.close()
    assert video.relay is None and relay.process.poll() is not None and statuses[-1] == {"video_port": None}


def test_video_relay_fails_loudly():
    with socket.socket() as taken:   # its port is held: the relay says so
        taken.bind(("127.0.0.1", 0))
        taken.listen()
        with pytest.raises(RuntimeError, match="could not be served"):
            VideoRelay("127.0.0.1", taken.getsockname()[1], {free_port(socket.SOCK_DGRAM): 0})
