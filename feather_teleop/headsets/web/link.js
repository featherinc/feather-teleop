// The WebSockets to the robot: poses up and the status down on one, the video on the same one or, when the status names a `video_port`, on one of its own. Message layouts: ../worn.py (POSE and VIDEO).
import { problem, show } from "./page.js";

export const TYPE_POSE = 1, TYPE_VIDEO = 2;
export const BUTTON_STICK = 4, BUTTON_LOWER = 8, BUTTON_UPPER = 16;   // a hand's bits: stick click, A or X, B or Y

const TAKEN_OVER = 4001;     // close code: another headset took this one's place
const RECONNECT_MS = 500;
const MAX_QUEUED_POSES = 2;  // a pose sent behind more than this would arrive old, so it is dropped

// POSE: a 20-byte header, three poses of 7 floats (head, left, right), then two hands of 20 bytes.
const HEADER_BYTES = 20, POSE_BYTES = 28, HAND_BYTES = 20;
const MESSAGE_BYTES = HEADER_BYTES + 3 * POSE_BYTES + 2 * HAND_BYTES;
const AT_TRACKED = 1, AT_SEQUENCE = 4, AT_EPOCH = 8, AT_SENT = 12;
const UNTRACKED = [0, 0, 0, 0, 0, 0, 1];
const NO_INPUT = { bits: 0, trigger: 0, squeeze: 0, x: 0, y: 0 };

export class Link {
  // `onStatus(status)` gets each status message, `onVideo(buffer)` each video message, `onLost()` a lost link.
  constructor(onStatus, onVideo, onLost) {
    this.onStatus = onStatus;
    this.onVideo = onVideo;
    this.onLost = onLost;
    this.socket = null;
    this.sequence = 0;
    this.epoch = (Math.random() * 0xffffffff) >>> 0;
  }

  connect() {
    const scheme = location.protocol === "https:" ? "wss://" : "ws://";
    this.socket = new WebSocket(scheme + location.host + "/ws");
    this.socket.binaryType = "arraybuffer";
    this.socket.onopen = () => show("link", "connected");
    this.socket.onclose = (event) => this.closed(event);
    this.socket.onmessage = (event) => this.received(event.data);
  }

  closed(event) {
    this.onLost();
    if (event.code === TAKEN_OVER) {
      show("link", "another headset took over; reload the page to take it back");
      return;
    }
    show("link", "lost, reconnecting");
    setTimeout(() => this.connect(), RECONNECT_MS);
  }

  received(data) {
    if (typeof data === "string") {
      this.onStatus(JSON.parse(data));
    } else if (new DataView(data).getUint8(0) === TYPE_VIDEO) {
      this.onVideo(data);
    }
  }

  // Tell the robot the tracking frame changed (a new session, or recentred), so it takes "forward" again.
  newTrackingFrame() {
    this.epoch = (this.epoch + 1) >>> 0;
  }

  // `poses`: head, left and right as {position, orientation}, or null when not tracked.
  // `inputs`: left and right as {bits, trigger, squeeze, x, y}.
  sendPose(poses, inputs) {
    const socket = this.socket;
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    if (socket.bufferedAmount > MAX_QUEUED_POSES * MESSAGE_BYTES) return;

    const data = new DataView(new ArrayBuffer(MESSAGE_BYTES));
    let tracked = 0;
    let at = HEADER_BYTES;
    [poses.head, poses.left, poses.right].forEach((pose, i) => {
      if (pose) tracked |= 1 << i;
      for (const value of pose ? [...pose.position, ...pose.orientation] : UNTRACKED) {
        data.setFloat32(at, value, true);
        at += 4;
      }
    });
    for (const side of ["left", "right"]) {
      const input = inputs[side] || NO_INPUT;
      data.setUint16(at, input.bits, true);
      [input.trigger, input.squeeze, input.x, input.y].forEach((value, i) => data.setFloat32(at + 4 + 4 * i, value, true));
      at += HAND_BYTES;
    }
    data.setUint8(0, TYPE_POSE);
    data.setUint8(AT_TRACKED, tracked);
    data.setUint32(AT_SEQUENCE, this.sequence++ >>> 0, true);
    data.setUint32(AT_EPOCH, this.epoch, true);
    data.setFloat64(AT_SENT, performance.now(), true);
    socket.send(data.buffer);
  }

  // "connecting", "open", "closing" or "closed".
  state() {
    return this.socket ? SOCKET_STATES[this.socket.readyState] : "closed";
  }
}

const SOCKET_STATES = ["connecting", "open", "closing", "closed"];

// The video's own socket, to the robot's video process (../video_relay.py): a late frame then never holds a pose up.
export class VideoLink {
  // `onVideo(buffer)` gets each video message, `onLost()` a lost link.
  constructor(onVideo, onLost) {
    this.onVideo = onVideo;
    this.onLost = onLost;
    this.socket = null;
    this.port = null;
  }

  // Keep a socket open to `port`, or none for null. Opened again when lost, as long as the port is still wanted.
  want(port) {
    if (port === this.port && (port === null || (this.socket && this.socket.readyState <= WebSocket.OPEN))) return;
    if (this.socket) {
      this.socket.onclose = null;
      this.socket.close();
      this.socket = null;
    }
    this.port = port;
    if (port !== null) this.connect(port);
  }

  connect(port) {
    const scheme = location.protocol === "https:" ? "wss://" : "ws://";
    const socket = new WebSocket(scheme + location.hostname + ":" + port + "/video");
    socket.binaryType = "arraybuffer";
    socket.onmessage = (event) => {
      if (typeof event.data !== "string" && new DataView(event.data).getUint8(0) === TYPE_VIDEO) this.onVideo(event.data);
    };
    socket.onerror = () => problem(`No link to the video port ${port}: open ${location.protocol}//${location.hostname}:${port}/ in this browser once, accept its certificate, then reload this page.`);
    socket.onclose = () => {
      if (this.socket !== socket) return;
      this.socket = null;
      this.onLost();
      setTimeout(() => { if (this.port === port && !this.socket) this.connect(port); }, RECONNECT_MS);
    };
    this.socket = socket;
  }

  // "connecting", "open", "closing", "closed", or "none" without a video port.
  state() {
    if (this.port === null) return "none";
    return this.socket ? SOCKET_STATES[this.socket.readyState] : "closed";
  }
}
