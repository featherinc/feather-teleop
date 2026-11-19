// The robot's video streams, decoded by the browser's hardware decoder (WebCodecs).
import { problem, show } from "./page.js";

export const STEREO = 0, WRISTS = { left: 1, right: 2 };   // ../worn.py STREAMS

// VIDEO header: type, keyframe, stream, frame number, age, and where the camera looked (up, left).
const HEADER_BYTES = 20;
const AT_KEY = 1, AT_STREAM = 2, AT_NUMBER = 4, AT_UP = 12, AT_LEFT = 16;
const MAX_QUEUED_FRAMES = 3;   // a decoder further behind skips to the next keyframe
const LOOKS_KEPT = 60;         // stereo frames whose camera direction is kept until they are decoded
const SIZE_SHOWN_EVERY = 30;   // decoded frames between updates of the picture's size on the page
const NAL_TYPE = 0x1f, NAL_SPS = 7;

// The codec string of an Annex B keyframe, read off its sequence parameter set ("avc1.PPCCLL").
function codecOf(bytes) {
  for (let i = 0; i + 7 < bytes.length; i++) {
    const startCode = bytes[i] === 0 && bytes[i + 1] === 0 && bytes[i + 2] === 1;
    if (startCode && (bytes[i + 3] & NAL_TYPE) === NAL_SPS) {
      const profile = [bytes[i + 4], bytes[i + 5], bytes[i + 6]];
      return "avc1." + profile.map((b) => b.toString(16).padStart(2, "0")).join("");
    }
  }
  return null;
}

class Stream {
  constructor() {
    this.decoder = null;
    this.newest = null;   // the newest decoded frame not yet drawn: {frame, up, left}
    this.waitingForKey = true;
  }

  reset() {
    if (this.decoder && this.decoder.state !== "closed") this.decoder.close();
    this.decoder = null;
    this.waitingForKey = true;
    this.dropNewest();
  }

  dropNewest() {
    if (this.newest) this.newest.frame.close();
    this.newest = null;
  }

  // Hand over the newest decoded frame, or null. The caller closes it.
  take() {
    const newest = this.newest;
    this.newest = null;
    return newest;
  }
}

export class Video {
  constructor() {
    this.streams = { [STEREO]: new Stream(), [WRISTS.left]: new Stream(), [WRISTS.right]: new Stream() };
    this.looks = new Map();   // stereo frame number -> where the camera looked when it was taken
    this.decoded = 0;         // stereo frames decoded
    this.skips = 0;           // times a decoder fell behind and skipped to the next keyframe
  }

  stream(id) {
    return this.streams[id];
  }

  reset(id) {
    this.streams[id].reset();
    if (id === STEREO) this.looks.clear();
  }

  resetAll() {
    for (const id in this.streams) this.reset(Number(id));
  }

  // One VIDEO message from the robot.
  take(buffer) {
    if (typeof VideoDecoder === "undefined") {
      problem("This browser cannot decode video (no WebCodecs).");
      return;
    }
    const header = new DataView(buffer, 0, HEADER_BYTES);
    const key = header.getUint8(AT_KEY) === 1;
    const id = header.getUint16(AT_STREAM, true);
    const number = header.getUint32(AT_NUMBER, true);
    const unit = new Uint8Array(buffer, HEADER_BYTES);
    const stream = this.streams[id];
    if (!stream) return;

    if (stream.decoder && stream.decoder.decodeQueueSize > MAX_QUEUED_FRAMES && !stream.waitingForKey) {
      stream.waitingForKey = true;
      this.skips++;
    }
    if (stream.waitingForKey) {
      if (!key) return;
      const codec = codecOf(unit);
      if (!codec) return;
      stream.reset();
      stream.decoder = this.newDecoder(id, codec);
      stream.waitingForKey = false;
    }
    if (id === STEREO) this.rememberLook(number, header);
    stream.decoder.decode(new EncodedVideoChunk({ type: key ? "key" : "delta", timestamp: number, data: unit }));
  }

  rememberLook(number, header) {
    this.looks.set(number, { up: header.getFloat32(AT_UP, true), left: header.getFloat32(AT_LEFT, true) });
    if (this.looks.size > LOOKS_KEPT) this.looks.delete(this.looks.keys().next().value);
  }

  newDecoder(id, codec) {
    const stream = this.streams[id];
    const decoder = new VideoDecoder({
      output: (frame) => {
        const look = (id === STEREO && this.looks.get(frame.timestamp)) || { up: NaN, left: NaN };
        this.looks.delete(frame.timestamp);
        stream.dropNewest();
        stream.newest = { frame, up: look.up, left: look.left };
        if (id === STEREO && ++this.decoded % SIZE_SHOWN_EVERY === 0) {
          show("video", `${frame.displayWidth / 2} x ${frame.displayHeight} per eye`);
        }
      },
      error: (error) => {
        problem(`video decoder: ${error.message}`);
        stream.waitingForKey = true;
      },
    });
    decoder.configure({ codec, optimizeForLatency: true });
    return decoder;
  }
}
