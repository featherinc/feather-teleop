// The debug readout (quest.yaml's `worn.debug`): a few lines fixed to the head, lower left, under the left wrist camera's picture.
//
// Measured here: frame rates, round trip, tracking, links. From the robot: its loop rate and current faults.
const PIXELS = [512, 256];   // the readout's texture
const EVERY_MS = 500;        // between two redraws of it
export const ASIDE = -0.27, BELOW = 0.40, WIDTH = 0.30;   // m: left of and below the middle of the view, and across
export const ASPECT = PIXELS[1] / PIXELS[0];
const LINE_PX = 30, MARGIN_PX = 10, MAX_FAULTS = 4;

export class Readout {
  constructor(drawing) {
    this.drawing = drawing;
    this.texture = drawing.texture();
    this.canvas = typeof OffscreenCanvas === "undefined" ? document.createElement("canvas") : new OffscreenCanvas(...PIXELS);
    [this.canvas.width, this.canvas.height] = PIXELS;
    this.at = null;        // ms, when it was last redrawn
    this.frames = 0;       // drawn since then
    this.decodedAt = 0;    // the video's decoded count then
    this.hands = { left: "-", right: "-" };
  }

  // Count one drawn frame, and redraw the texture every EVERY_MS. `debug`: the robot's side, `links`: {poses, video} states, `video`: the Video.
  update(now, debug, links, video) {
    this.frames++;
    if (this.at === null) {
      this.at = now;
      this.decodedAt = video.decoded;
      return;
    }
    if (now - this.at < EVERY_MS) return;
    const seconds = (now - this.at) / 1000;
    const fps = this.frames / seconds, videoFps = (video.decoded - this.decodedAt) / seconds;
    this.at = now;
    this.frames = 0;
    this.decodedAt = video.decoded;

    const echo = debug.echo;   // [the page's clock in the newest pose the robot had, ms since it arrived there]
    const ping = echo ? performance.now() - echo[0] - echo[1] : NaN;
    const lines = [`display ${fps.toFixed(0)} fps   video ${videoFps.toFixed(0)} fps   skips ${video.skips}`, `ping ${Number.isFinite(ping) ? ping.toFixed(0) + " ms" : "-"}   robot loop ${debug.hz ?? "-"} Hz`, `left controller: ${this.hands.left}   right: ${this.hands.right}`, `links: poses ${links.poses}, video ${links.video === "none" ? "on the poses'" : links.video}`];
    const faults = debug.faults && debug.faults.length ? debug.faults : null;
    const pen = this.canvas.getContext("2d");
    pen.fillStyle = "#101010";
    pen.fillRect(0, 0, PIXELS[0], PIXELS[1]);
    pen.font = "22px monospace";
    pen.textBaseline = "top";
    pen.fillStyle = "#e8e8e0";
    lines.forEach((line, i) => pen.fillText(line, MARGIN_PX, MARGIN_PX + LINE_PX * i, PIXELS[0] - 2 * MARGIN_PX));
    pen.fillStyle = faults ? "#ff8a5c" : "#7fd08a";
    (faults || ["no faults"]).slice(0, MAX_FAULTS).forEach((line, i) => pen.fillText(line, MARGIN_PX, MARGIN_PX + LINE_PX * (lines.length + i), PIXELS[0] - 2 * MARGIN_PX));
    const gl = this.drawing.gl;
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, this.canvas);
  }

  draw(viewport, projection, view, head) {
    this.drawing.drawPanel(viewport, projection, view, head, this.texture, ASIDE, BELOW, WIDTH, ASPECT);
  }
}
