// Drawing in the headset: the stereo picture, and the wrist cameras' pictures.
import { STEREO, WRISTS } from "./video.js";

const PICTURE_DISTANCE = 2.0;   // m from each eye to the picture; only its direction matters
const LOOK_HZ = 8.0;            // the picture's direction follows the camera's through a low-pass at this cutoff
const MAX_LOOK_STEP = 0.1;      // s, the longest step of that low-pass
const OVERLAY_DISTANCE = 0.9;   // m ahead of the head: a wrist camera's picture
const OVERLAY_WIDTH = 0.30;     // m
const OVERLAY_ASIDE = 0.27;     // m right or left of the middle of the view
const OVERLAY_BELOW = 0.20;     // m below it

// The picture: each corner is a direction from the camera, as the tangents of its angles.
const PICTURE_VERTEX = `#version 300 es
in vec2 corner;
uniform mat4 projection, view;
uniform mat3 turn;        // the camera's orientation in the tracking frame
uniform vec3 eye;         // centred on each eye, so only the picture's direction shows
uniform vec4 model;       // fx, fy, cx, cy over the picture's width and height
uniform float half_at;    // 0 for the left eye's half of the texture, 0.5 for the right's
uniform float inset;      // half a pixel, so sampling never reaches the other eye's half
out vec2 uv;
void main() {
  uv = vec2(half_at + 0.5 * clamp(corner.x * model.x + model.z, inset, 1.0 - inset), model.w - corner.y * model.y);
  gl_Position = projection * view * vec4(eye + turn * vec3(corner, -1.0) * ${PICTURE_DISTANCE.toFixed(1)}, 1.0);
}`;

// A wrist camera's picture: a flat rectangle placed by `place`.
const OVERLAY_VERTEX = `#version 300 es
in vec2 corner;           // 0..1 across and up
uniform mat4 projection, view, place;
uniform vec2 size;        // m
out vec2 uv;
void main() {
  uv = vec2(corner.x, 1.0 - corner.y);
  gl_Position = projection * view * place * vec4((corner - 0.5) * size, 0.0, 1.0);
}`;

const FRAGMENT = `#version 300 es
precision mediump float;
in vec2 uv;
uniform sampler2D picture;
out vec4 colour;
void main() { colour = vec4(texture(picture, uv).rgb, 1.0); }`;

// The camera's axes in the tracking frame: turned by `left` about y, then by `up` about x (column-major).
function lookMatrix(up, left) {
  const cu = Math.cos(up), su = Math.sin(up), cl = Math.cos(left), sl = Math.sin(left);
  return new Float32Array([cl, 0, -sl, sl * su, cu, cl * su, sl * cu, -su, cl * cu]);
}

// Where a pose's -z axis points, as {up, left}, from its quaternion.
export function lookOf(q) {
  const x = -2 * (q.x * q.z + q.w * q.y);
  const y = -2 * (q.y * q.z - q.w * q.x);
  const z = -(1 - 2 * (q.x * q.x + q.y * q.y));
  return { up: Math.atan2(y, Math.hypot(x, z)), left: Math.atan2(-x, -z) };
}

function shortestTurn(from, to) {
  return Math.atan2(Math.sin(to - from), Math.cos(to - from));
}

export class Drawing {
  constructor(canvas) {
    const gl = canvas.getContext("webgl2", { xrCompatible: true, alpha: true, antialias: false });
    if (!gl) throw new Error("no WebGL 2");
    this.gl = gl;

    this.picture = this.program(PICTURE_VERTEX, ["projection", "view", "turn", "eye", "model", "half_at", "inset"]);
    this.picture.texture = this.texture();

    this.overlay = this.program(OVERLAY_VERTEX, ["projection", "view", "place", "size"]);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([0, 0, 1, 0, 0, 1, 1, 1]), gl.STATIC_DRAW);
    this.overlay.textures = { left: this.texture(), right: this.texture() };
    this.overlay.shown = { left: false, right: false };

    this.forget();
  }

  // Forget the picture and its direction: the next frame starts where its own camera looked.
  forget() {
    this.hasPicture = false;
    this.look = null;     // the picture's direction {up, left}, low-passed
    this.lookAt = 0;      // ms, when it was last moved
    this.aim = null;      // where the newest frame's camera looked, or null if the robot did not say
  }

  program(vertex, uniforms) {
    const gl = this.gl;
    const program = gl.createProgram();
    for (const [kind, source] of [[gl.VERTEX_SHADER, vertex], [gl.FRAGMENT_SHADER, FRAGMENT]]) {
      const shader = gl.createShader(kind);
      gl.shaderSource(shader, source);
      gl.compileShader(shader);
      if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
      gl.attachShader(program, shader);
    }
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));

    // One attribute, `corner`: two floats per vertex, from a buffer left bound for the caller to fill.
    const array = gl.createVertexArray();
    const corners = gl.createBuffer();
    gl.bindVertexArray(array);
    gl.bindBuffer(gl.ARRAY_BUFFER, corners);
    const corner = gl.getAttribLocation(program, "corner");
    gl.enableVertexAttribArray(corner);
    gl.vertexAttribPointer(corner, 2, gl.FLOAT, false, 0, 0);

    const locations = Object.fromEntries([...uniforms, "picture"].map((name) => [name, gl.getUniformLocation(program, name)]));
    return { program, array, corners, uniforms: locations };
  }

  texture() {
    const gl = this.gl;
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    return texture;
  }

  upload(texture, frame) {
    const gl = this.gl;
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, frame);
    frame.close();
  }

  // Upload the newest frames and step the picture toward where its camera looked (in front of the head if unknown).
  update(video, overlays, now, headLook) {
    const newest = video.stream(STEREO).take();
    if (newest) {
      this.aim = Number.isNaN(newest.up) ? null : { up: newest.up, left: newest.left };
      this.upload(this.picture.texture, newest.frame);
      this.hasPicture = true;
    }
    const toward = this.aim || headLook;
    if (!this.look || !this.aim) {
      this.look = { ...toward };
    } else {
      const dt = Math.min(MAX_LOOK_STEP, (now - this.lookAt) / 1000);
      const share = 1 - Math.exp(-2 * Math.PI * LOOK_HZ * dt);
      this.look = { up: this.look.up + share * (toward.up - this.look.up), left: this.look.left + share * shortestTurn(this.look.left, toward.left) };
    }
    this.lookAt = now;

    for (const side in WRISTS) {
      if (!overlays[side]) {
        this.overlay.shown[side] = false;
        continue;
      }
      const frame = video.stream(WRISTS[side]).take();
      if (frame) {
        this.upload(this.overlay.textures[side], frame.frame);
        this.overlay.shown[side] = true;
      }
    }
  }

  clear(alpha) {
    this.gl.clearColor(0, 0, 0, alpha);
    this.gl.clear(this.gl.COLOR_BUFFER_BIT);
  }

  // One eye's view of the stereo picture. `camera` is its pinhole model {fx, fy, cx, cy, width, height}.
  drawPicture(viewport, projection, view, eye, right, camera) {
    const gl = this.gl;
    const { program, array, corners, uniforms: u, texture } = this.picture;
    const c = camera;
    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(program);
    gl.bindVertexArray(array);
    gl.uniformMatrix4fv(u.projection, false, projection);
    gl.uniformMatrix4fv(u.view, false, view);
    gl.uniformMatrix3fv(u.turn, false, lookMatrix(this.look.up, this.look.left));
    gl.uniform3f(u.eye, eye.x, eye.y, eye.z);
    gl.uniform4f(u.model, c.fx / c.width, c.fy / c.height, c.cx / c.width, c.cy / c.height);
    gl.uniform1f(u.half_at, right ? 0.5 : 0.0);
    gl.uniform1f(u.inset, 0.5 / c.width);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.uniform1i(u.picture, 0);
    const left = -c.cx / c.fx, rightEdge = (c.width - c.cx) / c.fx;
    const bottom = (c.cy - c.height) / c.fy, top = c.cy / c.fy;
    gl.bindBuffer(gl.ARRAY_BUFFER, corners);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([left, bottom, rightEdge, bottom, left, top, rightEdge, top]), gl.DYNAMIC_DRAW);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
  }

  // Each shown wrist camera in the lower corner of its side, fixed to the head (`head`: its pose, column-major). `overlays[side]` is that picture's {width, height}.
  drawOverlays(viewport, projection, view, head, overlays) {
    for (const side in WRISTS) {
      const size = overlays[side];
      if (!size || !this.overlay.shown[side]) continue;
      const aside = side === "right" ? OVERLAY_ASIDE : -OVERLAY_ASIDE;
      this.drawPanel(viewport, projection, view, head, this.overlay.textures[side], aside, OVERLAY_BELOW, OVERLAY_WIDTH, size.height / size.width);
    }
  }

  // A flat picture fixed to the head: `aside` m right (negative: left), `below` m down, OVERLAY_DISTANCE ahead; `width` m across, `aspect` its height over its width.
  drawPanel(viewport, projection, view, head, texture, aside, below, width, aspect) {
    const gl = this.gl;
    const { program, array, uniforms: u } = this.overlay;
    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(program);
    gl.bindVertexArray(array);
    gl.uniformMatrix4fv(u.projection, false, projection);
    gl.uniformMatrix4fv(u.view, false, view);
    gl.activeTexture(gl.TEXTURE0);
    gl.uniform1i(u.picture, 0);
    const place = new Float32Array(head);   // the head's axes, its origin moved to the picture's centre
    for (let row = 0; row < 3; row++) {
      const right = head[row], up = head[4 + row], back = head[8 + row];
      place[12 + row] = head[12 + row] + aside * right - below * up - OVERLAY_DISTANCE * back;
    }
    gl.uniformMatrix4fv(u.place, false, place);
    gl.uniform2f(u.size, width, width * aspect);
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
  }
}
