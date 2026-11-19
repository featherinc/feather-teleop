// The worn headset's page. Every displayed frame it sends the head's and both controllers' poses and inputs to the robot. While a demonstration runs it shows the robot's stereo camera, a wrist camera when asked and, if the robot asks, the debug readout; between demonstrations it shows the room.
import { Readout } from "./debug.js";
import { Drawing, lookOf } from "./draw.js";
import { BUTTON_LOWER, BUTTON_STICK, BUTTON_UPPER, Link, VideoLink } from "./link.js";
import { problem, show } from "./page.js";
import { STEREO, Video, WRISTS } from "./video.js";

// The "xr-standard" gamepad's buttons and stick axes.
const TRIGGER = 0, SQUEEZE = 1, STICK = 3, LOWER = 4, UPPER = 5;
const STICK_X = 2, STICK_Y = 3;

let status = { phase: "ready", camera: null, overlay: { left: null, right: null }, video_port: null, debug: null };
const video = new Video();
const link = new Link(onStatus, (buffer) => video.take(buffer), () => video.resetAll());
const videoLink = new VideoLink((buffer) => video.take(buffer), () => video.resetAll());
let drawing = null;
let readout = null;

function onStatus(newStatus) {
  status = newStatus;
  show("phase", status.phase === "active" ? "running" : "none (A starts one)");
  if (!status.camera) {
    video.reset(STEREO);
    show("video", "no picture");
  }
  for (const side in WRISTS) {
    if (!status.overlay[side]) video.reset(WRISTS[side]);
  }
  videoLink.want(status.video_port ?? null);
}

function inputOf(gamepad) {
  const value = (i) => (gamepad.buttons[i] ? gamepad.buttons[i].value : 0);
  const pressed = (i) => Boolean(gamepad.buttons[i] && gamepad.buttons[i].pressed);
  const hasStick = gamepad.axes.length > STICK_Y;
  return {
    bits: (pressed(STICK) ? BUTTON_STICK : 0) | (pressed(LOWER) ? BUTTON_LOWER : 0) | (pressed(UPPER) ? BUTTON_UPPER : 0),
    trigger: value(TRIGGER),
    squeeze: value(SQUEEZE),
    x: hasStick ? gamepad.axes[STICK_X] : 0,
    y: hasStick ? -gamepad.axes[STICK_Y] : 0,   // the gamepad's y is + down; the robot's + up
  };
}

function plain(transform) {
  const { position: p, orientation: q } = transform;
  return { position: [p.x, p.y, p.z], orientation: [q.x, q.y, q.z, q.w] };
}

// The head's pose and each tracked controller's: the controller's position is its grip (the hand), its orientation the pointing ray's (where it points).
function posesAndInputs(session, frame, space, viewer) {
  const poses = { head: viewer && !viewer.emulatedPosition ? plain(viewer.transform) : null, left: null, right: null };
  const inputs = {};
  for (const source of session.inputSources) {
    const side = source.handedness;
    if (!source.gripSpace || !source.gamepad || !(side === "left" || side === "right")) continue;
    const grip = frame.getPose(source.gripSpace, space);
    const ray = frame.getPose(source.targetRaySpace, space);
    if (grip && ray && !grip.emulatedPosition && !ray.emulatedPosition) {
      poses[side] = { position: plain(grip.transform).position, orientation: plain(ray.transform).orientation };
    }
    inputs[side] = inputOf(source.gamepad);
  }
  return { poses, inputs };
}

async function requestSession() {
  for (const mode of ["immersive-ar", "immersive-vr"]) {   // ar shows the room wherever nothing is drawn
    if (await navigator.xr.isSessionSupported(mode)) {
      return navigator.xr.requestSession(mode, { requiredFeatures: ["local-floor"] });
    }
  }
  return null;
}

async function enter() {
  problem("");
  const session = await requestSession();
  if (!session) {
    problem("This headset's browser offers no immersive session.");
    return;
  }
  if (!drawing) {
    drawing = new Drawing(document.getElementById("xr"));
    readout = new Readout(drawing);
  }
  const gl = drawing.gl;
  link.newTrackingFrame();
  drawing.forget();
  await gl.makeXRCompatible();
  // At the display's own resolution: the default renders smaller and scales up, which blurs the picture.
  const layer = new XRWebGLLayer(session, gl, { alpha: true, antialias: false, framebufferScaleFactor: XRWebGLLayer.getNativeFramebufferScaleFactor(session) });
  session.updateRenderState({ baseLayer: layer });
  const space = await session.requestReferenceSpace("local-floor");
  space.addEventListener("reset", () => { link.newTrackingFrame(); drawing.forget(); });   // recentred
  session.addEventListener("end", () => { document.getElementById("enter").disabled = false; drawing.forget(); });
  document.getElementById("enter").disabled = true;

  const onFrame = (now, frame) => {
    session.requestAnimationFrame(onFrame);
    const viewer = frame.getViewerPose(space);
    gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
    drawing.clear(0);
    // With the system menu up, or the headset off the head, the buttons aren't meant for the robot: send nothing, so the arms hold.
    if (session.visibilityState !== "visible") return;
    const { poses, inputs } = posesAndInputs(session, frame, space, viewer);
    link.sendPose(poses, inputs);
    for (const side of ["left", "right"]) readout.hands[side] = poses[side] ? "tracked" : inputs[side] ? "NOT TRACKED" : "off";

    if (!viewer || status.phase !== "active" || (!status.camera && !status.debug)) {
      drawing.forget();
      return;
    }
    if (status.camera) {
      drawing.update(video, status.overlay, now, lookOf(viewer.transform.orientation));
    } else {
      drawing.forget();
    }
    if (status.debug) readout.update(now, status.debug, { poses: link.state(), video: videoLink.state() }, video);
    if (!drawing.hasPicture && !status.debug) return;
    drawing.clear(1);   // black around the picture, so it reads as a scene rather than a screen in the room
    for (const view of viewer.views) {
      const viewport = layer.getViewport(view);
      const inverse = view.transform.inverse.matrix;
      if (drawing.hasPicture) {
        drawing.drawPicture(viewport, view.projectionMatrix, inverse, view.transform.position, view.eye === "right", status.camera);
        drawing.drawOverlays(viewport, view.projectionMatrix, inverse, viewer.transform.matrix, status.overlay);
      }
      if (status.debug) readout.draw(viewport, view.projectionMatrix, inverse, viewer.transform.matrix);
    }
  };
  session.requestAnimationFrame(onFrame);
}

link.connect();
if (!window.isSecureContext) {
  problem("Not a secure page: the browser grants an immersive session only over https or from localhost.");
} else if (!navigator.xr) {
  problem("This browser has no WebXR: open the page in the headset's browser.");
} else {
  const button = document.getElementById("enter");
  button.disabled = false;
  button.onclick = () => enter().catch((error) => problem(String(error)));
}
