# Turns one Quest controller into a velocity command for one hand.
#
# Adapted from DROID's `VRPolicy`: the grip is a clutch, and the hand moves by as much as the controller has moved since the grip was pressed.
import time
from dataclasses import dataclass, field
from typing import NamedTuple, Optional, Sequence

import numpy as np

from feather_teleop.control.maths import norm, smoothing_factor, to_matrix, to_vector
from feather_teleop.headsets import GRIP, HEAD, STICK_PRESS, TRIGGER

TRIGGER_OVERDRIVE = 1.5    # a full trigger asks for more than closed, so the gripper closes firmly (DROID's)
MIN_HEADING = 0.2          # below this the controller points too nearly up or down to give a heading
FILTER_RESET_AFTER = 0.2   # s without a reading before the pose filters restart
SPEED_ESTIMATE_HZ = 1.0    # Hz, cutoff of the filter that estimates the controller's speed
MIN_FILTER_DT = 1e-3       # s, floor on dt so the speed estimates cannot blow up
SAME_SAMPLE = 1e-3         # s, readings closer together than this are the same headset sample
MIN_SAMPLE_GAP = 0.002     # s, two samples closer than this give no usable velocity
MAX_SAMPLE_GAP = 0.060     # s, nor do two further apart than this


@dataclass
class SmoothSettings:
    # The One Euro filter (Casiez et al., 2012) on the controller's pose: its cutoff rises with the speed.
    min_hz: float = 1.5       # Hz, position cutoff while the controller is still
    per_m_s: float = 150.0    # Hz added per m/s
    turn_min_hz: float = 1.5  # Hz, orientation cutoff while the controller is still
    per_rad_s: float = 30.0   # Hz added per rad/s


@dataclass
class HoldSettings:
    # A bubble around a controller held still: inside it the hand does not move at all, so the operator's sway and drift never reach it.
    position: float = 0.02      # m, the bubble's radius around the controller's position (the operator's metres, before spatial_scale)
    rotation: float = 0.1       # rad, its radius around the orientation; it must span a whole sway, as it forms wherever the sway is
    still_speed: float = 0.02   # m/s, a low-passed speed above this counts as clearly moving
    still_rate: float = 0.2     # rad/s, the same for turning
    speed_hz: float = 1.0       # Hz, cutoff of the low-pass on the speed
    settle: float = 0.3         # s without clearly moving before the bubble forms
    release: float = 0.25       # s for the gap to the controller to close once it has left the bubble


@dataclass
class ControllerSettings:
    # How a controller becomes a hand command. The gains are DROID's.
    position_gain: float = 5.0    # 1/s
    rotation_gain: float = 2.0    # 1/s
    gripper_gain: float = 3.0     # 1/s
    # Robot x = -controller axis 2, y = -axis 1, z = -axis 3 (1-based); the 4 keeps the homogeneous row.
    axes: Sequence[int] = (-2, -1, -3, 4)
    up: Sequence[float] = (0.0, 1.0, 0.0)   # the headset frame's up axis
    forward_from: str = "controller"        # or "head": the way the worn headset faces
    absolute_orientation: bool = False      # the gripper points where the controller points
    align: float = 0.03           # s, the most the newest sample is extrapolated to the tick's time
    smooth: SmoothSettings = field(default_factory=SmoothSettings)
    hold: HoldSettings = field(default_factory=HoldSettings)

    @classmethod
    def worn(cls) -> "ControllerSettings":
        # Settings for the worn headset, whose poses arrive in the robot's axes already.
        return cls(axes=(1, 2, 3, 4), up=(0.0, 0.0, 1.0), forward_from=HEAD, absolute_orientation=True)


class Action(NamedTuple):
    # What the hand should do this tick.
    linear: np.ndarray            # fraction of the top hand speed per axis, in [-1, 1]
    angular: np.ndarray           # fraction of the top turning speed, as a rotation vector
    gripper: float                # closing speed, in [-1, 1]
    target_position: np.ndarray   # m, base frame: where the controller asks the hand to be
    target_rotation: np.ndarray   # rotation matrix: the hand's orientation there


@dataclass
class _FilterState:
    time: float
    raw_position: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    raw_rotation: np.ndarray
    rotation: np.ndarray
    angular_velocity: np.ndarray


@dataclass
class _Bubble:
    # One channel of the hold (position or orientation).
    center: Optional[object] = None                                  # the value followed while inside, or None: no bubble
    still: float = 0.0                                               # s without clearly moving
    offset: np.ndarray = field(default_factory=lambda: np.zeros(3))   # gap still to close since the controller left it


@dataclass
class _HoldState:
    time: float
    raw_position: np.ndarray
    raw_rotation: np.ndarray
    velocity: np.ndarray          # low-passed
    angular_velocity: np.ndarray
    place: _Bubble = field(default_factory=_Bubble)
    turn: _Bubble = field(default_factory=_Bubble)


def axis_map(axes: Sequence[int]) -> np.ndarray:
    # Build a signed permutation matrix: row i takes axis |axes[i]| (1-based), negated if axes[i] < 0.
    matrix = np.zeros((len(axes), len(axes)))
    for row, axis in enumerate(axes):
        matrix[row, abs(axis) - 1] = np.sign(axis)
    return matrix


class Controller:
    # One Quest controller driving one hand.
    #
    # Args:
    #     reader: The headset: anything whose `latest()` returns a `headsets.Reading`.
    #     side: "l" or "r".
    #     settings: The controller settings.
    #     mirror: True when the operator stands facing the robot.
    #     spatial_scale: Hand metres per controller metre.
    #     release_after: s; an older sample releases the clutch.
    #     clock: Returns the time in seconds.

    def __init__(self, reader, side: str, settings: ControllerSettings, mirror: bool, spatial_scale: float, release_after: float, clock=time.monotonic):
        self.reader = reader
        self.side = side
        self.settings = settings
        self.spatial_scale = spatial_scale
        self.release_after = release_after
        self.clock = clock
        self.up = np.asarray(settings.up, float) / np.linalg.norm(settings.up)
        self.forward_from = side if settings.forward_from == "controller" else settings.forward_from
        # With absolute orientation: the hand's rotation for an upright controller pointing forward. Set by the caller.
        self.hand_alignment: Optional[np.ndarray] = None

        x, y, z, w = settings.axes
        if mirror:
            # Facing the robot: positions turn half a turn about the vertical. Rotations are mirrored instead, so tipping the controller down still tips the hand down.
            self.position_map = axis_map([-x, -y, z, w])
            self.rotation_map = axis_map([x, -y, z])
        else:
            self.position_map = axis_map([x, y, z, w])
            self.rotation_map = self.position_map[:3, :3]

        self._forward_fixed = False       # until the first grip, "forward" follows the controller
        self._forward_requested = False
        self._to_forward_frame = np.eye(4)
        self._epoch = None
        self.reset()

    def reset(self) -> None:
        # Release the clutch and drop the origins and the filters' history.
        self.gripping = False        # the grip is held and the controller's pose is fresh
        self.grip_untracked = False  # the grip is held but the headset cannot see this controller
        self.stick_pressed = False
        self._pose: Optional[np.ndarray] = None
        self._head: Optional[np.ndarray] = None   # the head's newest pose, for operator()
        self._trigger = 0.0
        self._take_origin = True
        self._hand_origin = None
        self._controller_origin = None
        self._forget_history()
        if self.forward_from == HEAD:
            self.redefine_forward()   # the way the operator faces when the demonstration starts

    def _forget_history(self) -> None:
        self._samples = []           # the last two headset samples as (time, pose)
        self._filter: Optional[_FilterState] = None
        self._hold: Optional[_HoldState] = None

    def restart_origin(self) -> None:
        # Take new origins at the next action, so the hand continues from where it is.
        self._take_origin = True

    def redefine_forward(self) -> None:
        # Make "forward" the direction the controller (or the head) points at the next sample.
        self._forward_fixed = False
        self._forward_requested = True

    def update(self) -> None:
        # Read the newest headset sample. Call once per tick.
        reading = self.reader.latest()
        buttons = reading.buttons
        fresh = reading.age <= self.release_after
        has_pose = _usable(reading.poses, self.side) and _usable(reading.poses, self.forward_from)
        grip_held = bool(buttons.get(GRIP[self.side], False))
        # Buttons come by radio, so they keep arriving after the headset's cameras lose the controller.
        self.grip_untracked = fresh and grip_held and not has_pose
        if not (fresh and has_pose and GRIP[self.side] in buttons):
            self._release()
            return

        self._check_tracking_frame()
        if grip_held != self.gripping:
            self._take_origin = True
        self.gripping = grip_held
        self.stick_pressed = bool(buttons.get(STICK_PRESS[self.side], False))
        self._trigger = buttons.get(TRIGGER[self.side], (0.0,))[0]
        self._pose = self._aligned(np.asarray(reading.poses[self.side], float), reading.age)
        self._head = np.asarray(reading.poses[HEAD], float) if _usable(reading.poses, HEAD) else None
        if not self._forward_fixed:
            self._update_forward(reading.poses[self.forward_from])

    def _release(self) -> None:
        # Let go of the clutch: the stream is stale or the controller unseen.
        if self.gripping:
            self.gripping = False
            self._take_origin = True
        self.stick_pressed = False   # a stick held when the stream dropped must not stay held

    def _check_tracking_frame(self) -> None:
        # Start afresh when the worn headset is recentred: poses before and after are in different frames.
        epoch = getattr(self.reader, "epoch", None)
        if epoch != self._epoch:
            if self._epoch is not None:
                self.redefine_forward()
                self._forget_history()
            self._epoch = epoch

    def _update_forward(self, reference_pose) -> None:
        # Take "forward" from `reference_pose`, and fix it there on a grip or when asked to.
        self._to_forward_frame = np.linalg.inv(self._levelled(np.asarray(reference_pose, float)))
        if self.gripping or self._forward_requested:
            self._forward_fixed = True
            self._forward_requested = False
            self._take_origin = True   # the frame changed under the origins

    def action(self, hand_position, hand_rotation, gripper_closure: float) -> Action:
        # Return the command for a hand at the given pose. Call only while `gripping`.
        #
        # Args:
        #     hand_position: m, base frame.
        #     hand_rotation: Rotation matrix.
        #     gripper_closure: The gripper's commanded closure, 0 (open) to 1 (closed).
        hand_position = np.asarray(hand_position, float)
        hand_rotation = np.asarray(hand_rotation, float)
        position, rotation = self._controller_in_robot_axes()
        if self._take_origin:
            self._hand_origin = (hand_position, hand_rotation)
            self._controller_origin = (position, rotation)
            self._take_origin = False

        target_position, target_rotation = self._target(position, rotation)
        s = self.settings
        linear = (target_position - hand_position) * s.position_gain
        angular = to_vector(target_rotation @ hand_rotation.T) * s.rotation_gain
        closing = (self._trigger * TRIGGER_OVERDRIVE - gripper_closure) * s.gripper_gain
        return Action(_normalised(linear), _normalised(angular), float(np.clip(closing, -1.0, 1.0)), target_position, target_rotation)

    def _target(self, position: np.ndarray, rotation: np.ndarray):
        # Return the hand's target: its origin, moved by however the controller moved since its own origin.
        hand_position, hand_rotation = self._hand_origin
        controller_position, controller_rotation = self._controller_origin
        target_position = hand_position + (position - controller_position)
        if self.settings.absolute_orientation:
            return target_position, rotation @ self.hand_alignment
        return target_position, rotation @ controller_rotation.T @ hand_rotation

    def operator(self):
        # Return where the operator's head and this controller are and how the controller is turned, or None without both poses.
        #
        # In the forward frame on robot axes, in the operator's own metres: no scale, no filtering. For estimating the operator's elbow.
        if self._pose is None or self._head is None:
            return None
        head = (self.position_map @ (self._to_forward_frame @ self._head))[:3, 3]
        pose = self._to_forward_frame @ self._pose
        return head, (self.position_map @ pose)[:3, 3], self.rotation_map @ pose[:3, :3]

    def _controller_in_robot_axes(self):
        # Return the controller's filtered position (scaled) and rotation in the forward frame, on robot axes.
        pose = self._to_forward_frame @ self._pose
        position = (self.position_map @ pose)[:3, 3]
        # Only differences of this rotation are used, so a mirroring map still yields a proper rotation.
        rotation = self.rotation_map @ pose[:3, :3]
        position, rotation = self._filtered(position, rotation)
        position, rotation = self._held(position, rotation)
        return self.spatial_scale * position, rotation   # scaled last: the filters work in the operator's metres

    def _levelled(self, pose: np.ndarray) -> np.ndarray:
        # Return `pose` upright, with the same position and heading, so the room's up is the robot's up.
        rotation = pose[:3, :3]
        to_robot_axes = self.position_map[:3, :3]
        up = self.up
        handedness = np.linalg.det(to_robot_axes)   # -1 when the map mirrors
        ahead = self._horizontal(rotation @ to_robot_axes[0])
        if norm(ahead) < MIN_HEADING:   # pointing nearly straight up or down: take the heading from the side axis
            left = self._horizontal(rotation @ to_robot_axes[1])
            ahead = handedness * np.cross(left, up)
        ahead = ahead / norm(ahead)
        left = handedness * np.cross(up, ahead)
        level = pose.copy()
        level[:3, :3] = np.column_stack([ahead, left, up]) @ to_robot_axes
        return level

    def _horizontal(self, vector: np.ndarray) -> np.ndarray:
        # Return `vector` without its component along `up`.
        return vector - (vector @ self.up) * self.up

    def _aligned(self, pose: np.ndarray, age: float) -> np.ndarray:
        # Extrapolate the newest sample by its age (at most `align`) along the last two samples' velocity.
        #
        # The headset and the loop run at different rates, so without this an even motion arrives in uneven steps.
        now = self.clock()
        stamp = now - age
        if not self._samples or stamp - self._samples[-1][0] > SAME_SAMPLE:
            self._samples = (self._samples + [(stamp, pose)])[-2:]
        if len(self._samples) < 2:
            return pose
        (previous_time, previous), (newest_time, newest) = self._samples
        gap = newest_time - previous_time
        ahead = min(now - newest_time, self.settings.align)
        if not MIN_SAMPLE_GAP < gap < MAX_SAMPLE_GAP or ahead <= 0.0:
            return newest
        fraction = ahead / gap
        turned = to_vector(newest[:3, :3] @ previous[:3, :3].T)
        aligned = newest.copy()
        aligned[:3, 3] = newest[:3, 3] + (newest[:3, 3] - previous[:3, 3]) * fraction
        aligned[:3, :3] = to_matrix(turned * fraction) @ newest[:3, :3]
        return aligned

    def _filtered(self, position: np.ndarray, rotation: np.ndarray):
        # Smooth the pose (One Euro filter): hard while the controller is still, lightly while it moves.
        s = self.settings.smooth
        now = self.clock()
        state = self._filter
        if state is None or now - state.time > FILTER_RESET_AFTER:
            self._filter = _FilterState(time=now, raw_position=position, position=position, velocity=np.zeros(3), raw_rotation=rotation, rotation=rotation, angular_velocity=np.zeros(3))
            return position, rotation

        dt = max(now - state.time, MIN_FILTER_DT)
        speed_factor = smoothing_factor(SPEED_ESTIMATE_HZ, dt)

        velocity = _blend(state.velocity, (position - state.raw_position) / dt, speed_factor)
        position_cutoff = s.min_hz + s.per_m_s * norm(velocity)
        smooth_position = _blend(state.position, position, smoothing_factor(position_cutoff, dt))

        raw_angular_velocity = to_vector(rotation @ state.raw_rotation.T) / dt
        angular_velocity = _blend(state.angular_velocity, raw_angular_velocity, speed_factor)
        rotation_cutoff = s.turn_min_hz + s.per_rad_s * norm(angular_velocity)
        remaining = to_vector(rotation @ state.rotation.T)
        smooth_rotation = to_matrix(smoothing_factor(rotation_cutoff, dt) * remaining) @ state.rotation

        self._filter = _FilterState(time=now, raw_position=position, position=smooth_position, velocity=velocity, raw_rotation=rotation, rotation=smooth_rotation, angular_velocity=angular_velocity)
        return smooth_position, smooth_rotation

    def _held(self, position: np.ndarray, rotation: np.ndarray):
        # Return the pose to follow: once the controller has not clearly moved for `settle`, a bubble forms where the followed pose is, and the pose stays exactly there until the controller leaves it.
        #
        # Then it is followed from the bubble's edge, without a jump, and the gap closes over `release`. Position and orientation have separate bubbles, so turning the controller in place holds the position. The cost: a motion slower than `still_speed` comes out in steps of the bubble.
        h = self.settings.hold
        now = self.clock()
        state = self._hold
        if state is None or now - state.time > FILTER_RESET_AFTER:
            self._hold = _HoldState(time=now, raw_position=position, raw_rotation=rotation, velocity=np.zeros(3), angular_velocity=np.zeros(3))
            return position, rotation

        dt = max(now - state.time, MIN_FILTER_DT)
        factor = smoothing_factor(h.speed_hz, dt)
        state.velocity = _blend(state.velocity, (position - state.raw_position) / dt, factor)
        turn = to_vector(rotation @ state.raw_rotation.T) / dt
        state.angular_velocity = _blend(state.angular_velocity, turn, factor)
        state.time, state.raw_position, state.raw_rotation = now, position, rotation

        # A sway goes back and forth, so its low-passed speed stays low; a real motion keeps its direction.
        position = _bubble(state.place, position, norm(state.velocity) >= h.still_speed, dt, h.position, h, _apart, _back)
        rotation = _bubble(state.turn, rotation, norm(state.angular_velocity) >= h.still_rate, dt, h.rotation, h, _turned, _unturned)
        return position, rotation


def _blend(previous, value, factor: float):
    # One step of a first-order low-pass from `previous` toward `value`.
    return previous + factor * (value - previous)


def _bubble(bubble: _Bubble, value, moving: bool, dt: float, radius: float, settings: HoldSettings, gap, shifted):
    # Return the value to follow for the controller's `value`, one tick of one channel of the hold.
    #
    # `gap(value, center)` is how far `value` is from `center`, as a 3-vector; `shifted(value, offset)` is `value` moved back by `offset`.
    if bubble.center is not None:
        out = gap(value, bubble.center)
        if norm(out) <= radius:
            return bubble.center
        # Left it: followed from the edge, the motion past the edge comes through, and the radius is still to close.
        bubble.center, bubble.still, bubble.offset = None, 0.0, out * (radius / norm(out))
    else:
        far = norm(bubble.offset)
        if far > 0.0:   # the gap closes at one radius per `release`
            bubble.offset = bubble.offset * (max(0.0, far - radius * dt / settings.release) / far)
    bubble.still = 0.0 if moving else bubble.still + dt
    followed = shifted(value, bubble.offset)
    if bubble.still >= settings.settle:
        bubble.center, bubble.offset = followed, np.zeros(3)
    return followed


def _apart(position: np.ndarray, center: np.ndarray) -> np.ndarray:
    return position - center


def _back(position: np.ndarray, offset: np.ndarray) -> np.ndarray:
    return position - offset


def _turned(rotation: np.ndarray, center: np.ndarray) -> np.ndarray:
    return to_vector(rotation @ center.T)


def _unturned(rotation: np.ndarray, offset: np.ndarray) -> np.ndarray:
    return to_matrix(-offset) @ rotation


def _normalised(vector: np.ndarray) -> np.ndarray:
    # Scale `vector` down to length 1 if it is longer.
    length = norm(vector)
    return vector / length if length > 1.0 else vector


def _usable(poses: dict, name: str) -> bool:
    # Whether `poses` has a pose for `name` made of numbers. A NaN would stick in the filters.
    pose = poses.get(name)
    return pose is not None and bool(np.isfinite(pose).all())
