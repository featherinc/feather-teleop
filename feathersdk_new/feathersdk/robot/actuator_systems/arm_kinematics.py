"""Cartesian control of one arm: a small hand motion in, a small joint motion out, on the robot's URDF.

`ArmIK` is a damped least-squares solver with joint bounds, after DROID's (droid/robot_ik): hand and joint speeds
are capped, a joint near its limit is slowed rather than the whole motion rejected, and the redundant joint is
pulled weakly toward a rest posture. `Guards` steer before the collision check refuses: the elbow is pushed away
from the body and the other arm in the null space of the hand's motion, and the hand's motion loses the components
that would stretch the arm straight, pull the wrist behind its shoulder or bring the two wrists together.

For streaming a hand pose (teleoperation, a policy): each step, ask `ArmIK.step()` for the joint motion and send
the result as a follow target (`Robstride7MArm.set_follow_target()`). Angles are in the URDF's joint frame
(`JointDescription.urdf_direction` maps them to the motors'); positions in the robot's base frame (x forward,
y left, z up).
"""
import math
from dataclasses import dataclass, field
from typing import List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

ELBOW = 3                     # index of the elbow among an arm's seven joints
TINY = 1e-9                   # lengths below this are treated as zero
SOLVER_PASSES = 8             # active-set iterations; seven joints settle in fewer
BOUND_TOLERANCE = 1e-12
NULLSPACE_RCOND = 1e-2        # singular values below this fraction of the largest are treated as zero
MAX_STANDOFF_FRACTION = 0.45  # of a joint's range, so a narrow joint keeps a usable middle
POSES_KEPT = 8                # postures whose tool pose is remembered; a tick asks about a handful
FADE_DISTANCE = 0.05          # m over which a distance guard fades in
FADE_EXTENSION = 0.05         # fraction of full reach over which the stretch guard fades in
POSTURE_FADE = 0.5            # the posture pull is fully off once the arm is this fraction of body_clear into it
ELBOW_RCOND = 1e-2            # cutoff for the pseudo-inverse of the elbow's Jacobian
# Fractions along the forearm (0 at the elbow, 1 at the wrist) that are checked against the body. A
# point shares the elbow's motion in proportion to how close it is to the elbow. The upper arm is
# left out because its top is at the shoulder, inside the body's footprint wherever the elbow is.
FOREARM_CHECKS = (0.0, 1.0 / 3.0, 2.0 / 3.0)
# Out of a footprint through its low x, high x, low y and high y side.
OUT_OF_BOX = np.array([[-1.0, 0, 0], [1.0, 0, 0], [0, -1.0, 0], [0, 1.0, 0]])


def _norm(v: np.ndarray) -> float:
    """Length of a 1-D vector, without numpy's input checks (a hot path)."""
    return math.sqrt(v @ v)


def _clamp(x: float, low: float, high: float) -> float:
    return low if x < low else high if x > high else x


def _skew(v: np.ndarray) -> np.ndarray:
    """The matrix that takes the cross product with `v`."""
    x, y, z = v.tolist()
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


@dataclass
class IKSettings:
    """The solver's tuning."""
    max_lin_vel: float = 1.125       # m/s
    max_rot_vel: float = 2.25        # rad/s
    max_joint_vel: float = 3.0       # rad/s; each motor's own limit also applies
    limit_standoff: float = 0.1      # rad kept between a joint and its limit
    limit_scale: float = 0.95        # fraction of the distance to the standoff one step may close
    posture_gain: float = 0.75       # 1/s, null-space pull toward the rest posture
    regularization: float = 0.0003   # penalty on joint motion
    continuity: float = 0.002        # penalty on a joint changing speed between steps
    # m per rad. Low values keep the tool point in place when the wrist cannot reach the asked orientation.
    rot_weight: float = 0.1


def bounded_least_squares(H: np.ndarray, g: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    """Minimise 0.5 x'Hx - g'x subject to low <= x <= high, for positive definite H.

    An active-set method. Each pass solves for the free variables, pins those that left their bounds,
    and frees a pinned variable whose gradient points back inside. Most steps never touch a bound, so the
    first pass is the plain solve and usually the answer.
    """
    under = low - BOUND_TOLERANCE
    over = high + BOUND_TOLERANCE
    x = np.linalg.solve(H, g)
    pinned = (x < under) | (x > over)
    if not pinned.any():
        return x
    x = np.clip(x, low, high)
    has_room = low < high
    for _ in range(SOLVER_PASSES):
        free = ~pinned
        if free.any():
            rhs = g[free] - H[np.ix_(free, pinned)] @ x[pinned]
            x[free] = np.linalg.solve(H[np.ix_(free, free)], rhs)
        outside = free & ((x < under) | (x > over))
        x = np.clip(x, low, high)
        gradient = H @ x - g
        wants_in = ((x <= low) & (gradient < -BOUND_TOLERANCE)) | ((x >= high) & (gradient > BOUND_TOLERANCE))
        release = pinned & has_room & wants_in
        if not outside.any() and not release.any():
            break
        pinned = (pinned | outside) & ~release
    return np.clip(x, low, high)


class ArmIK:
    """Kinematics of one arm inside a Pinocchio model of the whole robot.

    Args:
        pin: The pinocchio module. The SDK imports it lazily, so its handle is passed in.
        model: The robot's Pinocchio model.
        q_indices, v_indices: Where the arm's joints sit in the model's position and velocity vectors.
        hand_joint: Name of the arm's last joint. Its frame is the hand.
        lower, upper: Each joint's commandable range (rad).
        max_velocity: Each joint's speed limit (rad/s).
        settings: The solver's settings.
        tool_offset: The tool point in the hand joint's frame (m).
        rest_posture: The posture the null-space term pulls toward (rad).
    """

    def __init__(self, pin, model, q_indices, v_indices, hand_joint: str, lower, upper, max_velocity,
                 settings: IKSettings, tool_offset: Sequence[float], rest_posture: Sequence[float]):
        self.pin = pin
        self.model = model
        self.data = model.createData()        # forward kinematics and Jacobians, for the solver and the guards
        self._pose_data = model.createData()  # forward kinematics alone, for `pose()`
        self._poses = {}                      # recent tool poses, by the joint angles' bytes
        self.q_indices = np.asarray(q_indices)
        self.v_indices = np.asarray(v_indices)
        self.hand_joint_id = model.getJointId(hand_joint)
        if self.hand_joint_id >= model.njoints:
            raise ValueError(f"the URDF has no joint named {hand_joint!r}")
        self.lower = np.asarray(lower, float)
        self.upper = np.asarray(upper, float)
        self.max_velocity = np.asarray(max_velocity, float)
        self.settings = settings
        self.tool_offset = np.asarray(tool_offset, float)
        self.rest_posture = np.asarray(rest_posture, float)
        self._full_q = pin.neutral(model)   # the whole robot; joints outside the arm keep these values
        self._computed_for: Optional[bytes] = None   # the joint angles the model's data holds, as bytes
        self._last_rate: Optional[np.ndarray] = None   # rad/s of the previous step, for `continuity`

        # What every step needs and no step changes.
        joint_count = len(self.q_indices)
        s = settings
        self._frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        self._identity = np.eye(joint_count)
        self._weights = np.array([1.0, 1.0, 1.0, s.rot_weight, s.rot_weight, s.rot_weight])
        self._damping = s.regularization * self._identity
        self._smoothing = s.continuity * self._identity
        self._joint_speed = np.minimum(s.max_joint_vel, self.max_velocity)
        standoff = np.minimum(s.limit_standoff, MAX_STANDOFF_FRACTION * (self.upper - self.lower))
        self._lowest = self.lower + standoff
        self._highest = self.upper - standoff

    @classmethod
    def for_arm(cls, arm, tool_offset: Sequence[float] = (0.0, 0.0, 0.0), rest_posture: Optional[Sequence[float]] = None,
                settings: Optional["IKSettings"] = None) -> "ArmIK":
        """The solver of an SDK arm (`Robstride7MArm`), on its own copy of the robot's Pinocchio model.

        Args:
            arm: The SDK arm.
            tool_offset: The tool point in the frame of the arm's last joint (m).
            rest_posture: Joint angles the redundant joint is pulled toward (rad, URDF frame, in the arm's joint
                order); zeros by default.
            settings: The solver's settings.
        """
        joints = arm.describe().joints
        # A range is given in the motor's frame; where the URDF's runs the other way, its ends swap.
        direction = np.array([joint.urdf_direction for joint in joints], float)
        ends = np.array([joint.position for joint in joints], float) * direction[:, None]
        return cls(arm.pin_import, arm.pin_model.copy(), arm.pin_arm_q_inds, arm.pin_arm_v_inds, joints[-1].urdf_joint,
                   ends.min(axis=1), ends.max(axis=1), [joint.velocity for joint in joints], settings or IKSettings(),
                   tool_offset, np.zeros(len(joints)) if rest_posture is None else rest_posture)

    def follow_torso(self, torso) -> None:
        """Place the torso's lift joint where the SDK torso is, so poses come out in the robot's base frame."""
        self.set_joint(torso.describe().joint, torso.urdf_position())

    def set_joint(self, name: str, position: float) -> None:
        """Set a joint outside the arm (the torso lift), so poses come out in the robot's base frame."""
        joint = self.model.getJointId(name)
        if joint >= self.model.njoints:
            raise ValueError(f"the URDF has no joint named {name!r}")
        self._full_q[self.model.joints[joint].idx_q] = position
        self._computed_for = None
        self._poses.clear()

    def _compute(self, q: np.ndarray) -> None:
        """Run forward kinematics and Jacobians for `q`, unless they are already computed for it.

        Several questions are asked about one posture each tick; comparing the bytes is the cheapest exact test.
        """
        key = q.tobytes()
        if key == self._computed_for:
            return
        self._full_q[self.q_indices] = q
        self.pin.computeJointJacobians(self.model, self.data, self._full_q)
        self._computed_for = key

    def joint_point(self, q, name: str) -> Tuple[np.ndarray, np.ndarray]:
        """Return the position (m, base frame) of a joint's origin and its 3x7 position Jacobian."""
        return self._point(q, self.model.getJointId(name))

    def wrist_point(self, q) -> Tuple[np.ndarray, np.ndarray]:
        """Return the same for the hand joint, which sits at the wrist."""
        return self._point(q, self.hand_joint_id)

    def _point(self, q, joint: int) -> Tuple[np.ndarray, np.ndarray]:
        self._compute(np.asarray(q, float))
        frame = self._frame
        jacobian = self.pin.getJointJacobian(self.model, self.data, joint, frame)[:3, self.v_indices]
        return self.data.oMi[joint].translation.copy(), jacobian.copy()

    def pose(self, q) -> Tuple[np.ndarray, np.ndarray]:
        """Return the tool point's position (m) and the hand's rotation matrix, in the base frame. Read-only.

        Forward kinematics alone, remembered for the last few postures: a tick asks about the target, the
        measured and the sent posture, each more than once.
        """
        q = np.asarray(q, float)
        key = q.tobytes()
        pose = self._poses.get(key)
        if pose is not None:
            return pose
        if key == self._computed_for:
            hand = self.data.oMi[self.hand_joint_id]
        else:
            self._full_q[self.q_indices] = q
            self.pin.forwardKinematics(self.model, self._pose_data, self._full_q)
            hand = self._pose_data.oMi[self.hand_joint_id]
        rotation = hand.rotation.copy()
        position = hand.translation + rotation @ self.tool_offset
        position.setflags(write=False)
        rotation.setflags(write=False)
        if len(self._poses) >= POSES_KEPT:
            del self._poses[next(iter(self._poses))]
        self._poses[key] = (position, rotation)
        return position, rotation

    def jacobian(self, q) -> np.ndarray:
        """Return the 6x7 Jacobian of the tool point (rows 0-2) and the hand's rotation (rows 3-5)."""
        self._compute(np.asarray(q, float))
        frame = self._frame
        J = self.pin.getJointJacobian(self.model, self.data, self.hand_joint_id, frame)[:, self.v_indices].copy()
        # The tool point is offset from the joint, so it also moves when the hand rotates.
        J[:3] -= _skew(self.data.oMi[self.hand_joint_id].rotation @ self.tool_offset) @ J[3:]
        return J

    def step_bounds(self, q, dt: float, speed_scale: float) -> Tuple[np.ndarray, np.ndarray]:
        """Return the least and most each joint may move in one step of `dt` seconds.

        A joint is limited by its speed and by its distance to the standoff before each limit. A joint
        already inside a standoff may only move away from that limit.
        """
        q = np.asarray(q, float)
        step = self._joint_speed * (speed_scale * dt)
        scale = self.settings.limit_scale
        up = np.maximum(0.0, scale * (self._highest - q))
        down = np.maximum(0.0, scale * (q - self._lowest))
        return -np.minimum(step, down), np.minimum(step, up)

    def forget_last_step(self) -> None:
        """Drop the previous step's joint speeds, which `continuity` would otherwise carry over."""
        self._last_rate = None

    def step(self, q, linear, angular, dt: float, speed_scale: float = 1.0,
             secondary: Optional[np.ndarray] = None, posture_fraction: float = 1.0) -> np.ndarray:
        """Solve for the joint motion (rad) that moves the tool point by `linear` and rotates the hand by `angular`.

        Args:
            q: Current joint angles (rad).
            linear: Wanted motion of the tool point this step (m, base frame).
            angular: Wanted rotation of the hand this step, as a rotation vector (rad, base frame).
            dt: Length of the step (s).
            speed_scale: Fraction of the joint speed limits to use.
            secondary: Extra joint motion (rad), applied in the null space of the hand's motion.
            posture_fraction: Scales the pull toward the rest posture, 0 to 1.
        """
        s = self.settings
        q = np.asarray(q, float)
        linear = _shortened(np.array(linear, float), s.max_lin_vel * dt)
        angular = _shortened(np.array(angular, float), s.max_rot_vel * dt)

        # Weighted least squares on the hand error, with a small penalty on joint motion.
        A = self.jacobian(q) * self._weights[:, None]
        b = np.concatenate([linear, angular]) * self._weights
        H = A.T @ A + self._damping
        g = A.T @ b
        if s.continuity and self._last_rate is not None:
            H += self._smoothing
            g += s.continuity * self._last_rate * dt
        low, high = self.step_bounds(q, dt, speed_scale)
        dq = bounded_least_squares(H, g, low, high)

        # Posture goals are projected into the null space of the task, so they do not move the hand
        # (beyond what clipping to the step bounds costs).
        posture = posture_fraction * s.posture_gain * dt * (self.rest_posture - q)
        if secondary is not None:
            posture = posture + secondary
        null_space = self._identity - np.linalg.pinv(A, rcond=NULLSPACE_RCOND) @ A
        dq = np.clip(dq + null_space @ posture, low, high)
        self._last_rate = dq / dt
        return dq


def _shortened(vector: np.ndarray, limit: float) -> np.ndarray:
    """Scale `vector` down to length `limit` if it is longer."""
    length = _norm(vector)
    return vector * (limit / length) if length > limit else vector


@dataclass
class GuardSettings:
    """The guards' tuning."""
    # The body's footprints in the base frame, each {x: [low, high], y: [low, high]} and optionally `below`
    # (m), the height up to which it counts.
    body_boxes: List[dict] = field(default_factory=list)
    arms_apart: float = 0.12    # m, forearms closer than this push their elbows apart
    apart_gain: float = 2.0     # 1/s
    hands_apart: float = 0.10   # m, the wrists are not brought closer than this
    keep_ahead: float = 0.10    # m, the wrist stays this far in front of its own shoulder
    stretch: float = 0.93       # fraction of full reach the wrist may go out to
    # m, an elbow or forearm closer to the body than this is pushed outward. Keep it under the start
    # posture's own distance to the body.
    body_clear: float = 0.10
    body_gain: float = 8.0      # 1/s


class ArmPoints(NamedTuple):
    """Where an arm is, for the guards."""
    shoulder: np.ndarray
    elbow: np.ndarray
    wrist: np.ndarray
    elbow_jacobian: np.ndarray   # 3x7, how the elbow moves with the joints


def unit(vector: np.ndarray) -> np.ndarray:
    """Return `vector` scaled to length 1, or zeros if it has no length."""
    length = _norm(vector)
    return vector / length if length > TINY else np.zeros_like(vector)


class Footprint(NamedTuple):
    """A box of the body seen from above, with the height up to which it counts (None: every height)."""
    low: np.ndarray    # [x, y] of its low corner
    high: np.ndarray   # [x, y] of its high corner
    top: Optional[float]

    @classmethod
    def from_settings(cls, box: dict, clearance: float) -> "Footprint":
        """From a `body_boxes` entry. A point is checked against it up to `below` plus the clearance."""
        top = box["below"] + clearance if "below" in box else None
        return cls(np.array([box["x"][0], box["y"][0]], float), np.array([box["x"][1], box["y"][1]], float), top)


def distance_outside(point: np.ndarray, box: Footprint) -> Tuple[float, np.ndarray]:
    """Return how far `point` is outside a footprint and the horizontal direction out of it.

    Inside the footprint the distance is 0 and the direction points out through the nearest side.
    """
    xy = point[:2]
    out = xy - np.clip(xy, box.low, box.high)
    distance = _norm(out)
    if distance > TINY:
        x, y = out.tolist()
        return distance, np.array([x / distance, y / distance, 0.0])
    to_sides = [xy[0] - box.low[0], box.high[0] - xy[0], xy[1] - box.low[1], box.high[1] - xy[1]]
    return 0.0, OUT_OF_BOX[int(np.argmin(to_sides))].copy()


def closest_on_segments(a0, a1, b0, b1) -> Tuple[float, np.ndarray, np.ndarray]:
    """Find the closest points of segments a0-a1 and b0-b1.

    Returns the fraction along a, the point on a and the point on b. After Ericson, Real-Time
    Collision Detection, section 5.1.9.
    """
    da = a1 - a0
    db = b1 - b0
    r = a0 - b0
    aa = da @ da
    bb = db @ db
    ab = da @ db
    ar = da @ r
    br = db @ r
    denominator = aa * bb - ab * ab
    # Closest point on line a to line b, clamped to the segment; then its nearest point on b, clamped;
    # then back to a, because the second clamp may have moved the answer.
    s = _clamp((ab * br - bb * ar) / denominator, 0.0, 1.0) if denominator > TINY else 0.0
    t = _clamp((ab * s + br) / bb, 0.0, 1.0) if bb > TINY else 0.0
    s = _clamp((ab * t - ar) / aa, 0.0, 1.0) if aa > TINY else 0.0
    return float(s), a0 + s * da, b0 + t * db


class Guards:
    """The guards of one arm.

    Args:
        ik: The arm's `ArmIK`.
        shoulder_joint, elbow_joint: URDF names of the arm's first joint and its elbow joint.
        settings: The guard settings.
    """

    @classmethod
    def for_arm(cls, ik: ArmIK, arm, settings: GuardSettings) -> "Guards":
        """The guards of an SDK arm, on its solver."""
        joints = arm.describe().joints
        return cls(ik, joints[0].urdf_joint, joints[ELBOW].urdf_joint, settings)

    def __init__(self, ik, shoulder_joint: str, elbow_joint: str, settings: GuardSettings):
        self.ik = ik
        self.settings = settings
        self.shoulder_joint = shoulder_joint
        self.elbow_joint = elbow_joint
        self._footprints = [Footprint.from_settings(box, settings.body_clear) for box in settings.body_boxes]

    def points(self, q) -> ArmPoints:
        """Locate the arm's shoulder, elbow and wrist for joint angles `q` (rad, URDF frame)."""
        shoulder, _ = self.ik.joint_point(q, self.shoulder_joint)
        elbow, elbow_jacobian = self.ik.joint_point(q, self.elbow_joint)
        wrist, _ = self.ik.wrist_point(q)
        return ArmPoints(shoulder, elbow, wrist, elbow_jacobian)

    def elbow_motion(self, arm: ArmPoints, dt: float, other: Optional[ArmPoints]) -> Tuple[np.ndarray, float]:
        """Compute the joint motion that moves the elbow away from the body and the other arm.

        Returns:
            The joint motion (rad) and the fraction of the posture pull to keep. Pass both to
            `ArmIK.step()` as `secondary` and `posture_fraction`. Near the body the posture pull is
            faded out, so nothing pulls the elbow back in while it is being pushed away.
        """
        s = self.settings
        velocity = np.zeros(3)   # of the elbow, m/s
        if other is not None:
            along, mine, theirs = closest_on_segments(arm.elbow, arm.wrist, other.elbow, other.wrist)
            gap = _norm(mine - theirs)
            if TINY < gap < s.arms_apart:
                velocity += s.apart_gain * (s.arms_apart - gap) * (mine - theirs) / gap * (1.0 - along)

        body_gap = None
        forearm = arm.wrist - arm.elbow
        for along in FOREARM_CHECKS:
            point = arm.elbow + along * forearm
            for box in self._footprints:
                if box.top is not None and point[2] > box.top:
                    continue
                gap, outward = distance_outside(point, box)
                body_gap = gap if body_gap is None else min(body_gap, gap)
                if gap < s.body_clear:
                    velocity += s.body_gain * (s.body_clear - gap) * outward * (1.0 - along)

        posture_fraction = 1.0
        if body_gap is not None:
            crowding = (s.body_clear - body_gap) / (POSTURE_FADE * s.body_clear)
            posture_fraction = 1.0 - _clamp(crowding, 0.0, 1.0)
        if not velocity.any():
            return np.zeros(arm.elbow_jacobian.shape[1]), posture_fraction
        return np.linalg.pinv(arm.elbow_jacobian, rcond=ELBOW_RCOND) @ velocity * dt, posture_fraction

    def limit_hand(self, linear: np.ndarray, arm: ArmPoints, other: Optional[ArmPoints]) -> np.ndarray:
        """Remove from `linear` (m, this step) the components the guards do not allow."""
        s = self.settings
        linear = np.array(linear, float)

        # Stretch: near full extension the arm is singular along its own line.
        line = arm.wrist - arm.shoulder
        outward_direction = unit(line)
        reach = _norm(arm.elbow - arm.shoulder) + _norm(arm.wrist - arm.elbow)
        extension = _norm(line) / reach
        outward = linear @ outward_direction
        fade_start = s.stretch - FADE_EXTENSION
        if outward > 0.0 and extension > fade_start:
            linear -= min(1.0, (extension - fade_start) / FADE_EXTENSION) * outward * outward_direction

        # Behind the shoulder: the upper arm would have to swing behind the body.
        ahead = line[0] - s.keep_ahead
        if linear[0] < 0.0 and ahead < FADE_DISTANCE:
            linear[0] *= _clamp(ahead / FADE_DISTANCE, 0.0, 1.0)

        # The other wrist.
        if other is not None:
            away = arm.wrist - other.wrist
            gap = _norm(away)
            apart = unit(away)
            closing = -(linear @ apart)
            fade_start = s.hands_apart + FADE_DISTANCE
            if closing > 0.0 and TINY < gap < fade_start:
                linear += min(1.0, (fade_start - gap) / FADE_DISTANCE) * closing * apart
        return linear
