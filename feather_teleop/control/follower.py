# One arm following one Quest controller, through the SDK arm's follow mode.
#
# Joint angles named `q` are in the motor's frame, which the SDK takes; `q_urdf` is in the URDF's frame, which the solver and the guards use.
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from feathersdk.robot.actuator_systems.arm_kinematics import ELBOW_RCOND, ArmIK, ArmPoints, Guards, unit
from feathersdk.robot.actuator_systems.collision import CollisionError

from feather_teleop.control.controller import Action, Controller
from feather_teleop.control.maths import clamp, norm, smoothing_factor, to_vector

MAX_STEP_TIME = 0.1        # s, the most time one step may cover, so a stall never becomes a jump
AT_START = 0.01            # rad, arrival tolerance at the start posture
STILL = 1e-3               # rad/s, a commanded joint slower than this is at rest
AT_REST = 0.001            # m, a commanded hand that stays this close to one place is at rest
MIN_RELEASE = 0.05         # s the grip must stay released before the next press counts
DOUBLE_PRESS = 0.4         # s, two grip presses within this count as a double press
STICK_HOLD = 0.6           # s the joystick is held down to redefine "forward"
GRIPPER_OPEN_SNAP = 0.01   # an opening gripper within this of fully open is sent fully open
FLOOR_ATTEMPTS = 4         # times a step that ends below the floor is halved before it is dropped
FLOOR_CREEP = 0.2          # share of `floor_down_speed` still allowed at the floor itself, so the last centimetre is reached
HOME, WRIST_VIEW = "home", "wrist_view"   # what two grip presses do


@dataclass
class ElbowFollowSettings:
    # The operator's elbow, estimated, steering the robot's: its swing round the shoulder-wrist line, through the arm's spare freedom, so the hand stays where it is.
    #
    # The headset gives no elbow: the shoulder is placed from the head, and the forearm runs back from the controller along a direction fixed in the controller's frame, taken at the demonstration's first grip with the operator's arm held as the start posture (upper arm hanging, forearm forward). The robot's arm is shaped like a human's, so the same swing reads the same on it.
    gain: float = 4.0                                    # 1/s, how hard the elbow is turned toward the operator's
    shoulder: Tuple[float, float, float] = (-0.05, 0.18, -0.22)   # m from the head: back, aside (to the arm's side), down
    upper_arm: float = 0.30                              # m
    forearm: float = 0.27                                # m
    straight: float = 0.03                               # m, an elbow nearer its shoulder-wrist line than this says nothing
    min_radius: float = 0.02                             # m, the robot's elbow nearer its line than this is not steered
    min_want: float = 0.2                                # the operator's direction, across the robot's line, must keep this much of its length


@dataclass
class ArmSettings:
    # How an arm follows. The `arm` section of quest.yaml sets the first three; the rest are tuned defaults.
    speed_scale: float = 0.6          # fraction of each motor's speed limit
    return_speed: float = 0.3         # rad/s, top joint speed on the way to the start posture
    floor: float = 0.0                # m, lowest height of the tool point above the floor (0: the fingertips reach the ground)
    floor_slow_within: float = 0.15   # m above `floor` from which the tool point goes down at most `floor_down_speed`
    floor_down_speed: float = 0.10    # m/s, less the nearer the floor, so nothing slams into it
    feedforward: float = 1.0          # fraction of the controller's own velocity added, so the hand does not trail
    feedforward_hz: float = 4.0       # Hz, cutoff of the low-pass on that velocity
    leash: float = 0.3                # rad, a joint this far from all its recent targets is caught on something
    leash_window: float = 0.6         # s of recent targets the leash checks; the measurement can lag this much
    sag_correction_hz: float = 0.5    # Hz, how fast the commanded hand is moved toward the measured one at rest
    sag_correction_max: float = 0.05  # m, the largest sag correction
    sag_still: float = 0.004          # m, smaller changes in the sag are ignored, so a sticking joint can't hunt
    gripper_speed: float = 3.75       # full gripper strokes per second at full command
    gripper_resend: float = 0.004     # fraction of the stroke the command must change before it is sent again
    gripper_squeeze: float = 0.012    # fraction of the stroke sent past jaws that stopped; limits grip force
    gripper_squeeze_after: float = 0.5  # s the jaws must stand still before the squeeze limit applies
    elbow_follow: bool = False        # the robot's elbow swings as the operator's does (worn headset; see ElbowFollowSettings)
    elbow: ElbowFollowSettings = field(default_factory=ElbowFollowSettings)


class GripPresses:
    # Detects grip presses. A release shorter than MIN_RELEASE is a stream dropout, not a new press.

    def __init__(self):
        self._was_down = False
        self._released_at: Optional[float] = None

    def pressed(self, down: bool, now: float) -> bool:
        # Return True on the tick the grip goes down after a real release.
        if not down:
            if self._was_down or self._released_at is None:
                self._released_at = now
            self._was_down = False
            return False
        is_press = not self._was_down and self._released_at is not None and now - self._released_at >= MIN_RELEASE
        self._was_down = True
        return is_press


class StickPresses:
    # Tells a click of the joystick from holding it down.
    CLICK, HOLD = "click", "hold"

    def __init__(self):
        self._was_down = False
        self._down_at = 0.0
        self._reported = False

    def event(self, down: bool, now: float) -> Optional[str]:
        # Return CLICK when the stick is released early, HOLD once it has been down for STICK_HOLD, else None.
        was_down = self._was_down
        self._was_down = down
        if down and not was_down:
            self._down_at = now
            self._reported = False
        elif down and not self._reported and now - self._down_at >= STICK_HOLD:
            self._reported = True
            return self.HOLD
        elif was_down and not down and not self._reported:
            return self.CLICK
        return None

    def ignore_current_press(self) -> None:
        # Make the press under way report nothing.
        self._reported = True


class GripperCommand:
    # Integrates the controller's closing speed into a gripper position and sends it to the SDK gripper.
    #
    # Args:
    #     gripper: The SDK gripper, or None if the arm has none.
    #     settings: The arm settings (the `gripper_` fields).

    def __init__(self, gripper, settings: ArmSettings, clock=time.monotonic):
        self.gripper = gripper
        self.speed = settings.gripper_speed
        self.resend = settings.gripper_resend
        self.squeeze = settings.gripper_squeeze
        self.squeeze_after = settings.gripper_squeeze_after
        self.clock = clock
        self._jaws: Optional[float] = None   # the measured opening where the jaws last stood
        self._jaws_since = 0.0               # and since when
        self._asking_since: Optional[float] = None   # since when the command asks the jaws further closed than `squeeze`
        self.closure = 0.0                   # commanded closure: 0 open, 1 closed
        self.fault: Optional[str] = None     # why the gripper was given up on, if it was
        self._sent: Optional[float] = None

    def sync(self) -> None:
        # Start from where the gripper is, so a held object is not dropped at the next grip.
        if self.gripper is None:
            return
        try:
            self.closure = 1.0 - float(self.gripper.get_opening())
        except (ValueError, RuntimeError) as error:   # not calibrated, or no feedback yet
            self._give_up(error)
        self._sent = None
        self._asking_since = None   # a pull of the demonstration before does not count

    def update(self, closing_speed: float, dt: float) -> None:
        # Advance the command by `closing_speed` (in [-1, 1]) and send it if it changed enough.
        closure = clamp(self.closure + closing_speed * self.speed * dt, 0.0, 1.0)
        if closure < GRIPPER_OPEN_SNAP and closing_speed < 0:
            closure = 0.0   # the proportional pull fades near its goal and would never fully open
        self.closure = closure
        if self.gripper is None:
            return
        try:
            opening = 1.0 - closure   # the SDK takes 1 for open, 0 for closed
            if closure > 0.0:
                opening = self._eased(opening)
            else:
                self._asking_since = None   # open: nothing is asked of the jaws
            sent = self._sent
            reached_an_end = opening in (0.0, 1.0) and opening != sent
            if sent is None or abs(opening - sent) >= self.resend or reached_an_end:
                self.gripper.go_to(opening)
                self._sent = opening
        except (ValueError, RuntimeError) as error:
            self._give_up(error)

    def _eased(self, opening: float) -> float:
        # Return `opening`, kept within `squeeze` of the jaws once they have stood still for `squeeze_after` while asked further closed.
        #
        # Counted from the later of the two, so jaws that rested at any opening get `squeeze_after` to start moving: counted from their last motion alone, a rested gripper was held to a squeeze too small to start it and never closed.
        jaws = float(self.gripper.get_opening())
        now = self.clock()
        if self._jaws is None or abs(jaws - self._jaws) > self.squeeze / 2:   # they moved
            self._jaws = jaws
            self._jaws_since = now
        if opening >= jaws - self.squeeze:
            self._asking_since = None
            return opening
        if self._asking_since is None:
            self._asking_since = now
        if now - max(self._jaws_since, self._asking_since) >= self.squeeze_after:   # blocked by an object, or at the end of the stroke
            return max(opening, jaws - self.squeeze)
        return opening

    def _give_up(self, error: Exception) -> None:
        # Drop a faulted or unplugged gripper for the rest of the session, so it never stops the arm.
        self.fault = f"{type(error).__name__}: {error}"
        self.gripper = None


class ArmFollower:
    # Drives one SDK arm from one controller.
    #
    # Args:
    #     side: "left" or "right".
    #     arm: The SDK arm (`robot.arms_system.left_arm`).
    #     ik: The arm's solver.
    #     controller: The controller that drives it.
    #     guards: The arm's guards.
    #     settings: The arm settings.
    #     start_posture: Joint angles of the start posture (rad, URDF frame, the arm's joint order).
    #     gripper: The SDK gripper on this arm, or None.
    #     double_press_action: HOME (back to the start posture) or WRIST_VIEW (toggle `wrist_view`).
    #     clock: Returns the time in seconds.
    #
    # Raises:
    #     ValueError: `speed_scale` is not in (0, 1], or the start posture is outside a joint's range.

    def __init__(self, side: str, arm, ik: ArmIK, controller: Controller, guards: Guards, settings: ArmSettings, start_posture, gripper=None, double_press_action: str = HOME, clock=time.monotonic):
        self.side = side
        self.arm = arm
        self.ik = ik
        self.controller = controller
        self.guards = guards
        self.settings = settings
        self.double_press_action = double_press_action
        self.clock = clock

        description = arm.describe()
        self.joints = description.joints
        self.names = [joint.name for joint in self.joints]
        self.tick = 1.0 / description.rate
        if not 0.0 < settings.speed_scale <= 1.0:
            raise ValueError(f"arm.speed_scale is {settings.speed_scale}; it must be above 0 and at most 1")
        if double_press_action not in (HOME, WRIST_VIEW):
            raise ValueError(f"double_press_action is {double_press_action!r}; it must be {HOME!r} or {WRIST_VIEW!r}")
        limits = description.limits()
        self.follow_speed = settings.speed_scale * limits.velocity
        self.return_speed = np.minimum(self.follow_speed, settings.return_speed)
        self.direction = np.array([joint.urdf_direction for joint in self.joints], float)   # URDF = direction * motor
        self.lower = np.array([joint.position[0] for joint in self.joints])
        self.upper = np.array([joint.position[1] for joint in self.joints])
        self.start_posture = np.asarray(start_posture, float) * self.direction
        outside = (self.start_posture < self.lower) | (self.start_posture > self.upper)
        if outside.any():
            names = ", ".join(joint.urdf_joint for joint, out in zip(self.joints, outside) if out)
            raise ValueError(f"start_posture puts {names} outside the joint's range in the robot's config")

        self.gripper = GripperCommand(gripper, settings, clock)
        self.other: Optional["ArmFollower"] = None   # the other arm's follower, when both are driven
        self._window = max(1, round(settings.leash_window / self.tick))   # ticks the leash looks back
        self._reset(np.array(arm.read_state().position, float))

    def _reset(self, q: np.ndarray) -> None:
        # Set every piece of per-demonstration state, continuing from target `q`.
        self.q = q                                   # the last target sent
        self.v = np.zeros(len(q))                    # and its velocity
        self.measured = q.copy()
        self.sag_offset = np.zeros(3)                # measured tool position minus commanded, low-passed
        self.returning = False                       # on the way back to the start posture
        self.position_locked = False                 # only orientation follows; the tool point stays put
        self.wrist_view = False                      # the worn headset shows this arm's wrist camera
        self._forearm: Optional[np.ndarray] = None   # the operator's forearm in the controller's frame, taken at the first grip
        self.target: Optional[np.ndarray] = None     # where the controller asks the tool point to be
        self.refused: Optional[str] = None           # why the last target was refused, if it was
        self.held_back: List[str] = []               # joints the leash held back on the last step
        self._return_stoppable = True
        self._locked_position: Optional[np.ndarray] = None
        self._last_step_time: Optional[float] = None
        self._last_press: Optional[float] = None
        self._grip = GripPresses()
        self._stick = StickPresses()
        self._recent = np.tile(q, (self._window, 1))   # ring of the targets wanted over the leash window
        self._recent_at = 0
        self._rest_anchor = self._hand_position(q)    # where the sent hand has stayed
        self._rest_since = self.clock()                # since when
        self._reset_feedforward()

    def start(self) -> None:
        # Switch the arm to follow mode and continue from the setpoint the SDK holds.
        self.arm.set_control_mode("follow")
        self._set_speed(self.follow_speed)
        state = self.arm.read_state()
        held = state.command.position if state.command is not None else state.position
        self._reset(np.array(held, float))
        self.measured = np.array(state.position, float)
        self.gripper.sync()
        self.controller.reset()

    def stop(self) -> None:
        # Brake the arm and hold it, in the SDK's targets mode.
        self.arm.set_control_mode("targets")

    def return_to_start(self, stoppable_by_grip: bool = True) -> None:
        # Send the arm back to its start posture, at `return_speed`.
        self.returning = True
        self._return_stoppable = stoppable_by_grip
        self.position_locked = False
        self._locked_position = None
        self._reset_feedforward()
        # Sent at rest before the speed drops, so no target is moving faster than the new limits.
        self._send(self.start_posture, np.zeros_like(self.v))
        if self.returning:   # not refused
            self._set_speed(self.return_speed)

    def at_start_posture(self, tolerance: float) -> bool:
        # Return True if every joint of the last target is within `tolerance` (rad) of the start posture.
        return bool(np.max(np.abs(self.q - self.start_posture)) <= tolerance)

    def tool_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        # Return the commanded tool point (m) plus the sag correction, and the hand's rotation.
        position, rotation = self.ik.pose(self.q * self.direction)
        return position + self.sag_offset, rotation

    def points(self) -> ArmPoints:
        # Locate the shoulder, elbow and wrist: at the last target, or on a return, where the SDK has the arm now.
        q = self._sent_position() if self.returning else self.q
        return self.guards.points(q * self.direction)

    def step(self) -> None:
        # Advance one tick and send the new target, if there is one.
        now = self.clock()
        dt = self._step_time(now)
        self.controller.update()
        state = self.arm.read_state()
        self.measured = np.array(state.position, float)
        self._correct_sag(state.command, now, dt)
        self._handle_grip(now)
        self._handle_stick(now)
        self.target = None

        if self.returning:
            self._check_arrival(state.command)
        elif self.controller.gripping:
            self._follow_controller(dt)
        else:
            self._come_to_rest()

    def _step_time(self, now: float) -> float:
        # Return the time this step covers: the real time since the last step, so a slow loop keeps its speed.
        elapsed = self.tick if self._last_step_time is None else now - self._last_step_time
        self._last_step_time = now
        return min(max(elapsed, self.tick), MAX_STEP_TIME)

    def _follow_controller(self, dt: float) -> None:
        # Send the next target toward the controller's, and move the gripper.
        action = self.controller.action(*self.tool_pose(), self.gripper.closure)
        goal = self._joint_goal(action, dt)
        if np.isfinite(goal).all():
            goal = np.clip(goal, self.lower, self.upper)
            velocity = np.clip((goal - self.q) / dt, -self.follow_speed, self.follow_speed)
            self._send(*self._leashed(goal, velocity))
        else:
            self.refused = "the solver's result was not a number"
        self.gripper.update(action.gripper, dt)

    def _come_to_rest(self) -> None:
        # Without the grip, stop on the last target.
        self._reset_feedforward()
        if self.v.any():
            self._send(self.q, np.zeros_like(self.v))

    def _check_arrival(self, command) -> None:
        # End the return once the SDK's own setpoint has reached the start posture and stopped.
        if command is None:
            return
        arrived = np.max(np.abs(command.position - self.start_posture)) <= AT_START
        if arrived and np.max(np.abs(command.velocity)) < STILL:
            self._end_return()

    def _end_return(self) -> None:
        self.returning = False
        self._set_speed(self.follow_speed)
        self.controller.restart_origin()   # a grip that is still held continues from here

    def _joint_goal(self, action: Action, dt: float) -> np.ndarray:
        # Return the joint goal (motor frame) that follows the controller for one step.
        s = self.settings
        position, _ = self.tool_pose()
        linear = action.linear * (self.ik.settings.max_lin_vel * dt)
        angular = action.angular * (self.ik.settings.max_rot_vel * dt)

        if self.position_locked:
            self.target = self._locked_position
            linear = self._locked_position - position
        else:
            self.target = action.target_position
            linear = linear + s.feedforward * dt * self._target_velocity(action.target_position, dt)
        angular = angular + s.feedforward * dt * self._target_angular_velocity(action.target_rotation, dt)

        linear[2] = max(linear[2], s.floor - position[2])
        linear[2] = slowed_down(linear[2], position[2] - s.floor, s.floor_slow_within, s.floor_down_speed, dt)
        q_urdf = self.q * self.direction
        mine = self.guards.points(q_urdf)
        theirs = self.other.points() if self.other is not None else None
        linear = self.guards.limit_hand(linear, mine, theirs)
        elbow_motion, posture_fraction = self.guards.elbow_motion(mine, dt, theirs)
        if s.elbow_follow:
            swing = self._elbow_swing(mine, dt)
            if swing is not None:
                # The arm has one spare freedom, so the lean to the start posture would pull against the operator's elbow on the same motion: the elbow has it alone, faded as the guards fade the lean.
                elbow_motion = elbow_motion + posture_fraction * swing
                posture_fraction = 0.0
        move = self.ik.step(q_urdf, linear, angular, dt, s.speed_scale, elbow_motion, posture_fraction)
        move = self._keep_above_floor(q_urdf, move)
        return self.q + move * self.direction

    def _operator_elbow(self) -> Optional[np.ndarray]:
        # Return which way the operator's elbow stands out from their own shoulder-wrist line (a unit vector, robot axes), or None when it cannot be told.
        seen = self.controller.operator()
        if seen is None:
            return None
        head, hand, turn = seen
        e = self.settings.elbow
        side = 1.0 if self.side == "left" else -1.0   # the robot's y points left
        shoulder = head + np.array([e.shoulder[0], side * e.shoulder[1], e.shoulder[2]])
        if self._forearm is None:   # the first grip: the operator's arm as the start posture
            elbow = shoulder - np.array([0.0, 0.0, e.upper_arm])
            self._forearm = turn.T @ unit(hand - elbow)
            return None
        elbow = hand - e.forearm * (turn @ self._forearm)
        axis = unit(hand - shoulder)
        out = (elbow - shoulder) - ((elbow - shoulder) @ axis) * axis
        size = norm(out)
        return None if size < e.straight else out / size

    def _elbow_swing(self, arm: ArmPoints, dt: float) -> Optional[np.ndarray]:
        # Return the joint motion (rad this step, URDF frame) that swings the robot's elbow toward the operator's, or None.
        want = self._operator_elbow()
        if want is None:
            return None
        e = self.settings.elbow
        axis = unit(arm.wrist - arm.shoulder)
        upper = arm.elbow - arm.shoulder
        radial = upper - (upper @ axis) * axis   # from the shoulder-wrist line out to the elbow
        want = want - (want @ axis) * axis
        if norm(radial) < e.min_radius or norm(want) < e.min_want:
            return None
        r, w = unit(radial), unit(want)
        angle = math.atan2(float(np.cross(r, w) @ axis), float(r @ w))
        velocity = e.gain * angle * np.cross(axis, r) * norm(radial)   # m/s of the elbow, round its circle
        return np.linalg.pinv(arm.elbow_jacobian, rcond=ELBOW_RCOND) @ velocity * dt

    def _leashed(self, goal: np.ndarray, velocity: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # Hold back joints that are not following: keep their target within `leash` of where they are.
        #
        # A joint counts as following if any target over the leash window was within `leash` of its measurement.
        recent = self._recent
        recent[self._recent_at] = goal
        self._recent_at = (self._recent_at + 1) % len(recent)
        leash = self.settings.leash
        measured = self.measured
        too_far = np.min(np.abs(recent - measured), axis=0) > leash
        if not too_far.any():
            self.held_back = []
            return goal, velocity
        self.held_back = [joint.urdf_joint for joint, far in zip(self.joints, too_far) if far]
        leashed = np.clip(measured + np.copysign(leash, goal - measured), self.lower, self.upper)
        return np.where(too_far, leashed, goal), np.where(too_far, 0.0, velocity)

    def _send(self, q: np.ndarray, v: np.ndarray) -> None:
        # Send a follow target; on a refusal, stop on the previous one and record why.
        try:
            self.arm.set_follow_target(self._by_name(q), self._by_name(v))
        except CollisionError as refused:
            # The SDK keeps the previous target and its velocity. Resend it at rest (it sweeps nothing, so it is never refused) so the arm stops there.
            self.v = np.zeros_like(self.v)
            self.arm.set_follow_target(self._by_name(self.q), self._by_name(self.v))
            self.refused = str(refused)
            self._reset_feedforward()   # the refused step's speeds must not carry into the next one
            if self.returning:
                self.returning = False
                self._set_speed(self.follow_speed)
            return
        self.q = q
        self.v = v
        self.refused = None

    def _set_speed(self, speed: np.ndarray) -> None:
        self.arm.set_follow_limits(max_velocities=self._by_name(speed))

    def _by_name(self, values: np.ndarray) -> dict:
        # Return per-joint `values` keyed by joint name, as the SDK takes them.
        return dict(zip(self.names, values.tolist()))

    def _hand_position(self, q: np.ndarray) -> np.ndarray:
        # Return the tool point's position (m) for motor-frame joint angles `q`.
        return self.ik.pose(q * self.direction)[0]

    def _correct_sag(self, command, now: float, dt: float) -> None:
        # Move the sag correction toward the measured hand's offset while the SDK's setpoint is at rest.
        #
        # The setpoint is used, not the target, because on a return the target is still while the arm moves.
        if command is None:
            return
        s = self.settings
        commanded = self._hand_position(command.position)
        if norm(commanded - self._rest_anchor) > AT_REST:
            self._rest_anchor = commanded
            self._rest_since = now
            return
        if now - self._rest_since < s.leash_window:   # the late measurement may still show the motion
            return
        gap = self._hand_position(self.measured) - commanded
        if not np.isfinite(gap).all() or norm(gap - self.sag_offset) <= s.sag_still:
            return
        self.sag_offset += smoothing_factor(s.sag_correction_hz, dt) * (gap - self.sag_offset)
        size = norm(self.sag_offset)
        if size > s.sag_correction_max:
            self.sag_offset *= s.sag_correction_max / size

    def _target_velocity(self, target: np.ndarray, dt: float) -> np.ndarray:
        # Return the target's low-passed velocity (m/s); headset samples and ticks do not line up, so it is noisy.
        if self._last_target is not None:
            raw = (target - self._last_target) / dt
            self._velocity += smoothing_factor(self.settings.feedforward_hz, dt) * (raw - self._velocity)
        self._last_target = target
        return self._velocity

    def _target_angular_velocity(self, rotation: np.ndarray, dt: float) -> np.ndarray:
        # Return the target's low-passed angular velocity (rad/s, base axes).
        if self._last_rotation is not None:
            raw = to_vector(rotation @ self._last_rotation.T) / dt
            factor = smoothing_factor(self.settings.feedforward_hz, dt)
            self._angular_velocity += factor * (raw - self._angular_velocity)
        self._last_rotation = rotation
        return self._angular_velocity

    def _reset_feedforward(self) -> None:
        # Clear the velocity estimates, so the next grip starts them from rest.
        self._last_target = None
        self._last_rotation = None
        self._velocity = np.zeros(3)
        self._angular_velocity = np.zeros(3)
        self.ik.forget_last_step()

    def _keep_above_floor(self, q_urdf: np.ndarray, move: np.ndarray) -> np.ndarray:
        # Shorten `move` until it no longer lowers the tool point below the floor.
        #
        # Limiting the hand's motion is not enough: the solver may trade position for orientation.
        floor = self.settings.floor - self.sag_offset[2]
        height = self.ik.pose(q_urdf)[0][2]
        for _ in range(FLOOR_ATTEMPTS):
            after = self.ik.pose(q_urdf + move)[0][2]
            if after >= floor or after >= height:
                return move
            move = move / 2.0
        return np.zeros_like(move)

    def _handle_grip(self, now: float) -> None:
        # A double grip press goes home or toggles the wrist view; a press during a return stops it.
        held = self.controller.gripping or self.controller.grip_untracked   # lost tracking is not a release
        if not self._grip.pressed(held, now):
            return
        if self.returning:
            if self._return_stoppable:
                self._send(self._sent_position(), np.zeros_like(self.v))   # brake to where the arm is now
                self._end_return()
                self._last_press = None
        elif self._last_press is not None and now - self._last_press <= DOUBLE_PRESS:
            self._last_press = None
            if self.double_press_action == WRIST_VIEW:
                self.wrist_view = not self.wrist_view
            else:
                self.return_to_start()
        else:
            self._last_press = now

    def _sent_position(self) -> np.ndarray:
        # Return the setpoint the SDK last sent to the motors, or the last target before there was one.
        command = self.arm.read_state().command
        return self.q if command is None else np.array(command.position, float)

    def _handle_stick(self, now: float) -> None:
        # A joystick click toggles the position lock; holding it redefines "forward".
        event = self._stick.event(self.controller.stick_pressed, now)
        if self.returning:
            self._stick.ignore_current_press()
            return
        if event == StickPresses.HOLD:
            self.controller.redefine_forward()
            self._reset_feedforward()
        elif event == StickPresses.CLICK:
            self.position_locked = not self.position_locked
            self._locked_position = self.tool_pose()[0] if self.position_locked else None
            self.controller.restart_origin()
            self._reset_feedforward()


def slowed_down(step: float, height: float, slow_within: float, down_speed: float, dt: float) -> float:
    # Return `step` (m this tick, up positive) for a point `height` m above the floor, slowed near it.
    #
    # Within `slow_within` of the floor a step down covers at most `down_speed` m/s, in proportion to the height left and never under FLOOR_CREEP of it. Steps up, and steps from higher up, are kept.
    if step >= 0.0 or height >= slow_within:
        return step
    return max(step, -down_speed * max(FLOOR_CREEP, height / slow_within) * dt)


def hand_alignment(ik: ArmIK, start_posture_urdf) -> np.ndarray:
    # Return the hand rotation that an upright controller pointing forward asks for, with absolute orientation.
    #
    # A controller pointing where the start posture's gripper points (turned and tipped, never rolled) gives that posture's hand. The gripper points along its hand frame's -z.
    hand = ik.pose(np.asarray(start_posture_urdf, float))[1]
    x, y, z = (-hand[:, 2]).tolist()
    heading = math.atan2(y, x)
    tip = math.atan2(-z, math.hypot(x, y))   # positive tips it down
    ch, sh, ct, st = math.cos(heading), math.sin(heading), math.cos(tip), math.sin(tip)
    turn = np.array([[ch, -sh, 0.0], [sh, ch, 0.0], [0.0, 0.0, 1.0]])
    tipped = np.array([[ct, 0.0, st], [0.0, 1.0, 0.0], [-st, 0.0, ct]])
    pointing = turn @ tipped   # forward (x) turned to where the gripper points, kept upright
    return pointing.T @ hand
