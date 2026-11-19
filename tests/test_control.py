# A controller driving one arm: the clutch, the frames, the targets sent to follow mode, the guards and the gripper.
import numpy as np
import pytest
from fakes import BENT, SPEED, TABLE, TICK, WORN, FakeArm, Gripper, hand, make_follower, press_twice, run
from feathersdk.robot.actuator_systems.arm_kinematics import Guards, GuardSettings

from feather_teleop.control.follower import AT_START, FLOOR_CREEP, STICK_HOLD, WRIST_VIEW, ArmFollower, ArmSettings, slowed_down
from feather_teleop.control.maths import to_matrix, to_vector


def move(follower, headset, clock, delta, side="r", ticks=150):
    # Grip, move the controller by `delta` over 25 ticks, wait; return how far the hand moved.
    start = hand(follower)[0]
    headset.grip[side] = True
    run(follower, clock, ticks, lambda i: headset.move(side, np.asarray(delta) / 25) if 1 <= i <= 25 else None)
    return hand(follower)[0] - start


@pytest.mark.parametrize("vector", [[0.3, -0.2, 0.5], [0, 0, 0], [1e-10, 0, 0], [np.pi - 1e-8, 0, 0], [0, 2.0, 0]])
def test_rotation_helpers(vector):
    assert to_vector(to_matrix(np.array(vector, float))) == pytest.approx(vector, abs=1e-6)


@pytest.mark.parametrize("mode, controller, robot", [
    ("table", TABLE["forward"], [1, 0, 0]),
    ("table", TABLE["left"], [0, 1, 0]),
    ("table", TABLE["up"], [0, 0, 1]),
    ("mirror", TABLE["forward"] + TABLE["up"], [-1, 0, 1]),   # facing the robot: forward is toward it, up is up
    ("worn", WORN["forward"], [1, 0, 0]),
])
def test_the_hand_follows_the_controller(tmp_path, mode, controller, robot):
    follower, headset, _, clock = make_follower(tmp_path, worn=mode == "worn", mirror=mode == "mirror")
    side = "l" if mode == "mirror" else "r"   # mirrored, the opposite hand drives the arm
    assert move(follower, headset, clock, 0.05 * controller, side) == pytest.approx(0.05 * np.array(robot), abs=4e-3)


def test_the_clutch(tmp_path):
    follower, headset, arm, clock = make_follower(tmp_path)
    run(follower, clock, 20, lambda i: headset.move("r", 0.01 * TABLE["forward"]))
    assert arm.sent == []   # without the grip nothing is sent; the arm stays on its last target

    move(follower, headset, clock, 0.04 * TABLE["forward"])
    headset.grip["r"] = False
    held = follower.q.copy()
    run(follower, clock, 20, lambda i: headset.move("r", 0.01 * TABLE["up"]))   # moved with the clutch out
    headset.grip["r"] = True
    run(follower, clock, 30)
    assert follower.q == pytest.approx(held, abs=1e-3)   # re-gripping doesn't jump to where the controller went

    headset.age = 1.0   # a stale stream lets go
    run(follower, clock, 3, lambda i: headset.move("r", 0.01 * TABLE["forward"]))
    assert not follower.controller.gripping and follower.q == pytest.approx(held, abs=1e-3)

    headset.age, headset.visible = 0.0, {"l", "head"}   # the grip held, the controller out of the cameras' view
    run(follower, clock, 3)
    assert follower.controller.grip_untracked and not follower.controller.gripping


def test_holding_still_ignores_sway(tmp_path):
    follower, headset, _, clock = make_follower(tmp_path)
    headset.grip["r"] = True
    run(follower, clock, 50)
    held = hand(follower)[0]
    sway = lambda i: headset.move("r", 0.002 * np.sin(i / 10) * TABLE["left"] / 10)   # a few mm, slowly
    run(follower, clock, 200, sway)
    assert hand(follower)[0] == pytest.approx(held, abs=1e-3)
    assert move(follower, headset, clock, 0.05 * TABLE["forward"])[0] == pytest.approx(0.05, abs=6e-3)


def test_a_slow_drift_inside_the_bubble_is_ignored(tmp_path):
    follower, headset, _, clock = make_follower(tmp_path)
    headset.grip["r"] = True
    run(follower, clock, 50)   # held still: the bubble forms
    held = hand(follower)[0]
    run(follower, clock, 200, lambda i: headset.move("r", 0.015 / 200 * TABLE["forward"]))   # 1.5 cm over 4 s, under still_speed
    assert hand(follower)[0] == pytest.approx(held, abs=1e-3)
    run(follower, clock, 150, lambda i: headset.move("r", 0.05 / 25 * TABLE["forward"]) if i < 25 else None)   # out of it: followed, and the gap closes
    assert hand(follower)[0][0] - held[0] == pytest.approx(0.065, abs=6e-3)


def test_orientation(tmp_path):
    follower, headset, _, clock = make_follower(tmp_path)   # table: the hand turns as the controller turns
    start = hand(follower)[1]
    headset.grip["r"] = True
    run(follower, clock, 200, lambda i: headset.turn("r", 0.3 / 25 * TABLE["up"]) if 1 <= i <= 25 else None)
    assert to_vector(hand(follower)[1] @ start.T) == pytest.approx([0.0, 0.0, 0.3], abs=0.03)

    follower, headset, _, clock = make_follower(tmp_path, worn=True)   # worn: it points where the controller points
    headset.grip["r"] = True
    run(follower, clock, 400)
    assert np.linalg.norm(to_vector(hand(follower)[1] @ follower.controller.hand_alignment.T)) < 0.05


def test_forward(tmp_path):
    follower, headset, _, clock = make_follower(tmp_path)
    move(follower, headset, clock, 0.01 * TABLE["forward"])   # the first grip sets "forward"
    headset.grip["r"] = False
    headset.turn("r", np.pi / 2 * TABLE["up"])   # the controller now points to the robot's left
    headset.stick["r"] = True
    run(follower, clock, int(STICK_HOLD / TICK) + 3)   # held: "forward" is where it points now
    headset.stick["r"] = False
    run(follower, clock, 2)
    assert move(follower, headset, clock, 0.05 * TABLE["left"]) == pytest.approx([0.05, 0, 0], abs=5e-3)

    headset.grip["r"] = False
    headset.stick["r"] = True
    run(follower, clock, 2)
    headset.age = 1.0   # the stream drops with the stick down: that is not a hold
    run(follower, clock, int(1.0 / TICK))
    assert not follower.controller.stick_pressed and not follower.controller._forward_requested

    follower, headset, _, clock = make_follower(tmp_path, worn=True)   # worn: "forward" is where the head faces
    facing = to_matrix([0.0, 0.0, np.pi / 2])
    headset.pose["head"][:3, :3] = headset.pose["r"][:3, :3] = facing
    assert move(follower, headset, clock, 0.05 * WORN["left"]) == pytest.approx([0.05, 0, 0], abs=4e-3)
    headset.grip["r"] = False
    run(follower, clock, 5)
    turned = to_matrix([0.0, 0.0, -np.pi / 2])   # recentred: the tracking frame turned under the head
    headset.pose["head"][:3, :3] = headset.pose["r"][:3, :3] = turned
    headset.epoch += 1
    assert move(follower, headset, clock, -0.05 * WORN["left"]) == pytest.approx([0.05, 0, 0], abs=4e-3)


def test_position_lock(tmp_path):
    follower, headset, _, clock = make_follower(tmp_path)
    headset.grip["r"] = True
    run(follower, clock, 5)
    start = hand(follower)[0]
    run(follower, clock, 6, lambda i: headset.stick.update(r=i < 3))   # a click
    assert follower.position_locked
    run(follower, clock, 100, lambda i: headset.move("r", 0.002 * TABLE["forward"]) if i < 25 else None)
    assert hand(follower)[0] == pytest.approx(start, abs=3e-3)


def test_targets(tmp_path):
    follower, headset, arm, clock = make_follower(tmp_path)
    limit = follower.settings.speed_scale * SPEED
    assert arm.mode == "follow" and arm.speed == pytest.approx(np.full(7, limit))
    move(follower, headset, clock, 0.3 * TABLE["forward"], ticks=60)   # faster than the arm can go
    assert max(np.max(np.abs(v)) for _, v in arm.sent) > 0.5 * limit   # FakeArm asserts none is above the limit
    headset.grip["r"] = False
    run(follower, clock, 2)
    assert not arm.sent[-1][1].any()   # let go while moving: the last target says to stop there

    headset.grip["r"] = True
    run(follower, clock, 5, lambda i: headset.move("r", 0.003 * TABLE["forward"]))
    arm.refuse = True
    held = follower.q.copy()
    run(follower, clock, 5, lambda i: headset.move("r", 0.003 * TABLE["forward"]))
    assert "touch the body" in follower.refused and follower.q == pytest.approx(held)
    assert arm.sent[-1][0] == pytest.approx(held) and not arm.sent[-1][1].any()   # stopped on the last target
    arm.refuse = False
    run(follower, clock, 5, lambda i: headset.move("r", 0.003 * TABLE["forward"]))
    assert follower.refused is None


@pytest.mark.parametrize("action", ["home", WRIST_VIEW])
def test_double_press(tmp_path, action):
    follower, headset, arm, clock = make_follower(tmp_path, double_press=action)
    move(follower, headset, clock, 0.06 * TABLE["forward"])
    headset.grip["r"] = False
    run(follower, clock, 5)
    run(follower, clock, 20, press_twice(headset))
    if action == WRIST_VIEW:
        assert follower.wrist_view and not follower.at_start_posture(0.05)
    else:   # back at return_speed, then following at full speed again (the fake arm arrives at once)
        assert follower.at_start_posture(AT_START) and not follower.returning and not follower.wrist_view
        assert any(np.allclose(q, follower.start_posture) and not v.any() for q, v in arm.sent)

    run(follower, clock, 5)   # losing tracking with the grip held is not a release, so not a second press
    headset.grip["r"] = True
    run(follower, clock, 3)
    headset.visible = {"l", "head"}
    run(follower, clock, 5)
    headset.visible = {"l", "r", "head"}
    run(follower, clock, 5)
    assert not follower.returning


class LateArm(FakeArm):
    # Measured where it was 10 targets ago, as with slow feedback.

    def __init__(self):
        super().__init__()
        self.history = [self.position.copy()] * 10

    def set_follow_target(self, positions, velocities):
        super().set_follow_target(positions, velocities)
        self.history = self.history[1:] + [self.position.copy()]

    def measured(self):
        return self.history[0].copy()


class StuckArm(FakeArm):
    # Its elbow never moves.

    def measured(self):
        position = super().measured()
        position[3] = BENT[3] * self.direction[3]
        return position


@pytest.mark.parametrize("arm, held_back", [(StuckArm, ["J3"]), (LateArm, [])])
def test_leash(tmp_path, arm, held_back):
    follower, headset, _, clock = make_follower(tmp_path, arm=arm(), leash=0.05)
    move(follower, headset, clock, 0.15 * TABLE["up"], ticks=60)
    assert follower.held_back == held_back
    if held_back:
        assert abs(follower.q[3] - BENT[3] * follower.direction[3]) <= 0.05 + 1e-9


def test_sag_correction(tmp_path):
    class Sagging(FakeArm):
        nan = False

        def measured(self):
            position = super().measured()
            position[0] += 0.05 * self.direction[0]   # 0.05 rad below its target at the shoulder
            if self.nan:
                position[2] = np.nan
            return position

    arm = Sagging()
    follower, headset, _, clock = make_follower(tmp_path, arm=arm)
    headset.grip["r"] = True
    run(follower, clock, 20, lambda i: headset.move("r", 0.004 * TABLE["forward"]))
    assert not follower.sag_offset.any()   # moving: a late measurement is not sag
    run(follower, clock, 400)
    assert np.linalg.norm(follower.sag_offset) > 0.01   # at rest it is corrected
    arm.nan = True
    run(follower, clock, 50)
    assert np.isfinite(follower.sag_offset).all()   # a measurement that isn't a number is ignored


def test_floor(tmp_path):
    follower, headset, _, clock = make_follower(tmp_path)
    floor = hand(follower)[0][2] - 0.02
    follower.settings.floor = floor
    move(follower, headset, clock, 0.10 * -TABLE["up"])
    assert hand(follower)[0][2] == pytest.approx(floor, abs=3e-3)


@pytest.mark.parametrize("step, height, expected", [
    (0.01, 0.05, 0.01),                             # up is never slowed
    (-0.01, 0.20, -0.01),                           # nor down from above slow_within
    (-0.01, 0.075, -0.10 * 0.5 * 0.02),             # halfway: half of down_speed
    (-0.01, 0.0, -0.10 * FLOOR_CREEP * 0.02),       # at the floor: the creep, so it arrives
    (-0.0001, 0.075, -0.0001),                      # already slower than that: kept
])
def test_slowed_down(step, height, expected):
    assert slowed_down(step, height, slow_within=0.15, down_speed=0.10, dt=0.02) == pytest.approx(expected)


def test_gripper(tmp_path):
    gripper = Gripper()
    follower, headset, _, clock = make_follower(tmp_path, gripper=gripper)
    headset.grip["r"], headset.trigger["r"] = True, 0.5
    run(follower, clock, 100)
    assert follower.gripper.closure == pytest.approx(0.75, abs=0.02)   # DROID's 1.5 x trigger
    assert gripper.sent[-1] == pytest.approx(0.25, abs=0.02)
    headset.trigger["r"] = 0.0
    run(follower, clock, 100)
    assert follower.gripper.closure == 0.0 and gripper.sent[-1] == 1.0   # fully open, not almost

    blocked = Gripper(blocked_at=0.5)   # jaws stopped by an object half open: squeezed only a little
    follower, headset, _, clock = make_follower(tmp_path, gripper=blocked)
    headset.grip["r"], headset.trigger["r"] = True, 1.0
    run(follower, clock, 100)
    assert blocked.sent[-1] == pytest.approx(0.5 - follower.settings.gripper_squeeze)

    rested = Gripper(late=50)   # open at rest, its feedback a second late: it still closes at full speed
    follower, headset, _, clock = make_follower(tmp_path, gripper=rested)
    headset.grip["r"] = True
    run(follower, clock, 50)
    headset.trigger["r"] = 1.0
    run(follower, clock, 20)   # under squeeze_after
    assert rested.sent[-1] < 0.5

    holding = Gripper(opening=0.1)   # a new demonstration starts from where the jaws are
    follower, headset, _, clock = make_follower(tmp_path, gripper=holding)
    assert follower.gripper.closure == pytest.approx(0.9)
    headset.grip["r"], headset.trigger["r"] = True, 1.0
    run(follower, clock, 5)
    assert max(holding.sent) <= 0.1


def test_a_faulty_gripper_is_dropped(tmp_path):
    follower, headset, arm, clock = make_follower(tmp_path, gripper=Gripper(fault="not calibrated"))
    assert arm.mode == "follow" and "not calibrated" in follower.gripper.fault
    headset.trigger["r"] = 1.0
    assert move(follower, headset, clock, 0.03 * TABLE["forward"])[0] == pytest.approx(0.03, abs=4e-3)


@pytest.mark.parametrize("change, why", [({"posture": np.where(np.arange(7) == 3, 4.0, BENT)}, "start_posture puts J3 outside"), ({"speed_scale": 1.5}, "speed_scale is 1.5"), ({"double_press_action": "jump"}, "double_press_action is 'jump'")])
def test_bad_settings(tmp_path, change, why):
    follower, *_ = make_follower(tmp_path)
    settings = ArmSettings(**{**vars(follower.settings), **{k: v for k, v in change.items() if k == "speed_scale"}})
    with pytest.raises(ValueError, match=why):
        ArmFollower("right", follower.arm, follower.ik, follower.controller, follower.guards, settings, change.get("posture", BENT), double_press_action=change.get("double_press_action", "home"))


class Returning(FakeArm):
    # An arm whose setpoint is still on its way: the SDK's command has a velocity until `arrived`.
    arrived = False

    def read_state(self):
        state = super().read_state()
        if state.command is not None and not self.arrived:
            state.command.velocity = np.ones(7)
        return state


def test_a_return_stopped_midway(tmp_path):
    arm = Returning()
    follower, headset, _, clock = make_follower(tmp_path, arm=arm)
    move(follower, headset, clock, 0.06 * TABLE["forward"])
    headset.grip["r"] = False
    run(follower, clock, 5)
    follower.return_to_start()
    run(follower, clock, 5)
    assert follower.returning   # the setpoint is still moving
    before = len(arm.sent)
    run(follower, clock, 3, lambda i: headset.grip.update(r=True))   # a press stops it where it is, at rest
    assert not follower.returning and any(not v.any() for _, v in arm.sent[before:])
    assert arm.speed == pytest.approx(np.full(7, follower.settings.speed_scale * SPEED))   # back to full speed


def test_a_pose_that_is_not_a_number(tmp_path):
    follower, headset, _, clock = make_follower(tmp_path)
    headset.grip["r"] = True
    run(follower, clock, 5)
    held = hand(follower)[0]
    good = headset.pose["r"].copy()
    headset.pose["r"][:3, 3] = np.nan
    run(follower, clock, 3)
    assert np.isfinite(follower.q).all() and hand(follower)[0] == pytest.approx(held, abs=1e-3)   # the hand holds
    headset.pose["r"] = good   # and follows again once the pose is a number
    assert move(follower, headset, clock, 0.03 * TABLE["forward"])[0] == pytest.approx(0.03, abs=4e-3)


def test_the_elbow_follows_the_operators(tmp_path):
    # The operator's arm: shoulder below and beside the head, forearm forward at the first grip; then the controller turns about the vertical, which swings their elbow out to one side. No body boxes, so nothing crowds the swing.
    shoulder = np.array([-0.05, -0.18, -0.22])
    at_hand = shoulder + np.array([0.27, 0.0, -0.30])
    turn = np.radians(-40.0)
    elbow = at_hand - 0.27 * np.array([np.cos(turn), np.sin(turn), 0.0])
    line = (at_hand - shoulder) / np.linalg.norm(at_hand - shoulder)
    out = (elbow - shoulder) - ((elbow - shoulder) @ line) * line
    want = out / np.linalg.norm(out)

    def swing(on):
        follower, headset, _, clock = make_follower(tmp_path, worn=True, elbow_follow=on)
        follower.guards = Guards(follower.ik, "J0", "J3", GuardSettings())
        headset.pose["r"][:3, 3] = at_hand
        headset.grip["r"] = True
        run(follower, clock, 5)   # the first grip: the forearm's direction is taken
        run(follower, clock, 600, lambda i: headset.turn("r", turn / 50 * WORN["up"]) if i < 50 else None)   # 12 s: it settles over about 10
        arm = follower.points()
        axis = (arm.wrist - arm.shoulder) / np.linalg.norm(arm.wrist - arm.shoulder)
        radial = (arm.elbow - arm.shoulder) - ((arm.elbow - arm.shoulder) @ axis) * axis
        want_here = want - (want @ axis) * axis
        cosine = radial @ want_here / (np.linalg.norm(radial) * np.linalg.norm(want_here))
        return np.arccos(np.clip(cosine, -1.0, 1.0)), hand(follower)[0]

    off, hand_off = swing(False)
    on, hand_on = swing(True)
    assert on < 0.05 and off > 0.5                                # the robot's elbow turned toward the operator's
    assert hand_on == pytest.approx(hand_off, abs=5e-3)          # through the spare freedom: the hand stays
