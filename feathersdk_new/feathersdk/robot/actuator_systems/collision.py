"""Whether an arm's motion would bring it into the torso or the base, from the URDF's collision meshes: the arm's
links as their convex hulls (an arm link is nearly convex), the body as its meshes simplified (the base is concave).

The body is every link the torso's joint doesn't carry on a further joint: the base and its wheels, the column, and
what is fixed to the column (the arms' mounts, the neck's base). The arms and the neck move on their own and aren't part
of it (arm against arm isn't checked). A motion is refused when it brings the arm within `margin` of the body, or
closer to it than it already is: an arm already that close (the torso lowered it, or the check was turned on there) can
always move away. Once the meshes touch, the distance is 0 however deep: a move deeper in can't be told from one
along the surface, and is let through like any move that isn't closer."""
import numpy as np

from feathersdk.utils.feathertypes import Any, Iterable, List, NamedTuple, Optional, Sequence, Tuple


class CollisionError(ValueError):
    """A motion refused because it would bring an arm into the torso or the base. `contact` is the closest pair."""

    def __init__(self, message: str, contact: "Contact") -> None:
        super().__init__(message)
        self.contact = contact


class Contact(NamedTuple):
    """The closest arm link and body link within the margin, and how far apart (m; 0 once the meshes touch)."""
    arm_link: str
    body_link: str
    distance: float


class BodyCheck:
    """One arm's links against the body, at the arm's motor positions and the torso's URDF position. Not
    thread-safe: one per arm, used by one thread at a time (the arm's command calls are)."""

    def __init__(self, pin: Any, model: Any, hulls: Any, coarse: Any, arm_joints: Sequence[str],
                 directions: Sequence[int], torso_joint: str, margin: float, fixed_only: bool = False) -> None:
        """`model` is the URDF's Pinocchio model; `hulls` and `coarse` its collision geometry, the same objects in the
        same order, with the meshes as convex hulls and as simplified meshes (`URDFLoader.pin_collision_hulls` and
        `pin_collision_coarse`); `arm_joints` the arm's URDF joints in the order its motor positions come in,
        `directions` each one's `urdf_direction`; `margin` (m) how close the arm may come to the body. `fixed_only`:
        only the body the torso doesn't carry (the base, the wheels, the inner column), what `lowered()` asks about.

        Raises:
            ValueError: A joint isn't in the URDF, an arm joint isn't carried by the torso joint, the margin isn't
                positive, or no pair is left to check.
        """
        if not margin > 0:
            raise ValueError(f"margin must be above 0 m, got {margin}")
        ids = {}
        for name in (*arm_joints, torso_joint):
            if not model.existJointName(name):
                raise ValueError(f"Joint {name!r} isn't in the URDF")
            ids[name] = model.getJointId(name)
        torso = ids[torso_joint]
        carried = set(model.subtrees[torso]) - {torso}  # the joints on the torso: the arms', the neck's
        if not {ids[name] for name in arm_joints} <= carried:
            raise ValueError(f"The arm joints {list(arm_joints)} aren't all carried by the torso joint {torso_joint!r}")
        arm = {j for name in arm_joints for j in model.subtrees[ids[name]]}  # with the links fixed to the hand
        objects = hulls.geometryObjects
        arm_geoms = [i for i, g in enumerate(objects) if g.parentJoint in arm]
        moved = carried | {torso} if fixed_only else carried  # what the torso moves with the arm: no nearer for it
        body_geoms = [i for i, g in enumerate(objects) if g.parentJoint not in moved]

        self._pin, self._model, self._data = pin, model, model.createData()
        self._geometry = hulls.copy()  # its own pairs and geometry list; the shapes themselves are shared
        for i in body_geoms:
            self._geometry.geometryObjects[i].geometry = coarse.geometryObjects[i].geometry
        self._geometry.removeAllCollisionPairs()
        for a in arm_geoms:
            for b in body_geoms:
                self._geometry.addCollisionPair(pin.CollisionPair(a, b))
        self._q = pin.neutral(model)  # the joints not set below (the neck, the steering) stay at their zero
        self._arm_q = np.array([model.joints[ids[n]].idx_q for n in arm_joints])
        self._directions = np.array(directions, dtype=np.float64)
        self._torso_q = model.joints[torso].idx_q
        names = [model.frames[g.parentFrame].name for g in objects]

        # Pairs already within the margin at the zero pose, the torso at its top (furthest from the base), are links
        # where they join (the shoulder on its mount, on the column): never checked, or the arm could hardly move from
        # home. So are the arm's first link's at the torso's bottom too: it only turns on its mount (the
        # shoulder is 26 mm from the inner column there, confirmed). Not the rest at the bottom: the hanging forearm
        # reaches the base, and the arms must bend forward for the torso to go all the way down (confirmed). Kept as
        # indices, not the pair objects: those are views into the model's list, which shifts as pairs are removed.
        zero = self._q.copy()
        zero[self._torso_q] = model.upperPositionLimit[self._torso_q]
        pairs = self._geometry.collisionPairs
        overlapping = [(pairs[k].first, pairs[k].second) for k in self._colliding(zero, margin)]
        zero[self._torso_q] = model.lowerPositionLimit[self._torso_q]
        first = {i for i in arm_geoms if objects[i].parentJoint == ids[arm_joints[0]]}
        overlapping += [(pairs[k].first, pairs[k].second) for k in self._colliding(zero, margin)
                        if pairs[k].first in first and (pairs[k].first, pairs[k].second) not in overlapping]
        self.skipped = [(names[a], names[b]) for a, b in overlapping]
        """The (arm link, body link) pairs never checked, for the log."""
        for a, b in overlapping:
            self._geometry.removeCollisionPair(pin.CollisionPair(a, b))
        if len(self._geometry.collisionPairs) == 0:
            raise ValueError(f"No arm link left to check against the body for {list(arm_joints)}")
        self._data_geometry = pin.GeometryData(self._geometry)
        for request in self._data_geometry.collisionRequests:
            request.security_margin = margin
        self._pairs = [(names[p.first], names[p.second]) for p in self._geometry.collisionPairs]
        self.margin = margin
        # The last contact() and what it was asked: a direct setpoint's "before" is the previous one's "now", and near
        # the body a contact() takes milliseconds (its distances), so `refused()` would compute it twice a tick.
        self._last: Optional[tuple] = None

    def _colliding(self, q: np.ndarray, margin: float) -> List[int]:
        """The indices of the pairs within `margin` at `q` (used once, to find the pairs never checked)."""
        data = self._pin.GeometryData(self._geometry)
        for request in data.collisionRequests:
            request.security_margin = margin
        self._pin.computeCollisions(self._model, self._data, self._geometry, data, q, False)
        return [k for k, result in enumerate(data.collisionResults) if result.isCollision()]

    def contact(self, arm_q: np.ndarray, torso_q: float) -> Optional[Contact]:
        """The closest pair within the margin at these positions, or None when the arm is clear of the body."""
        key = (np.asarray(arm_q, dtype=np.float64).tobytes(), torso_q)
        if self._last is not None and self._last[0] == key:
            return self._last[1]
        closest = self._contact(arm_q, torso_q)
        self._last = (key, closest)
        return closest

    def _contact(self, arm_q: np.ndarray, torso_q: float) -> Optional[Contact]:
        q = self._q
        q[self._arm_q] = arm_q * self._directions
        q[self._torso_q] = torso_q
        pin, geometry, data = self._pin, self._geometry, self._data_geometry
        if not pin.computeCollisions(self._model, self._data, geometry, data, q, False):
            return None  # the common case: only the collision test, no distances
        closest = None
        for k, result in enumerate(data.collisionResults):
            if result.isCollision():
                distance = max(float(pin.computeDistance(geometry, data, k).min_distance), 0.0)
                if closest is None or distance < closest.distance:
                    closest = Contact(*self._pairs[k], distance)
        return closest

    def lowered(self, arm_q: np.ndarray, torso_q: float, lower_q: float) -> Optional[Contact]:
        """The contact that refuses the torso going from `torso_q` down to `lower_q` with the arm at `arm_q`, or None:
        it comes within the margin there, closer than here, or the meshes touch there (going down, deeper in)."""
        there = self.contact(arm_q, lower_q)
        if there is None:
            return None
        here = self.contact(arm_q, torso_q)
        return there if here is None or there.distance < here.distance or there.distance == 0.0 else None

    def lowest(self, arm_q: np.ndarray, torso_q: float, lower_q: float, resolution: float = 0.005
               ) -> Optional[Tuple[float, Contact]]:
        """How far the torso may go from `torso_q` down toward `lower_q` with the arm at `arm_q` (`lowered()`): None
        for all the way, or the lowest position it may (to `resolution`, m, above the first refused one) and the
        contact there. By bisection, so a clearance that shrinks steadily as the torso goes down (the arm toward the
        base); a dip between is found by the torso's own check on the way down, which halts it."""
        refused = self.lowered(arm_q, torso_q, lower_q)
        if refused is None:
            return None
        allowed = torso_q
        while allowed - lower_q > resolution:
            middle = (allowed + lower_q) / 2.0
            contact = self.lowered(arm_q, torso_q, middle)
            if contact is None:
                allowed = middle
            else:
                lower_q, refused = middle, contact
        return allowed, refused

    def refused(self, start: Optional[np.ndarray], path: Iterable[np.ndarray], torso_q: float) -> Optional[Contact]:
        """The contact that refuses moving from `start` (None: from nowhere known) through each position of `path`,
        or None if the motion may go: no position brings the arm within the margin, or closer than the position
        before it was."""
        before = None if start is None else self.contact(start, torso_q)
        for q in path:
            now = self.contact(q, torso_q)
            if now is not None and (before is None or now.distance < before.distance):
                return now
            before = now
        return None
