# What every headset reader hands the loop: a `Reading`, from `latest()`.
#
# Readers also have `close()`. One whose tracking frame can move (the worn headset) has `epoch`, which changes with it.
from typing import Dict, NamedTuple

import numpy as np

# Button names, as the Quest app gives them. A, B, X and Y are the face buttons (True or False).
GRIP = {"r": "RG", "l": "LG"}                    # grip held
STICK_PRESS = {"r": "RJ", "l": "LJ"}             # stick pressed down
TRIGGER = {"r": "rightTrig", "l": "leftTrig"}    # index trigger, a 1-tuple in 0..1
STICK = {"r": "rightJS", "l": "leftJS"}          # stick as (right, up), each in -1..1
HEAD = "head"                                    # the worn headset's own pose, in `Reading.poses`


class Reading(NamedTuple):
    # One sample from the headset.
    #
    # Poses are 4x4: in the table headset's tracking frame (y up), or the robot's axes (x forward, y left, z up) if worn.
    poses: Dict[str, np.ndarray]   # by controller ("l", "r"), plus HEAD when worn; one not tracked is missing
    buttons: dict
    age: float                     # s since the sample was taken; infinite before the first
