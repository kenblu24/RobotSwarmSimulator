from enum import Flag, auto
import numpy as np


class CollisionMode(Flag):
    Disabled = 0
    Detect = 1
    Correct = 2
    DetectAndCorrect = Detect | Correct
    Invalid = Correct


CollisionMode_T = CollisionMode | int | str


def collision_mode(mode: CollisionMode_T, error='raise') -> CollisionMode_T:
    if isinstance(mode, str):
        mode = CollisionMode[mode]
    elif isinstance(mode, (int, np.integer)):
        mode = CollisionMode(mode)
    else:
        mode = mode
    if error == 'raise' and mode == CollisionMode.Invalid:
        raise ValueError("Collider created with Invalid Collision Mode")
    return mode
