"""
This class defines a configuration for how the World will output a large numpy array representing the
pixels on the screen
"""

from dataclasses import dataclass


@dataclass
class OutputTensorConfig:
    screen = None
    total_frames: int = 5
    steps_between_frames: int = 4
    timeless: bool = False
    colored: bool = False
    background_color: tuple[int, int, int] = (0, 0, 0)
    step: int | None = None
