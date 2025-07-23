from swarmsim.sensors.ObjectFOVSensor import sectorPointIntersect
import numpy as np
import pytest


# Source: https://www.desmos.com/calculator/vv2lloe5if
TEST_SET: list[dict] = [
    {
        "bias": 0.0,
        "theta": np.pi * 0.25,
        "points": [(2.5, 1.5), (2.65, 1.485), (2.93, 0.84), (3.2, 0.8), (2.066, 0.847)],
        "expected": [True, True, True, True, False]
    },
    {
        "bias": 0.0,
        "theta": np.pi * 0.75,
        "points": [(1.8, 0.32), (2.147, 1.935), (1.858, 1.0), (2.943, 0.942), (1.234, 1.55)],
        "expected": [True, True, False, True, False]
    },
]


@pytest.mark.parametrize(
    "theta,bias,points,expected", 
    [(x["theta"], x["bias"], x["points"], x["expected"]) for x in TEST_SET]
)
def test_sector_pt_intersect(
    theta: float, bias: float, points: list[tuple[float, float]], expected: list[bool]
) -> None:
    center = np.array((2, 1))
    radius: float = 1.0

    angle_left: float = bias - theta
    angle_right: float = bias + theta

    for pt, exp in zip(points, expected):
        np_pt = np.array(pt)
        result = sectorPointIntersect(center, radius, angle_left, angle_right, np_pt)
        assert result == exp
