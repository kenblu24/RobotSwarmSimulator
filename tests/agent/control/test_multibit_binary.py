import unittest.mock as mock

from swarmsim.agent.control.MultibitBinaryController import MultiBinaryController
from swarmsim.agent.MazeAgent import MazeAgent


def check_mbc_agent(mbc: MultiBinaryController, sens_values: list[bool], expected: tuple[float, float]):
    agent = mock.MagicMock(spec=MazeAgent)

    # Ensure controller sensor id list matches length of sensor values
    assert len(sens_values) == len(mbc.sensor_ids)
    agent.sensors = [{
        "current_state": 1 if sens_val else 0
    } for sens_val in sens_values]
    assert isinstance(mbc, MultiBinaryController)
    agent.controller = mbc

    # Convert from a list of booleans into a single integer where each boolean
    # in the list is mapped to a bit in the integer
    # TODO: find a better way to do this
    index = int("".join("1" if b else "0" for b in sens_values), 2)
    if index in mbc.outputs:
        assert mbc.outputs[index] == expected
    else:
        assert mbc.default_output == expected


def test_mbc_01():
    mbc = MultiBinaryController(
        outputs={
            0b00: (0.0, 0.0),
            0b01: (0.0, 1.0),
            0b10: (1.0, 0.0),
            0b11: (1.0, 1.0),
        },
        default_output=(0.5, 0.5),
        sensor_ids=[0, 1]
    )

    check_mbc_agent(mbc, [False, False], (0.0, 0.0))
    check_mbc_agent(mbc, [False, True] , (0.0, 1.0))
    check_mbc_agent(mbc, [True,  False], (1.0, 0.0))
    check_mbc_agent(mbc, [True,  True] , (1.0, 1.0))


def test_mbc_02():
    mbc = MultiBinaryController(
        outputs={
            0b00: (0.0, 0.0),
            0b01: (0.0, 1.0),
            0b10: (1.0, 0.0),
            # 0b11: (1.0, 1.0),
        },
        default_output=(0.5, 0.5),
        sensor_ids=[0, 1]
    )

    check_mbc_agent(mbc, [False, False], (0.0, 0.0))
    check_mbc_agent(mbc, [False, True] , (0.0, 1.0))
    check_mbc_agent(mbc, [True,  False], (1.0, 0.0))
    check_mbc_agent(mbc, [True,  True] , (0.5, 0.5))

