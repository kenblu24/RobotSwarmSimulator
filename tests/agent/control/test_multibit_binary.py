import unittest.mock as mock

from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.agent.control.MultibitBinaryController import MultibitBinaryController
from swarmsim.agent.MazeAgent import MazeAgent


def check_mbc_agent(controller: MultibitBinaryController, sens_values: list[bool], expected: tuple[float, float]):
    agent = mock.MagicMock(spec=MazeAgent)

    # Ensure controller sensor id list matches length of sensor values
    assert len(sens_values) == len(controller.sensor_ids)
    agent.sensors = [mock.MagicMock(spec=BinaryFOVSensor) for _ in sens_values]
    for i, sv in enumerate(sens_values):
        agent.sensors[i].current_state = sv

    assert isinstance(controller, MultibitBinaryController)
    agent.controller = controller
    assert controller.get_actions(agent) == expected


def test_mbc_01():
    controller = MultibitBinaryController(
        outputs={
            0b00: (0.0, 0.0),
            0b01: (0.0, 1.0),
            0b10: (1.0, 0.0),
            0b11: (1.0, 1.0),
        },
        default_output=(0.5, 0.5),
        sensor_ids=[0, 1]
    )

    check_mbc_agent(controller, [False, False], (0.0, 0.0))
    check_mbc_agent(controller, [True, False], (0.0, 1.0))
    check_mbc_agent(controller, [False, True], (1.0, 0.0))
    check_mbc_agent(controller, [True, True], (1.0, 1.0))


def test_mbc_02():
    controller = MultibitBinaryController(
        outputs={
            0b00: (0.0, 0.0),
            0b01: (0.0, 1.0),
            0b10: (1.0, 0.0),
            # Intentionally skipping this value
            # 0b11: (1.0, 1.0),
        },
        default_output=(0.5, 0.5),
        sensor_ids=[0, 1]
    )

    check_mbc_agent(controller, [False, False], (0.0, 0.0))
    check_mbc_agent(controller, [True, False], (0.0, 1.0))
    check_mbc_agent(controller, [False, True], (1.0, 0.0))
    check_mbc_agent(controller, [True, True], (0.5, 0.5))
