import swarmsim


from swarmsim.agent.control.MultibitBinaryController import MultiBinaryController
from swarmsim.agent.MazeAgent import MazeAgent
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor


def test_init():
    agent = MazeAgent()
    controller = MultiBinaryController(
        outputs={
            0b00: (0.0, 0.0),
            0b01: (0.0, 1.0),
            0b10: (1.0, 0.0),
            0b11: (1.0, 1.0),
        },
        default_output=(0.5, 0.5),
        sensor_ids=[0, 1]
    )
    agent.controller = controller
    controller.set_agent(agent)
    agent.sensors.append(BinaryFOVSensor(agent, 0, fov=1.0))
    agent.sensors.append(BinaryFOVSensor(agent, 1, fov=1.0))
    assert len(agent.sensors) == 2
    assert agent.sensors[0].current_state == 0
    assert agent.sensors[1].current_state == 0
    assert agent.controller.get_actions(agent) == (0.5, 0.5)

    # test with default output
    controller = MultiBinaryController(
        outputs={
            0b00: (0.0, 0.0),
            0b01: (0.0, 1.0),
            0b10: (1.0, 0.0),
            0b11: (1.0, 1.0),
        },
        sensor_ids=[0, 1]
    )
