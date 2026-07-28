import os
import pathlib as pl
from unittest.mock import MagicMock

import pytest
import numpy as np

from swarmsim.world.simulate import main
from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.Agent import Agent
from swarmsim.sensors.Sensor import Sensor
from ...helpers import load_custom_yaml


@pytest.fixture
def agent():
    return MagicMock(spec=Agent)


def test_static_controller(agent):
    from swarmsim.agent.control.StaticController import StaticController
    static_controller = StaticController(agent=agent, output=(1.5, 2.3))
    assert np.array_equal(static_controller.get_actions(agent), (1.5, 2.3))

    static_controller = StaticController(agent=agent)
    assert np.array_equal(static_controller.get_actions(agent), (0.0, 0.0))


@pytest.fixture
def binary_sensor_active():
    sensor = MagicMock(spec=Sensor)
    sensor.current_state = 1
    return sensor


@pytest.fixture
def binary_sensor_inactive():
    sensor = MagicMock(spec=Sensor)
    sensor.current_state = 0
    return sensor


def test_binary_controller(agent, binary_sensor_active, binary_sensor_inactive):
    from swarmsim.agent.control.BinaryController import BinaryController

    agent.sensors = [binary_sensor_active, binary_sensor_inactive]
    static_controller = BinaryController((1.5, 2.3), (3.0, 4.0), parent=agent)
    assert np.array_equal(static_controller.get_actions(agent), (3.0, 4.0))

    static_controller = BinaryController((1.5, 2.3), (3.0, 4.0), parent=agent, sensor_id=1)
    assert np.array_equal(static_controller.get_actions(agent), (1.5, 2.3))

    static_controller = BinaryController(((1.5, 2.3), (3.0, 4.0)), parent=agent)
    assert np.array_equal(static_controller.get_actions(agent), (3.0, 4.0))


wd = pl.Path(__file__).parent.parent.parent
path = wd / "sensors" / "configs" / "180degFOV"
yaml_files = tuple(path.glob("*.yaml"))


@pytest.mark.parametrize("yaml_path", yaml_files, ids=lambda x: x.stem)
def test_binary_controller_yaml(yaml_path: str) -> None:
    from swarmsim.agent.control.BinaryController import BinaryController
    _, world_setup = load_custom_yaml(yaml_path)
    world_config = RectangularWorldConfig(**world_setup)
    world = RectangularWorld(world_config)
    world.setup()

    assert len(world.population) >= 1
    agent1: Agent = world.population[0]
    assert agent1.name == "agent1"

    assert len(agent1.sensors) == 1
    sensor = agent1.sensors[0]
    assert sensor.as_config_dict()["type"] == "BinaryFOVSensor"

    on_see = [0.02, -0.5]
    on_nothing = [0.02, 0.5]
    agent1.controller = BinaryController(on_nothing, on_see)

    world.step()
    detected = sensor.current_state == 1
    actions = agent1.controller.get_actions(agent1)

    if detected:
        assert (actions == on_see).all(), f"{actions} != {on_see}"
    else:
        assert (actions == on_nothing).all()
