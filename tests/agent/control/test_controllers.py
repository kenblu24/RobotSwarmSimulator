import os
import pathlib as pl

import pytest

from swarmsim.agent.control.StaticController import StaticController
from swarmsim.agent.control.BinaryController import BinaryController
from swarmsim.world.simulate import main
from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.Agent import Agent
from ...util import load_custom_yaml



wd = pl.Path(__file__).parent.parent.parent
path = wd / "sensors" / "configs"
yaml_files = path.glob("*.yaml")

@pytest.mark.parametrize("yaml_path", yaml_files, ids=lambda x: x.stem)
def test_binary_controller(yaml_path: str) -> None:
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
    agent1.controller = BinaryController(on_nothing, on_see) # type: ignore

    world.step()
    detected = sensor.current_state == 1
    actions = agent1.controller.get_actions(agent1)

    if detected:
        assert (actions == on_see).all(), f"{actions} != {on_see}"
    else:
        assert (actions == on_nothing).all()



