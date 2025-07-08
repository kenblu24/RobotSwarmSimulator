import os
import pathlib as pl
import pytest

from swarmsim.agent.control.StaticController import StaticController
from swarmsim.agent.control.BinaryController import BinaryController
from swarmsim.world.simulate import main
from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.Agent import Agent
from ...util import load_custom_yaml


def stop_after_n_steps(world: RectangularWorld, n_frames: int = 5) -> bool:
    return world.total_steps == n_frames


wd = pl.Path(__file__).parent.parent
path = wd / "sensors" / "configs"
yaml_files = path.glob("*.yaml")

@pytest.mark.parametrize("yaml_path", yaml_files, ids=lambda x: x.stem)
def setup_tester(yaml_path: str, binary: bool = True) -> bool:
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

    on_see: tuple[float, float] = (0.02, -0.5)
    on_nothing: tuple[float, float] = (0.02, 0.5)
    const_output: tuple[float, float] = (0.01, 0.1)
    if binary:
        agent1.controller = BinaryController(on_see, on_nothing)
    else:
        agent1.controller = StaticController(output=const_output)

    main(world, show_gui=False, stop_detection=stop_after_n_steps)
    detected = sensor.current_state == 1
    actions = agent1.controller.get_actions(agent1)

    result: bool = False
    if binary:
        if detected:
            result = actions == on_see
        else:
            result = actions == on_nothing
    else:
        result = actions == const_output

    return result


