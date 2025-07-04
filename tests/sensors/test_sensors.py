from os import PathLike
import pathlib as pl
import pytest
from ..util import load_custom_yaml

from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.world.simulate import main
from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.Agent import Agent


def stop_after_one_step(world: RectangularWorld) -> bool:
    return world.total_steps == 1


def setup_common_world(world_setup: dict) -> RectangularWorld:
    world_config = RectangularWorldConfig(**world_setup)
    world = RectangularWorld(world_config)
    main(world, show_gui=False, stop_detection=stop_after_one_step)

    # > Did world step only once?
    assert world.total_steps == 1

    # > How many agents are there?
    assert len(world.population) == 2

    return world


def setup_common_agent(world: RectangularWorld) -> BinaryFOVSensor:
    # > How many agents are there?
    assert len(world.population) >= 1

    # > Is the first agent the "test-dummy" agent?
    agent1: Agent = world.population[0]
    assert agent1.name == "agent1"

    # > Does this agent have a sensor?
    assert len(agent1.sensors) == 1
    sensor = agent1.sensors[0]

    # > Is this agent's only sensor a BinaryFOVSensor?
    assert sensor.as_config_dict()["type"] == "BinaryFOVSensor"

    return sensor


wd = pl.Path(__file__).parent
path = wd / "configs"
yaml_files = path.glob("*.yaml")


@pytest.mark.parametrize("yaml_path", yaml_files, ids=lambda x: x.stem)
def test_yaml_file(yaml_path: PathLike):
    spec, world_setup = load_custom_yaml(yaml_path)
    world: RectangularWorld = setup_common_world(world_setup)
    bfovs: BinaryFOVSensor = setup_common_agent(world)

    collided = bfovs.current_state == 1
    assert collided == spec['expected']
