from os import PathLike
import pathlib as pl
import pytest

from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.world.simulate import main
from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.Agent import Agent
from ..util import load_custom_yaml


def stop_after_one_step(world: RectangularWorld) -> bool:
    return world.total_steps == 1


def setup_common_world(world_setup: dict) -> RectangularWorld:
    world_config = RectangularWorldConfig(**world_setup)
    world = RectangularWorld(world_config)
    main(world, show_gui=False, stop_detection=stop_after_one_step)

    # > Did world step only once?
    assert world.total_steps == 1

    # > How many agents are there?
    assert len(world.population) >= 2

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

binary_fov_yaml_files = (path / "BinaryFOV").glob("*.yaml")


@pytest.mark.parametrize("yaml_path", binary_fov_yaml_files, ids=lambda x: x.stem)
def test_yaml_file(yaml_path: PathLike):
    spec, world_setup = load_custom_yaml(yaml_path)
    world: RectangularWorld = setup_common_world(world_setup)
    bfovs: BinaryFOVSensor = setup_common_agent(world)

    collided = bfovs.current_state == 1
    assert collided == spec["expected"]


large_fov_yaml_files = (path / "180degFOV").glob("*.yaml")


@pytest.mark.parametrize("yaml_path", large_fov_yaml_files, ids=lambda x: x.stem)
def test_180degFOV_yaml_file(yaml_path: PathLike):
    spec, world_setup = load_custom_yaml(yaml_path)

    # Setup world
    world_config = RectangularWorldConfig(**world_setup)
    agent1: Agent = world_config.agents[0]
    agents: list[Agent] = []
    for agent in world_config.agents[1:]:
        agents.append(agent.copy())

    for agent in agents:
        world_config.agents = [agent1, agent]
        world = RectangularWorld(world_config)
        world.setup()

        agent1 = world.population[0]
        other_agent_name = world.population[1].name
        # > Does this agent have a sensor?
        assert len(agent1.sensors) == 1
        bfov = agent1.sensors[0]
        # > Is this agent's only sensor a BinaryFOVSensor?
        assert bfov.as_config_dict()["type"] == "BinaryFOVSensor"

        main(world, show_gui=False, start_paused=False, stop_detection=stop_after_one_step)
        detected = bfov.current_state == 1
        assert detected == spec[f"canSee_{other_agent_name}"], (
            f"{other_agent_name}: exp: {spec[f"canSee_{other_agent_name}"]}, got: {detected}"
        )
