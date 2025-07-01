import os

from src.swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from src.swarmsim.world.simulate import main
from src.swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from src.swarmsim.agent.Agent import Agent
from src.swarmsim import yaml as ssyaml

def stop_after_one_step(world: RectangularWorld) -> bool:
    return world.total_steps == 1

def load_custom_yaml(path: str) -> tuple[bool, dict]:
    with open(path, "r") as yf:
        d = ssyaml.load(yf)
        return (d["expected"], d["world_setup"])

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

def setup_tester(yaml_path: str) -> bool:
    expected_collided, world_setup = load_custom_yaml(yaml_path)
    world: RectangularWorld = setup_common_world(world_setup)
    bfovs: BinaryFOVSensor = setup_common_agent(world)

    collided = bfovs.current_state == 1
    return collided == expected_collided

def test_setups():
    dir: str = "vision_setup/"

    for file in os.listdir(dir):
        if file.endswith(".yaml"):
            yaml_path: str = os.path.join(dir, file)
            print(yaml_path)
            result = setup_tester(yaml_path)
            assert result, f"[ERROR] Failed on '{yaml_path}'"
