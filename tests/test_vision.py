from src.swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from src.swarmsim.world.simulate import main
from src.swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from src.swarmsim.agent.Agent import Agent

def stop_after_one_step(world: RectangularWorld) -> bool:
    return world.total_steps == 1

def setup_common_world(path: str) -> RectangularWorld:
    world_config = RectangularWorldConfig.from_yaml(path)
    world = RectangularWorld(world_config)
    main(world, show_gui=False, stop_detection=stop_after_one_step)

    # > Did world step only once?
    assert world.total_steps == 1

    # > How many agents are there?
    assert len(world.population) == 2

    return world

def setup_common_agent(world: RectangularWorld) -> Agent:
    # > Did any agent (w/ name "agent1") see the other agent?
    agent1 = world.population[0]
    ## >> Does the first agent have a name of "agent1"?
    assert agent1.name == "agent1"
    ## >> Does this agent have only one sensor?
    assert len(agent1.sensors) == 1

    return agent1

def test_setup01():
    world: RectangularWorld = setup_common_world("yaml/setup01.yaml")
    agent1: Agent = setup_common_agent(world)

    ## >> Is this agent's sensor a BinaryFOVSensor?
    sensor = agent1.sensors[0]
    assert sensor.as_config_dict()["type"] == "BinaryFOVSensor"
    ## >> Did the sensor detect anything?
    bfovsensor: BinaryFOVSensor = agent1.sensors[0]
    assert bfovsensor.current_state == 1

def test_setup02():
    world: RectangularWorld = setup_common_world("yaml/setup02.yaml")

    # > Did any agent (w/ name "agent1") see the other agent?
    agent1 = world.population[0]
    ## >> Does the first agent have a name of "agent1"?
    assert agent1.name == "agent1"
    ## >> Does this agent have only one sensor?
    assert len(agent1.sensors) == 1
    ## >> Is that sensor a BinaryFOVSensor
    sensor = agent1.sensors[0]
    assert sensor.as_config_dict()["type"] == "BinaryFOVSensor"
    ## >> Did the sensor detect anything?
    bfovsensor: BinaryFOVSensor = agent1.sensors[0]
    assert bfovsensor.current_state == 0
