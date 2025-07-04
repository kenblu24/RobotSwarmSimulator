import pathlib as pl

from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.world.simulate import main as sim
from swarmsim.agent.control.StaticController import StaticController
from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.agent.control.BinaryController import BinaryController

# TODO: continue from the sensors and controllers section

# Global variables used in the code blocks
# These have been initialized with default values
world_config: RectangularWorldConfig = RectangularWorldConfig()
world: RectangularWorld = RectangularWorld(world_config)
agent_config: MazeAgentConfig = MazeAgentConfig()
agent: MazeAgent = MazeAgent(agent_config, world)
controller: StaticController = StaticController()


def code_block_simulate(w: RectangularWorld | RectangularWorldConfig, limit_steps: bool = True) -> None:
    """ Starting the simulation """
    def stop_after_n_frames(world: RectangularWorld, n_frames: int = 500):
        return world.total_steps == n_frames

    if limit_steps:
        sim(world, show_gui=False, stop_detection=stop_after_n_frames)
    else:
        sim(world, show_gui=False)


def code_block_01() -> None:
    """ Creating a world """
    world_config = RectangularWorldConfig(size=[10, 10], time_step=1 / 40)  # pyright: ignore[reportArgumentType]
    world = RectangularWorld(world_config)

    """ Creating an agent """
    agent_config = MazeAgentConfig(position=(5, 5), agent_radius=0.1)
    agent = MazeAgent(agent_config, world)

    world.population.append(agent)  # add the agent to the world

    """ Starting the simulation """
    code_block_simulate(world)

    """ Adding a controller """
    controller = StaticController(output=[0.01, 0.1])  # 10 cm/s forwards, 0.1 rad/s clockwise.
    agent.controller = controller
    code_block_simulate(world)

    """ Spawners """
    spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True, agent=agent, mode="oneshot")
    world.spawners.append(spawner)

    del world.population[-1]
    code_block_simulate(world)  # simulate world

    """ Sensors & Controllers """
    sensor = BinaryFOVSensor(agent, theta=0.45, distance=2,)
    agent.sensors.append(sensor)

    controller = BinaryController((0.02, -0.5), (0.02, 0.5), agent)
    agent.controller = controller

    del world.population[:]  # Delete all agents
    spawner.mark_for_deletion = False  # Re-enable the spawner
    world.spawners.append(spawner)
    code_block_simulate(world)  # simulate world


def code_block_02() -> None:
    world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
    world = RectangularWorld(world_config)
    controller = StaticController(output=[0.01, 0])
    agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1,
                                      controller=controller), world)
    spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True,
                                agent=agent, mode="oneshot")
    world.spawners.append(spawner)

    code_block_simulate(world)  # simulate world


def code_block_03() -> None:
    wd = pl.Path(__file__).parent
    world_config = RectangularWorldConfig.from_yaml(wd / "tutorial_world.yaml")

    code_block_simulate(world_config)


def test_01() -> None:
    code_block_01()
    code_block_02()
    code_block_03()
