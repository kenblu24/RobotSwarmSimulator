import pytest

# from https://kenblu24.github.io/RobotSwarmSimulator/guide/firstrun.html

def test_milling():
    from swarmsim import RectangularWorld, RectangularWorldConfig
    from swarmsim import MazeAgent, MazeAgentConfig
    from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
    from swarmsim.agent.control.BinaryController import BinaryController
    from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
    from swarmsim import run_sim

    world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40, stop_at=2)
    world = RectangularWorld(world_config)
    agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1), world)
    sensor = BinaryFOVSensor(agent, theta=0.45, distance=2,)
    agent.sensors.append(sensor)
    controller = BinaryController((0.27, -0.6), (0.27, 0.6), agent)
    agent.controller = controller
    spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True,
                                agent=agent, mode="oneshot")
    world.spawners.append(spawner)

    run_sim(world, show_gui=False)
