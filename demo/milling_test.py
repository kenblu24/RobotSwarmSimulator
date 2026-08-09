import pytest

# from https://kenblu24.github.io/RobotSwarmSimulator/guide/firstrun.html

def test_milling(show_gui=False):
    from swarmsim import RectangularWorld, RectangularWorldConfig
    from swarmsim import MazeAgent, MazeAgentConfig
    from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
    from swarmsim.agent.control.BinaryController import BinaryController
    from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
    from swarmsim.metrics import Circliness
    from swarmsim import run_sim

    world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40, stop_at=600)
    world = RectangularWorld(world_config)
    agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1), world)
    sensor = BinaryFOVSensor(agent, theta=0.45, distance=2,)
    agent.sensors.append(sensor)
    controller = BinaryController((0.27, -0.6), (0.27, 0.6), agent)
    agent.controller = controller
    spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True,
                                agent=agent, mode="oneshot")
    world.spawners.append(spawner)
    world.metrics.append(Circliness(avg_history_max=100))
    world.metrics[0].attach_world(world)

    run_sim(world, show_gui=False)

    assert world.metrics[0].out_current()[1] > 0.5


if __name__ == "__main__":
    test_milling(show_gui=True)
