import pytest


def test_uniform_circle_agent_spawner():
    from swarmsim import RectangularWorld
    from swarmsim import RectangularWorldConfig
    from swarmsim.world.simulate import main as sim
    from swarmsim.world.spawners.AgentSpawner import UniformCircleAgentSpawner
    from swarmsim import StaticAgentConfig


    # world
    world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40, stop_at=2)
    world = RectangularWorld(world_config)

    agent = StaticAgentConfig()

    # spawner
    spawner = UniformCircleAgentSpawner(world, center=[1, 1], radius=10, n=100, facing="away", avoid_overlap=True, agent=agent, mode="oneshot")
    world.spawners.append(spawner)


    sim(world, show_gui=False, stop_detection=1, start_paused=False)

