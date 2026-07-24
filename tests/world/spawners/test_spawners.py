import pytest

import numpy as np


@pytest.fixture
def world():
    from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
    return RectangularWorld(RectangularWorldConfig(size=(5, 5)))


@pytest.fixture
def static_agent_config():
    from swarmsim.agent.StaticAgent import StaticAgentConfig
    return StaticAgentConfig()


valid_facings = (
    None,
    "towards",
    "away",
    "random",
    0,
    1.0,
    np.pi,
    [0],
    [1.0],
    [np.pi / 2],
    [0, 0],
    [100, 100],
    np.array([0, 0]),
)

@pytest.mark.parametrize("n", (0, 1, 5, 1000))
@pytest.mark.parametrize("facing", valid_facings)
def test_point_spawn(world, static_agent_config, facing, n):
    from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
    spawner = PointAgentSpawner(
        world, n=n, facing=facing, avoid_overlap=True,
        agent=static_agent_config,
        mode="oneshot",
    )
    world.spawners.append(spawner)
    assert len(world.population) == 0
    world.setup()
    assert len(world.population) == n


@pytest.mark.parametrize("n", (0, 1, 5, 1000))
@pytest.mark.parametrize("facing", valid_facings)
def test_uniform_box_spawn(world, static_agent_config, facing, n):
    from swarmsim.world.spawners.AgentSpawner import UniformAgentSpawner
    spawner = UniformAgentSpawner(
        world, n=n, facing=facing, avoid_overlap=True,
        region=[[0, 0], [0, 5], [5, 5], [5, 0]],
        agent=static_agent_config,
        mode="oneshot",
    )
    world.spawners.append(spawner)
    assert len(world.population) == 0
    world.setup()
    assert len(world.population) == n


@pytest.mark.parametrize("n", (0, 1, 5, 1000))
@pytest.mark.parametrize("facing", valid_facings)
def test_uniform_circle_spawn(world, static_agent_config, facing, n):
    from swarmsim.world.spawners.AgentSpawner import UniformCircleAgentSpawner
    spawner = UniformCircleAgentSpawner(
        world, n=n, facing=facing, avoid_overlap=True,
        center=[2.5, 2.5], radius=10,
        agent=static_agent_config, mode="oneshot"
    )
    world.spawners.append(spawner)
    assert len(world.population) == 0
    world.setup()
    assert len(world.population) == n
