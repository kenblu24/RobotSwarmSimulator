import pytest


@pytest.fixture
def world():
    from swarmsim.world.World import World, BaseWorldConfig
    world = World(BaseWorldConfig())
    return world


@pytest.fixture
def rectangular_world():
    from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
    world_config = RectangularWorldConfig()
    world = RectangularWorld(world_config)
    return world


def test_pickle_rectangular_world(rectangular_world):
    import pickle

    pickle.dumps(rectangular_world)
