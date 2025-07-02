import pytest
from pytest_mock import MockerFixture

import numpy as np
from src.swarmsim.agent.MazeAgent import MazeAgent
from src.swarmsim.world.RectangularWorld import RectangularWorld
from src.swarmsim.world.spawners.AgentSpawner import PointAgentSpawner


class TestPointSpawners:
    @pytest.fixture(autouse=True)
    def spawner_setup(self, mocker: MockerFixture):
        self.world = mocker.Mock(spec=RectangularWorld)
        self.world.population = []
        self.world.rng = np.random.default_rng() # np.random.Generator
        self.world.spawners = [] # list[Spawner]

        agent = mocker.MagicMock(spec=MazeAgent)
        # agent = mocker.Mock(spec=MazeAgent)
        # mocker.patch("agent.pos", np.array((5, 5)))
        agent.pos = np.array((5, 5))

        self.n_objects: int = 6
        spawner = PointAgentSpawner(
            self.world, n=self.n_objects, facing="away", avoid_overlap=True,
            agent=agent,
        )
        spawner.agent_config.position = np.array((5, 5))
        self.world.spawners.append(spawner)

    def test_do_spawn(self):
        self.world.spawners[0].do_spawn()
        assert len(self.world.population) == 1

    # def test_step(self):
    #     self.spawner.step()
    #     assert len(self.world.population), self.n_objects
