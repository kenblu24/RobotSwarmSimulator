import pytest
from pytest_mock import MockerFixture

import numpy as np
from novel_swarms.agent.MazeAgent import MazeAgent
from novel_swarms.world.RectangularWorld import RectangularWorld
from novel_swarms.world.spawners.AgentSpawner import PointAgentSpawner


class TestPointSpawners:
    @pytest.fixture(autouse=True)
    def spawner_setup(self, mocker: MockerFixture):
        self.world = mocker.Mock(spec=RectangularWorld)
        self.world.population = []
        self.world.rng = np.random.default_rng() # np.random.Generator
        self.world.spawners = [] # list[Spawner]

        agent = mocker.Mock(spec=MazeAgent)
        agent.pos = np.array((5, 5))

        self.n_objects: int = 6
        self.spawner = PointAgentSpawner(
            self.world, n=self.n_objects, facing="away", avoid_overlap=True,
            agent=agent,
        )
        self.spawner.agent_config.position = np.array((5, 5))

        self.world.spawners.append(self.spawner)

    def test_do_spawn(self):
        self.spawner.do_spawn()
        assert len(self.world.population) == 1

    def test_step(self):
        self.spawner.step()
        assert len(self.world.population), self.n_objects
