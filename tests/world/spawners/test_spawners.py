import pytest
from pytest_mock import MockerFixture

import numpy as np
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.agent.StaticAgent import RectangularWorldConfig
from swarmsim.world.RectangularWorld import RectangularWorld
from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner


class TestPointSpawners:
    @pytest.fixture(autouse=True)
    def spawner_setup(self, mocker: MockerFixture):
        rwc = RectangularWorldConfig(size=(5, 5))
        self.world: RectangularWorld = RectangularWorld(rwc)
        self.world.setup()

        agentconf = MazeAgentConfig(position=(2, 2))
        agent = MazeAgent(agentconf, self.world)
        agent.pos = np.array((5, 5))

        spawner = PointAgentSpawner(
            self.world, n=6, facing="away", avoid_overlap=True,
            agent=agent,
        )
        self.world.spawners.append(spawner)

    def test_do_spawn(self):
        assert len(self.world.population) == 0
        self.world.spawners[0].do_spawn() # type: ignore
        assert len(self.world.population) == 1

        agent = self.world.population[0] # type: ignore
        assert type(agent).__name__ == "MazeAgent"

