"""This file contains sample unit tests for the Point spawner system.
There are two different version of these unit tests. One developed using `unittest` and another
developed using `pytest`.

This file contains the implementation that uses `unittest`, a module from python's
standard library.
"""


import unittest
from unittest.mock import Mock

import numpy as np
from novel_swarms.agent.MazeAgent import MazeAgent
from novel_swarms.world.RectangularWorld import RectangularWorld
from novel_swarms.world.spawners.AgentSpawner import PointAgentSpawner


class TestPointSpawners(unittest.TestCase):
    def setUp(self):
        self.world = Mock(spec=RectangularWorld)
        self.world.population = []
        self.world.rng = np.random.default_rng() # np.random.Generator
        self.world.spawners = [] # list[Spawner]

        agent = Mock(spec=MazeAgent)
        agent.pos = np.array((5, 5))

        self.n: int = 6
        self.spawner = PointAgentSpawner(
            self.world, n=self.n, facing="away", avoid_overlap=True,
            agent=agent,
        )
        self.spawner.agent_config.position = np.array((5, 5))

        self.world.spawners.append(self.spawner)
        return super().setUp()

    def test_do_spawn(self):
        self.spawner.do_spawn()
        self.assertEqual(len(self.world.population), 1)

    def test_step(self):
        self.spawner.step()
        self.assertEqual(len(self.world.population), self.n)
