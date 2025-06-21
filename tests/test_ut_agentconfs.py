"""This file contains sample unit tests for the Agent Config system.
There are two different version of these unit tests. One developed using `unittest` and another
developed using `pytest`.

This file contains the implementation that uses `unittest`, a module from python's
standard library.
"""

import unittest
from unittest.mock import Mock

import numpy as np
from novel_swarms.agent.Agent import Agent, BaseAgentConfig
from novel_swarms.agent.MazeAgent import MazeAgent, MazeAgentConfig
from novel_swarms.agent.control.BinaryController import BinaryController
from novel_swarms.sensors.BinaryFOVSensor import BinaryFOVSensor
from novel_swarms.world.RectangularWorld import RectangularWorld


class TestAgentConf(unittest.TestCase):
    def setUp(self) -> None:
        world = Mock(spec=RectangularWorld)
        world.rng = np.random.default_rng()
        world.dt = 1/40
        controller = Mock(spec=BinaryController)
        controller.agent = None
        sensor = Mock(spec=BinaryFOVSensor)
        sensor.agent = None

        self.bac = BaseAgentConfig(
            position=(5, 5),
            angle=np.pi * 0.5,
            name="Agent_1",
            controller=controller,
            grounded=True,
            collides=True,
            sensors=[sensor],
            team="hunter"
        )
        self.agent = Agent(self.bac, world)

        return super().setUp()

    def test_position(self):
        self.assertEqual(len(self.agent.pos), len(self.bac.position))
        self.assertEqual(self.agent.pos[0], self.bac.position[0])
        self.assertEqual(self.agent.pos[1], self.bac.position[1])

    def test_angle(self):
        self.assertEqual(self.agent.angle, self.bac.angle)

    def test_name(self):
        self.assertEqual(self.agent.name, self.bac.name)

    def test_grounded(self):
        self.assertEqual(self.agent.grounded, self.bac.grounded)

    def test_collided(self):
        self.assertEqual(self.agent.collides, self.bac.collides)

    def test_team(self):
        self.assertEqual(self.agent.team, self.bac.team)


class TestMazeAgentConf(unittest.TestCase):
    def setUp(self) -> None:
        self.conf = MazeAgentConfig(position=(5, 5), agent_radius=0.1)
        world = Mock(spec=RectangularWorld)
        world.rng = np.random.default_rng()
        world.dt = 1/40
        self.agent = MazeAgent(self.conf, world)
        return super().setUp()

    def test_position(self):
        self.assertEqual(len(self.agent.pos), len(self.conf.position))
        self.assertEqual(self.agent.pos[0], self.conf.position[0])
        self.assertEqual(self.agent.pos[1], self.conf.position[1])

    def test_radius(self):
        self.assertEqual(self.agent.radius, self.conf.agent_radius)
