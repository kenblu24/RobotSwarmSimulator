import pytest
from pytest_mock import MockerFixture

import numpy as np
from swarmsim.agent.Agent import Agent, BaseAgentConfig
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.agent.control.BinaryController import BinaryController
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.world.RectangularWorld import RectangularWorld


class TestAgentConf:
    @pytest.fixture(autouse=True)
    def agent_conf_setup(self, mocker: MockerFixture):
        world = mocker.Mock(spec=RectangularWorld)
        world.rng = np.random.default_rng()
        world.dt = 1 / 40
        controller = mocker.Mock(spec=BinaryController)
        controller.agent = None
        sensor = mocker.Mock(spec=BinaryFOVSensor)
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

    def test_position(self):
        assert len(self.agent.pos) == len(self.bac.position)
        assert (self.agent.pos == self.bac.position).all()

    def test_angle(self):
        assert self.agent.angle == self.bac.angle

    def test_name(self):
        assert self.agent.name == self.bac.name

    def test_grounded(self):
        assert self.agent.grounded == self.bac.grounded

    def test_collided(self):
        assert self.agent.collides == self.bac.collides

    def test_team(self):
        assert self.agent.team == self.bac.team


class TestMazeAgentConf:
    @pytest.fixture(autouse=True)
    def maze_agent_conf_setup(self, mocker: MockerFixture) -> None:
        self.conf = MazeAgentConfig(position=(5, 5), agent_radius=0.1)
        world = mocker.Mock(spec=RectangularWorld)
        world.rng = np.random.default_rng()
        world.dt = 1 / 40
        self.agent = MazeAgent(self.conf, world)

    def test_radius(self):
        assert self.agent.radius == self.conf.agent_radius
