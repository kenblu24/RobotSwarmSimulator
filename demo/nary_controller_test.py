import numpy as np

from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.agent.control.BinaryController import BinaryController
from swarmsim.agent.control.NaryController import NaryController
from swarmsim.world.simulate import main as sim

world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
world = RectangularWorld(world_config)

agent1_sensors = [
    BinaryFOVSensor(theta=0.45, distance=2, bias=0),
    BinaryFOVSensor(theta=0.45, distance=2, bias=np.pi / -2)
]
agent1_controller = NaryController(on_nothing=(0.05, 0.0), on_detect=[
    (0.0, 0.05), (0.0, 0.5)
])
agent1_config = MazeAgentConfig(
    position=(2, 5), agent_radius=0.175, controller=agent1_controller, sensors=agent1_sensors
)
agent1 = MazeAgent(agent1_config, world)

agent2_config = MazeAgentConfig(position=(5, 5), agent_radius=0.175)
agent2 = MazeAgent(agent2_config, world)


world.population.append(agent1)
world.population.append(agent2)

sim(world)
