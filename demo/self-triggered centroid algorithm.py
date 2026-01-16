# from https://kenblu24.github.io/RobotSwarmSimulator/guide/firstrun.html

from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.control.StaticController import StaticController
from swarmsim.world.spawners.DonutSpawner import DonutAgentSpawner
from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner, UniformAgentSpawner
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.world.simulate import main as sim
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.agent.control.VoronoiController import VoronoiController
from swarmsim.world.objects.StaticObject import StaticObject, StaticObjectConfig
from swarmsim.world.objects.Wall import Wall

import numpy as np

def paper_density(x, y):
    return np.exp(-((x-2)**2 + (y-3)**2)) + np.exp(-((x-3)**2 + (y-1)**2))

# world
world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
world = RectangularWorld(world_config)

# spawned binary controller agent
controller = VoronoiController(agent=None, parent=None)
agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1, controller=controller), world)

agent.controller.set_density_function(paper_density)
sensor = BinaryFOVSensor(agent, theta=0.45, distance=2, bias=0)
# agent.sensors.append(sensor)
# controller = BinaryController((0.27, -0.6), (0.27, 0.6))
# agent.controller = controller



# spawner
spawner = PointAgentSpawner(world, n=3, facing="away", avoid_overlap=True, agent=agent, mode="oneshot")
# spawner = UniformAgentSpawner(world, n=6, facing="away", avoid_overlap=True, agent=agent, mode="oneshot")
world.spawners.append(spawner)

# print("Population before spawner:", len(world.population))

sim(world, start_paused=True)

