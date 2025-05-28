# from https://kenblu24.github.io/RobotSwarmSimulator/guide/firstrun.html

from novel_swarms.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from novel_swarms.agent.control.StaticController import StaticController
from novel_swarms.world.spawners.AgentSpawner import PointAgentSpawner
from novel_swarms.agent.MazeAgent import MazeAgent, MazeAgentConfig
from novel_swarms.world.simulate import main as sim
from novel_swarms.sensors.BinaryFOVSensor import BinaryFOVSensor
from novel_swarms.agent.control.BinaryController import BinaryController


world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
world = RectangularWorld(world_config)

controller = StaticController(output=[0.01, 0])
agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1, controller=controller), world)
sensor = BinaryFOVSensor(agent, theta=0.45, distance=2,)
agent.sensors.append(sensor)
controller = BinaryController((0.02, -0.5), (0.02, 0.5))
agent.controller = controller

spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True, agent=agent, oneshot=True)
world.spawners.append(spawner)

sim(world)