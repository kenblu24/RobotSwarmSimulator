# from https://kenblu24.github.io/RobotSwarmSimulator/guide/firstrun.html

from novel_swarms.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from novel_swarms.agent.control.StaticController import StaticController
from novel_swarms.world.spawners.DonutSpawner import DonutAgentSpawner
from novel_swarms.world.spawners.AgentSpawner import PointAgentSpawner
from novel_swarms.agent.MazeAgent import MazeAgent, MazeAgentConfig
from novel_swarms.world.simulate import main as sim
from novel_swarms.sensors.BinaryFOVSensor import BinaryFOVSensor
from novel_swarms.agent.control.BinaryController import BinaryController
from novel_swarms.agent.control.HumanController import HumanController
from novel_swarms.agent.control.StaticController import StaticController
from novel_swarms.world.objects.StaticObject import StaticObject, StaticObjectConfig
from novel_swarms.world.objects.Wall import Wall

# world
world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
world = RectangularWorld(world_config)

# spawned binary controller agent
controller = StaticController(output=[0.01, 0])
agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1, controller=controller), world)
sensor = BinaryFOVSensor(agent, theta=0.45, distance=2,)
agent.sensors.append(sensor)
controller = BinaryController((0.27, -0.6), (0.27, 0.6))
agent.controller = controller

# spawner
spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True, agent=agent, mode="oneshot")
world.spawners.append(spawner)

# human controller agent
humanAgent = MazeAgent(MazeAgentConfig(position=(5, 1), agent_radius = 0.08, controller=HumanController()), world)
world.addAgent(humanAgent)

# static controller agent
staticAgent = MazeAgent(MazeAgentConfig(position=(6, 6), agent_radius = 0.12, controller=StaticController()), world)
world.addAgent(staticAgent)

# static object
staticObject = StaticObject(StaticObjectConfig(points=[(0.5, 3.5), (4.5, 3.5), (4.5, 5.5), (2.5, 6.5), (0.5, 5.5)]), world=world)
world.addAgent(staticObject)

sim(world, start_paused=True)