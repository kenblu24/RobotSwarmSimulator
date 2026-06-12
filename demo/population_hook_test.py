# from https://kenblu24.github.io/RobotSwarmSimulator/guide/firstrun.html

from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.control.StaticController import StaticController
from swarmsim.world.spawners.DonutSpawner import DonutAgentSpawner
from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.world.simulate import main as sim
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.agent.control.BinaryController import BinaryController
from swarmsim.agent.control.HumanController import HumanController
from swarmsim.agent.control.StaticController import StaticController
from swarmsim.world.objects.StaticObject import StaticObject, StaticObjectConfig
from swarmsim.world.objects.Wall import Wall

# world
world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
world = RectangularWorld(world_config)

# spawned binary controller agent
controller = StaticController(output=[0.01, 0])
agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1, controller=controller), world)
sensor = BinaryFOVSensor(agent, theta=0.45, distance=2, bias=0)
agent.sensors.append(sensor)
controller = BinaryController((0.27, -0.6), (0.27, 0.6))
agent.controller = controller

# spawner
spawner = PointAgentSpawner(world, n=10, facing="away", avoid_overlap=True, agent=agent, mode="oneshot")
world.spawners.append(spawner)

# this is the part we are testing
world.population.register_add_callback(lambda x: print(f"added {x}"))
world.population.register_del_callback(lambda x: print(f"deleted {x}"))

sim(world, start_paused=True)