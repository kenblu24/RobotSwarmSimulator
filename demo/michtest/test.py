from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.agent.control.StaticController import StaticController
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor

from swarmsim.world.simulate import main as simulate_world

rw_cfg = RectangularWorldConfig(size=(8, 8))
world = RectangularWorld(rw_cfg)

# spawned binary controller agent
controller = StaticController(output=[0.01, 0])
agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1, controller=controller), world)
sensor = BinaryFOVSensor(agent, theta=0.45, distance=2, bias=0)
agent.sensors.append(sensor)

world.population.append(agent)

simulate_world(world, start_paused=True)