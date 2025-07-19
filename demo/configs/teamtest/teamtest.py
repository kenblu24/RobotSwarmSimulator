from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
from swarmsim.world.simulate import main as sim
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.agent.control.BinaryController import BinaryController


controller = BinaryController((0.4, 0), (0.4, 0.2))
world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
world = RectangularWorld(world_config)

agent1_config = MazeAgentConfig(angle=.0, position=(5, 4.2), agent_radius=0.1,team="Green",body_color=(0,255,0))
agent1 = MazeAgent(agent1_config, world)

world.population.append(agent1)


agent2_config = MazeAgentConfig(angle=3.14, position=(10,5), agent_radius=0.1, team="Orange", body_color=(255,165,0))
agent2 = MazeAgent(agent2_config,world)
agent2.controller = controller
sensor2 = BinaryFOVSensor(agent2,target_team="Blue", theta=0.45,distance=2,)
agent2.sensors.append(sensor2)

world.population.append(agent2)


agent3_config = MazeAgentConfig(angle=3.14, position=(5,5), agent_radius=0.1, team="Blue", body_color=(0,0,255))
agent3 = MazeAgent(agent3_config,world)

world.population.append(agent3)
sim(world)