from RobotSwarmSimulator.src.novel_swarms.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from RobotSwarmSimulator.src.novel_swarms.agent.MazeAgent import MazeAgent, MazeAgentConfig
from RobotSwarmSimulator.src.novel_swarms.world.spawners.AgentSpawner import PointAgentSpawner
from RobotSwarmSimulator.src.novel_swarms.world.simulate import main as sim
from RobotSwarmSimulator.src.novel_swarms.sensors.BinaryFOVSensor import BinaryFOVSensor
from RobotSwarmSimulator.src.novel_swarms.agent.control.BinaryController import BinaryController


controller = BinaryController((0.4, 0), (0.4, 0.2))
world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
world = RectangularWorld(world_config)

agent1_config = MazeAgentConfig(angle=.0, position=(0, 5), agent_radius=0.1,team="Orange",body_color=(255,165,0))
agent1 = MazeAgent(agent1_config, world)
agent1.controller = controller
sensor = BinaryFOVSensor(agent1,target_team="Blue", theta=0.45, distance=2,)
agent1.sensors.append(sensor)

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