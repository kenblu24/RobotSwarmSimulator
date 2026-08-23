from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.world.simulate import main as sim
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.agent.control.BinaryController import BinaryController


world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
world = RectangularWorld(world_config)

agent1_config = MazeAgentConfig(angle=.0, position=(5, 4.2), agent_radius=0.1, team="Green", body_color=(0, 255, 0))
agent1 = MazeAgent(agent1_config, world)
world.population.append(agent1)

agent2_config = MazeAgentConfig(angle=3.14, position=(5, 5), agent_radius=0.1, team="Blue", body_color=(0, 0, 255))
agent2 = MazeAgent(agent2_config, world)
world.population.append(agent2)


sensor = BinaryFOVSensor(theta=0.45, distance=2, target_team="Blue")
agentx_config = MazeAgentConfig(angle=3.14, position=(8, 5), agent_radius=0.1,
                                controller=BinaryController((0.4, 0), (0.4, 0.2)),
                                sensors=[sensor],
                                team="Orange", body_color=(255, 165, 0))
agentx = MazeAgent(agentx_config, world)
world.population.append(agentx)


sim(world, stop_detection=lambda world: world.total_steps == 300)

# world.population.remove(agentx)
sensor.target_team = None  # detect all agents regardless of team
world.population.append(MazeAgent(agentx_config, world))

sim(world, stop_detection=lambda world: world.total_steps == 800)
