import numpy as np

from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.sensors.ObjectFOVSensor import ObjectFOVSensor
from swarmsim.sensors.OccludedAgentFOVSensor import OccludedAgentFOVSensor
from swarmsim.agent.control.NaryController import NaryController
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.world.World import AbstractWorldConfig
from swarmsim.world.simulate import main as sim

rwc = RectangularWorldConfig.from_yaml("demo/test.yaml")
world = RectangularWorld(rwc)

sens_dist: float = 3
th: float = np.pi/8
sensors = [
    ObjectFOVSensor(theta=th, distance=sens_dist),
    OccludedAgentFOVSensor(theta=th, distance=sens_dist)
    # ObjectFOVSensor(theta=np.pi/8, distance=obj_sens_dist, bias=np.pi/-8), # left
    # ObjectFOVSensor(theta=np.pi/8, distance=obj_sens_dist, bias=np.pi/8)   # right
    # BinaryFOVSensor(theta=0.45, distance=2, bias=0),
    # BinaryFOVSensor(theta=0.45, distance=2, bias=np.pi / -2)
]
# controller = NaryController(on_nothing=(0.5, 0), on_detect=[
#     (0.5, 0.05), # left
#     (0.5, 0.0) # right
# ], mode="first")
controller = NaryController(on_nothing=(0.5, 0.2), on_detect=[
    (0.5, -0.2),
], mode="first")

pos = (27.4, -3)
agent_config = MazeAgentConfig(
    position=pos, agent_radius=0.6, controller=controller, sensors=sensors,
    angle=np.pi/2
)
agent = MazeAgent(agent_config, world)
world.population.append(agent)

sim(world, start_paused=True)


# Successful settings
# - nothing: (0.5, 0.1), detect = [(0.5, -0.1)]
