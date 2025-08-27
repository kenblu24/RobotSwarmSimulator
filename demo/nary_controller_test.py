import numpy as np

from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.sensors.ObjectFOVSensor import ObjectFOVSensor
from swarmsim.world.objects.StaticObject import StaticObject, StaticObjectConfig
from swarmsim.agent.control.NaryController import NaryController
from swarmsim.world.simulate import main as sim

world_config = RectangularWorldConfig(size=(10, 10), time_step=1/40)
world = RectangularWorld(world_config)

sensors = [
    ObjectFOVSensor(theta=np.pi/8, distance=1.5, bias=np.pi/-8),
    ObjectFOVSensor(theta=np.pi/8, distance=1.5, bias=np.pi/8)
    # BinaryFOVSensor(theta=0.45, distance=2, bias=0),
    # BinaryFOVSensor(theta=0.45, distance=2, bias=np.pi / -2)
]
controller = NaryController(on_nothing=(0.1, 0.0), on_detect=[
    (0.5, 0.2), (0.5, -0.2)
    # (0.0, 0.2), (0.0, -0.2)
], mode="first")
pos = (5, 8)
agent_config = MazeAgentConfig(
    position=pos, agent_radius=0.175, controller=controller, sensors=sensors, angle=-np.pi/2
)
agent = MazeAgent(agent_config, world)
world.population.append(agent)

dx = 1
static_objs = [
    StaticObject(StaticObjectConfig(points=[(pos[0]-dx, 10), (pos[0]-dx, 3), (pos[0]-dx-1, 3), (pos[0]-dx-1, 10)]), world=world),
    StaticObject(StaticObjectConfig(points=[(pos[0]+dx, 10), (pos[0]+dx, 3), (pos[0]+dx+1, 3), (pos[0]+dx+1, 10)]), world=world)
]
for obj in static_objs:
    world.objects.append(obj)

sim(world, start_paused=True)
