import pymunk
from pymunk import Vec2d
import numpy as np
# from ..world.RectangularWorld import RectangularWorld
# from ..agent.Agent import Agent

def copyCoords(dest, src):
    coords = [0, 0]
    if isinstance(src, Vec2d):
        coords[0] = src.x
        coords[1] = src.y
    elif isinstance(src, np.ndarray):
        coords[0] = float(src[0])
        coords[1] = float(src[1])
    
    if isinstance(dest, Vec2d):
        return Vec2d(coords[0], coords[1])
    elif isinstance(dest, np.ndarray):
        return np.array([np.float64(coords[0]), np.float64(coords[1])])

class Physics:
    # world: RectangularWorld
    def __init__(self, world):
        self.world = world
        self.space = pymunk.Space()

    def step(self):
        self.space.step(self.world.dt)
        for agent in self.world.population:
            agent.pos = copyCoords(agent.pos, agent.physobj.position)
            agent.angle = np.float64(agent.physobj.angle)

    def createAgentBody(self, agent):
        mass = 1
        radius = 0.1
        body = pymunk.Body(mass=mass, moment=pymunk.moment_for_circle(mass, 0, radius))
        shape = pymunk.shapes.Circle(body=body, radius=radius)
        shape.friction = 0.5
        body.position = copyCoords(body.position, agent.pos)
        body.angle = float(agent.angle)
        self.space.add(body, shape)
        return body