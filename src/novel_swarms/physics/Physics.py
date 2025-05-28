import pymunk
from ..world.World import World


class Physics:
    world: World
    def __init__(self, world):
        self.world = world
        self.space = pymunk.Space()

    def step(self):
        self.space.step(self.world.dt)
        