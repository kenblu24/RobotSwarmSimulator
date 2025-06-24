import numpy as np
import random

from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner

class DonutAgentSpawner(PointAgentSpawner):
    def __init__(
            self,
            world,
            n=1,
            agent=None,
            facing=None,
            avoid_overlap=False,
            seed='unspecified',
            oneshot=False,
            circle_centre=[5.0, 5.0],
            inner_radius=4.0,
            outer_radius=6.0,
            **kwargs
        ):
            super().__init__(world, n, agent, facing, avoid_overlap, seed, oneshot, **kwargs)


    def generate_positions(self, n, circle_centre, inner_radius, outer_radius):

        theta = random.uniform(0, 2*np.pi)
        radius = random.uniform(inner_radius, outer_radius)

        positions = np.array([])
        for i in range(n):
            x = circle_centre[0] + (radius * np.cos(theta))
            y = circle_centre[1] + (radius * np.sin(theta))
            positions = np.append(positions, np.array([x, y]))
            theta = random.uniform(0, 2*np.pi)

        return positions

    def generate_config(self, name=None):
        config = super().generate_config(name)
        config.position = self.generate_positions(1).flatten()
        return config




