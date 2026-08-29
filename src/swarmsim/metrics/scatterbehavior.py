import numpy as np
from .metric import Metric


class ScatterBehavior(Metric):
    def __init__(self, regularize=True, **kwargs):
        super().__init__(**kwargs)
        self.world_radius = 0
        self.regularize = regularize

    @property
    def population(self):
        return self.parent.population

    def calculate(self):
        n = len(self.population)
        r = self.world.config.radius

        distance_list = []
        mew = self.center_of_mass()

        for agent in self.population:
            x_i = agent.getPosition()

            if self.regularize:
                distance = np.linalg.norm(x_i - mew) ** 2
            else:
                distance = np.linalg.norm(x_i - mew)
            distance_list.append(distance)

        if self.regularize:
            scatter = sum(distance_list) / (r * r * n)
        else:
            scatter = sum(distance_list) / n

        self.set_value(scatter)

    def center_of_mass(self):
        positions = [
            [
                agent.getPosition()[i] for agent in self.population
            ] for i in range(len(self.population[0].getPosition()))
        ]
        center = np.array([np.average(pos) for pos in positions])
        return center
