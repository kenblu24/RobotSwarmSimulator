import numpy as np
from .metric import Metric


class AngularMomentumBehavior(Metric):
    def __init__(self, name='__class__', history=None):
        super().__init__(name=name, history_size=history)

    @property
    def population(self):
        return self.parent.population

    def calculate(self):
        n = len(self.population)
        r = self.world.config.radius

        momentum_list = []
        mew = self.center_of_mass()

        for agent in self.population:
            x_i = agent.getPosition()
            v_i = agent.getVelocity()

            momentum = np.cross(v_i, x_i - mew)
            momentum_list.append(momentum)

        average_momentum = sum(momentum_list) / (r * n)
        self.set_value(average_momentum)

    def center_of_mass(self):
        positions = [
            [
                agent.getPosition()[i] for agent in self.population
            ] for i in range(len(self.population[0].getPosition()))
        ]
        center = np.array([np.average(pos) for pos in positions])
        return center
