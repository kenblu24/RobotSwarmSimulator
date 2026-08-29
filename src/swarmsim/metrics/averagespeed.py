import numpy as np
from .metric import Metric


class AverageSpeedBehavior(Metric):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @property
    def population(self):
        return self.parent.population

    def calculate(self):
        n = len(self.population)
        velocities = [np.linalg.norm(agent.getVelocity()) for agent in self.population]
        average_speed = sum(velocities) / n
        self.set_value(average_speed)
