import numpy as np
from typing import List
from .Metric import Metric


class AverageSpeedBehavior(Metric):
    def __init__(self, history=None):
        super().__init__(name="Average_Speed", history_size=history)
        self.population = None

    def attach_world(self, world):
        super().attach_world(world)
        self.population = world.population

    def calculate(self):
        n = len(self.population)
        velocities = [np.linalg.norm(agent.getVelocity()) for agent in self.population]
        average_speed = sum(velocities) / n
        self.set_value(average_speed)
