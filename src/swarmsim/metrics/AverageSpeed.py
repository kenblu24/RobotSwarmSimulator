import numpy as np
from typing import List
from .AbstractMetric import AbstractMetric


class AverageSpeedBehavior(AbstractMetric):
    def __init__(self, history=100):
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
