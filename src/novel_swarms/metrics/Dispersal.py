import numpy as np
import math
from .Circliness import RadialVarianceHelper


class Dispersal(RadialVarianceHelper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = self.__class__.__name__

    def _calculate(self):
        positions = [agent.getPosition() for agent in self.population]

        for agent in self.population:
            distances = []

            for position in positions:
                distance = self.distance(agent.getPosition(), position)

                if distance != 0:
                    distances.append(distance)

            distances.sort()

            return distances[0]

    @staticmethod
    def distance(a, b):
        return np.linalg.norm(a - b)
