import numpy as np
from itertools import combinations
from .Circliness import RadialVarianceHelper

# typing
from typing import Any, override


class InteragentDispersion(RadialVarianceHelper):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = self.__class__.__name__

    @override
    def _calculate(self):
        # This function creates a metric for the agents to follow

        positions = np.array([agent.getPosition() for agent in self.population])
        dim = positions.shape[1]

        pairs = np.asarray(list(combinations(positions, 2)))
        distances = np.linalg.norm(np.diff(pairs, axis=1).reshape(-1, dim), axis=1)

        return np.average(distances) * self.scale


class ExplodingDispersion(RadialVarianceHelper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = self.__class__.__name__

    @override
    def _calculate(self):
        positions = np.array([agent.getPosition() for agent in self.population])
        average_position = np.average(positions, axis=0)
        distances = np.linalg.norm(positions - average_position, axis=1)
        return np.average(distances) * self.scale
