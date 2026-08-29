import numpy as np
from itertools import combinations
from .metric import Metric
from .aggregation import Aggregation, aggregation_functions

# typing
from typing import Any, override


class InteragentDispersion(Aggregation):
    _invert_multiplier = 1
    default_aggregation = 'average'

    def __init__(
        self,
        *args,
        reduce_agent_distances='nearest',
        reduce_distances='mean',
        **kwargs
    ):
        super().__init__(*args, reduce_distances=reduce_distances, **kwargs)


class ExplodingDispersion(Metric):
    default_aggregation = 'average'

    def __init__(self, *args, scale=1.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.scale = scale

    def _calculate(self):
        positions = np.array([agent.getPosition() for agent in self.parent.population])
        average_position = np.average(positions, axis=0)
        distances = np.linalg.norm(positions - average_position, axis=1)
        return np.average(distances) * self.scale
