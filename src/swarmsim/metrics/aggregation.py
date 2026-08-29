import numpy as np
from .metric import Metric
from ..util.geometry.euclidean import fast_pairwise_distances


aggregation_functions = {
    'mean': np.mean,
    'avg': np.mean,
    'average': np.mean,
    'sum': np.sum,
    'total': np.sum,
    'max': np.max,
    'maximum': np.max,
    'farthest': np.max,
    'min': np.min,
    'minimum': np.min,
    'closest': np.min,
    'nearest': np.min,
    'None': None,
    None: None,
}


class Aggregation(Metric):
    _invert_multiplier = -1
    default_aggregation = 'average'

    def __init__(
        self,
        *args,
        name='__class__',
        scale=1.0,
        reduce_agent_distances='farthest',
        reduce_distances='mean',
        **kwargs
    ):
        super().__init__(*args, name=name, **kwargs)
        self.scale = scale
        self.reduce_agent_distances = aggregation_functions.get(reduce_agent_distances, reduce_agent_distances)
        self.reduce_distances = aggregation_functions.get(reduce_distances, reduce_distances)

    def _calculate(self):
        positions = [agent.pos for agent in self.parent.population]
        distances = fast_pairwise_distances(positions, collapse_diagonal_along=0)

        if self.reduce_agent_distances:
            distances = self.reduce_agent_distances(distances, axis=1)
        aggregation = self.reduce_distances(distances)

        return self.scale * aggregation * self._invert_multiplier
