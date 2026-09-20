import numpy as np

from .aggregation import Aggregation


class MovingMass(Aggregation):
    def __init__(self, weights: tuple[float, float] = (1, 1), *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.centroids = []
        self.weights = weights

    def center_of_mass(self):
        # NOTE(mabay): Copied this over from 'RadialVarianceMetric'
        positions = np.asarray([agent.getPosition() for agent in self.parent.population])
        return positions.mean(axis=0)

    def _calculate(self):
        agg_wt, disp_wt = self.weights
        centroid = self.center_of_mass()
        self.centroids.append(centroid)
        disp_value = np.linalg.norm(centroid - self.centroids[0])
        return agg_wt * super()._calculate() + disp_wt * np.clip(disp_value, 0, 1)