import numpy as np

from .aggregation import Aggregation


class MovingMass(Aggregation):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.centroids = []

    def center_of_mass(self):
        # NOTE(mabay): Copied this over from 'RadialVarianceMetric'
        positions = np.asarray([agent.getPosition() for agent in self.population])
        return positions.mean(axis=0)

    def _calculate(self):
        centroid = self.center_of_mass()
        self.centroids.append(centroid)
        return self.agg._calculate() + np.linalg.norm(centroid - self.centroids[0])