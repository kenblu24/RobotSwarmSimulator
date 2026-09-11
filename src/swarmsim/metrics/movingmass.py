import numpy as np
from swarmsim.metrics.aggregation import Aggregation


class MovingMass(Aggregation):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.centroids = []

    def _calculate(self):
        centroid = self.center_of_mass()
        self.centroids.append(centroid)
        return super()._calculate() + np.linalg.norm(centroid - self.centroids[0])