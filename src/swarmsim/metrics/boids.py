import numpy as np
from .metric import Metric
from ..util.collections import RefProp


def ensure_type_tag(metric, tag):
    if isinstance(metric, dict):
        if 'type' not in metric:
            metric['type'] = tag

    return metric


class Boids(Metric):
    def __init__(
        self,
        separation_metric=None,
        cohesion_metric=None,
        alignment_metric=None,
        linear=(1, 1, 1,),
        **kwargs
    ):
        super().__init__(**kwargs)
        self.separation = ensure_type_tag(separation_metric, 'Separation')
        self.cohesion = ensure_type_tag(cohesion_metric, 'Separation')
        self.alignment = ensure_type_tag(alignment_metric, 'Alignment')
        self.linear = linear
        self.min_travel = 0.1

    @Metric.world.setter
    def world(self, value):
        Metric.world.fset(self, value)
        # if isinstance(self.separation, dict):
        # if isinstance(self.cohesion, dict):
        # if isinstance(self.alignment, dict):
        self._separation = self.setup_submetric(self.separation)
        self._separation.world = value
        self._cohesion = self.setup_submetric(self.cohesion)
        self._cohesion.world = value
        self._alignment = self.setup_submetric(self.alignment)
        self._alignment.world = value

    def setup_submetric(self, metric):
        if self.world and metric is not None:
            metric = self.world.add_metric(metric, add_to_world=False)
        return metric

    separation = RefProp('parent', set_callbacks=[setup_submetric], extra_names={'world': 'world'})
    cohesion = RefProp('parent', set_callbacks=[setup_submetric], extra_names={'world': 'world'})
    alignment = RefProp('parent', set_callbacks=[setup_submetric], extra_names={'world': 'world'})

    def calculate(self):
        a, b, c = self.linear
        self.separation.calculate()
        self.cohesion.calculate()
        self.alignment.calculate()

        score = a * self.separation.average + b * self.cohesion.average + c * self.alignment.average
        dist = self.distance_reward()
        self.set_value(score + score*dist)

    # TODO: Give this method a better name
    def distance_reward(self) -> float:
        curr_centroid = self.center_of_mass()

        # Update centroids list
        self.centroids.append(curr_centroid)

        T = self.history_size
        prev_centroid = self.centroids[-T if len(self.centroids) >= T else 0]
        dist = np.linalg.norm(curr_centroid - prev_centroid)

        return 0. if dist < self.min_travel else dist

    def center_of_mass(self):
        # NOTE(mabay): Copied this over from 'RadialVarianceMetric'
        return self.positions.mean(axis=0)