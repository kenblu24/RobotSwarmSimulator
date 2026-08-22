import numpy as np
from .Metric import Metric
from ..util.collections import RefProp


def ensure_type_tag(metric, tag):
    if isinstance(metric, dict):
        if 'type' not in metric:
            metric['type'] = tag
        return metric


class Boids(Metric):
    def __init__(
        self,
        name="Boids",
        separation_metric=None,
        cohesion_metric=None,
        alignment_metric=None,
        history=None,
        default_aggregation='average',
    ):
        super().__init__(name=name, history_size=history, default_aggregation=default_aggregation)
        self.separation = ensure_type_tag(separation_metric, 'Cohesion')
        self.cohesion = ensure_type_tag(cohesion_metric, 'Cohesion')
        self.alignment = ensure_type_tag(alignment_metric, 'Alignment')

    @Metric.world.setter
    def world(self, value):
        Metric.world.fset(self, value)
        if isinstance(self.separation, dict):
            self._separation = self.setup_submetric(self.separation)
        if isinstance(self.cohesion, dict):
            self._cohesion = self.setup_submetric(self.cohesion)
        if isinstance(self.alignment, dict):
            self._alignment = self.setup_submetric(self.alignment)

    def setup_submetric(self, metric):
        if self.world and metric is not None:
            metric = self.world.add_metric(metric, add_to_world=False)
        return metric

    separation = RefProp('parent', set_callbacks=[setup_submetric], extra_names={'world': 'world'})
    cohesion = RefProp('parent', set_callbacks=[setup_submetric], extra_names={'world': 'world'})
    alignment = RefProp('parent', set_callbacks=[setup_submetric], extra_names={'world': 'world'})

    def calculate(self):
        separation = self.separation.calculate()
        cohesion = self.cohesion.calculate()
        alignment = self.alignment.calculate()
        self.set_value(separation + cohesion + alignment)

    def set_value(self, value):
        self.value_history.append(value)
        if len(self.value_history) > self.history:
            self.value_history.pop(0)
        self.set_value(np.mean(self.value_history))
