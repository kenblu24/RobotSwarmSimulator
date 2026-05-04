import numpy as np
from .AbstractMetric import AbstractMetric


class AndMetrics(AbstractMetric):

    def __init__(
        self,
        name="AndMetrics",
        history=100,
        metrics=None,
        and_fn='python'
    ):
        super().__init__(name=name, history_size=history)
        self.metrics = metrics
        self.and_fn = and_fn
        try:
            self.setup_submetrics()
        except ValueError:
            pass

    def attach_world(self, world):
        res = super().attach_world(world)
        self.setup_submetrics()
        return res

    def setup_submetrics(self):
        if self.metrics is None:
            raise ValueError("AndMetrics requires a list of metrics.")
        if self.world is None:
            raise ValueError("World must be set before metrics can be added.")
        self.metrics = [self.world.add_metric(metric) for metric in self.metrics]

    def calculate(self):
        if self.metrics is None:
            raise ValueError("AndMetrics requires a list of metrics.")
        for metric in self.metrics:
            metric.calculate()
        if self.and_fn == 'python':
            self.set_value(all(metric.value for metric in self.metrics))
        elif self.and_fn == 'numpy':
            self.set_value(np.logical_and.reduce([metric.value for metric in self.metrics]))
