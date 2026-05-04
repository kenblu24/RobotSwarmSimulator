import numpy as np
from .AbstractMetric import AbstractMetric


class StepsUntil(AbstractMetric):

    def __init__(
        self,
        name="StepsUntil",
        history=100,
        metric=None,
        # default='__passthrough__',
        sentinel='__none__',
    ):
        super().__init__(name=name, history_size=history)
        self.metric = metric
        try:
            self.setup_submetric()
        except ValueError:
            pass
        # self.default = default
        self.sentinel = sentinel
        self.t = 0
        self.time_activated = None

    def attach_world(self, world):
        res = super().attach_world(world)
        self.setup_submetric()
        return res

    def setup_submetric(self):
        if self.metric is None:
            raise ValueError("StepsUntil requires a metric.")
        if self.world is None:
            raise ValueError("World must be set before metrics can be added.")
        self.metric = self.world.add_metric(self.metric)

    def calculate(self):
        if self.time_activated is None:
            if self.sentinel != '__none__':
                if self.metric.value == self.sentinel:
                    self.time_activated = self.t
            elif self.metric.value:
                self.time_activated = self.t
            self.set_value(self.time_activated)
        self.t += 1
