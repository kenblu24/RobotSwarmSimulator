import numpy as np
from .Metric import Metric


class Boids(Metric):
    def __init__(
        self,
        name="Boids",
        separation_
        history=None,
        default_aggregation='average',
    ):
        super().__init__(name=name, history_size=history, default_aggregation=default_aggregation)
        self.name = name
        self.history = history
        self.default = default
        self.default_aggregation = default_aggregation
        self.value_history = []

    def calculate(self):
        self.set_value(self.default)

    def set_value(self, value):
        self.value_history.append(value)
        if len(self.value_history) > self.history:
            self.value_history.pop(0)
        self.set_value(np.mean(self.value_history))
