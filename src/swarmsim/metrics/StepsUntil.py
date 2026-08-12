import numpy as np
from .Metric import Metric
from ..util.collections import RefProp


class StepsUntil(Metric):
    # TODO: metric = RefProp('parent')  # metric may be a config dict, need to allow refprop to handle that

    def __init__(
        self,
        name="StepsUntil",
        history=None,
        metric=None,
        until_expression=None,
        default='__unset__',
        sentinel='__none__',
    ):
        self._default = default
        self._expression = until_expression
        super().__init__(name=name, history_size=history)
        self.metric = metric
        self.sentinel = sentinel
        try:
            self.setup_submetric()
        except ValueError:
            pass

    def reset(self):
        super().reset()
        self.t = 0
        if self._default != '__unset__':
            self.current_value = self._default
        self.time_activated = None
        self.expression = self.expression

    @property
    def expression(self):
        return self._expression

    @expression.setter
    def expression(self, value):
        self._expression = value
        if value and self.world:
            self.compiled_expr = self.world.jenv.compile_expression(self._expression)
        else:
            self.compiled_expr = None

    @Metric.world.setter
    def world(self, value):
        Metric.world.fset(self, value)
        self.expression = self.expression
        self.setup_submetric()

    def setup_submetric(self):
        if self.metric is None:
            raise ValueError("StepsUntil requires a metric.")
        if self.world is None:
            raise ValueError("World must be set before metrics can be added.")
        self.metric = self.world.add_metric(self.metric, add_to_world=False)
        self.metric.world = self.world

    def calculate(self):
        if self.time_activated is None:
            self.metric.calculate()

            if self.sentinel != '__none__':
                activated = self.metric.value == self.sentinel
            elif self.compiled_expr:
                activated = self.compiled_expr(world=self.world, metric=self.metric)
            else:
                activated = self.metric.value

            if activated:
                self.time_activated = self.t
                self.set_value(self.time_activated)
        self.t += 1
