import numpy as np
from .metric import Metric, HasSubMetric
from ..util.collections import RefProp


class StepsUntil(Metric, HasSubMetric):
    # TODO: metric = RefProp('parent')  # metric may be a config dict, need to allow refprop to handle that

    def __init__(
        self,
        name='__class__',
        history=None,
        metric=None,
        until_expression=None,
        default='__unset__',
        sentinel='__none__',
    ):
        self._default = default
        self._expression = until_expression
        HasSubMetric.__init__(self, metric=metric)
        super().__init__(name=name, history_size=history)
        self.sentinel = sentinel

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
        HasSubMetric.world.fset(self, value)
        self.expression = self.expression

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
