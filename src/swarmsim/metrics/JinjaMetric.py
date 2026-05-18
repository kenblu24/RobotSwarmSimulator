from functools import partial
import numpy as np
from .AbstractMetric import AbstractMetric
from typing import Sequence


class JinjaMetric(AbstractMetric):

    def __init__(
        self,
        name="JinjaMetric",
        history=None,
        metric=None,
        metrics=None,
        template=None,
        expression=None,
        aux_expressions=(),
        eval_condition=None,
        save_condition=None,
        default='__unset__',
        default_aggregation=None,
    ):
        self._default = default
        self.metric = metric
        self.metrics = metrics
        self.default_aggregation = default_aggregation
        super().__init__(name=name, history_size=history)
        self.template_src = template
        self.template = None
        self._module = None
        self.expression = expression
        self.eval_condition = eval_condition
        self.save_condition = save_condition
        self.exprargs = {'self': self}
        if self.world:
            self.setup_submetrics()

    def reset(self):
        super().reset()
        self.t = 0
        if self._default != '__unset__':
            self.current_value = self._default
        self.time_activated = None
        if isinstance(self.metric, AbstractMetric):
            self.metric.reset()
        if self.metrics is not None:
            for metric in self.metrics:
                if isinstance(metric, AbstractMetric):
                    metric.reset()

    def attach_world(self, world):
        res = super().attach_world(world)
        self.setup_submetrics()
        if self.template_src is not None:
            self.template = self.world.jenv.from_string(self.template_src)
        return res

    def setup_submetrics(self):
        if self.metric is not None:
            self.metric = self.world.add_metric(self.metric, add_to_world=False)
            self.exprargs['metric'] = self.metric
        if self.metrics is not None:
            self.metrics = [self.world.add_metric(metric, add_to_world=False) for metric in self.metrics]
            self.exprargs['metrics'] = self.metrics

    def make_module(self, **kwargs):
        if self.template is None:
            return None
        return self.template.make_module(self.exprargs | kwargs)

    @property
    def module(self):
        if self.template is None:
            return None
        if self._module is not None:
            return self._module
        else:
            return self.make_module()

    def eval_template(self, expression):
        if self.template is None:
            return self.world.jenv.compile_expression(expression)(**self.exprargs)
        return self.template.compile_ctxpr(expression)(**self.exprargs)

    def calculate_submetrics(self):
        if self.metric is not None:
            self.metric.calculate()
        if self.metrics is not None:
            for metric in self.metrics:
                metric.calculate()

    def calculate(self):
        if self.eval_condition is not None:
            if self.template:
                self.template.export_with(**self.exprargs)
            if not self.eval_template(self.eval_condition):
                return
        self.calculate_submetrics()
        if self.template:
            self.template.export_with(**self.exprargs)
            self._module = self.template.module_from_cached_context
        value = self.eval_template(self.expression)
        if self.save_condition is not None and not self.eval_template(self.save_condition):
            return
        self.set_value(value)
