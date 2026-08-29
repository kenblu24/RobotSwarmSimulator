"""JinjaMetric Module

.. inheritance-diagram:: JinjaMetric

:meta public:

JinjaMetric
-----------

.. autoclass:: JinjaMetric
    :members:
    :undoc-members:
    :inherited-members:

Examples
--------

.. code-block:: yaml+jinja
   :caption: world.yaml

   type: RectangularWorld
   metrics:
   - type: JinjaMetric
     metrics:
     - type: Circliness
     template: |
       {% set old_aggregation = world.metrics[1].value %}
       {% set multiplier = 2 %}
     expression: old_aggregation * multiplier + metrics[0].value
   - type: Aggregation

"""

from .metric import Metric, HasSubMetrics
from typing import Sequence


class JinjaMetric(Metric, HasSubMetrics):
    """Combine multiple metrics programmatically using a Jinja expression.

    Evaluation Order on ``JinjaMetric.calculate()``:

       1. Evaluate the ``template`` and ``eval_condition`` and return if False.
       2. Call :py:meth:`Metric.calculate` on the submetrics.
       3. Render the ``template`` and save the context.
       4. Evaluate the ``expression``.
       5. Evaluate the ``save_condition`` and return if False.
       6. Call :py:meth:`Metric.set_value`.

    You may reference ``self``, ``world``, ``metric``, and ``metrics`` in the
    ``template``, ``expression``, ``eval_condition``, and ``save_condition``..
    Additionally, any variables set in the ``template`` will be available in the
    expressions.

    Note that metrics in ``world.metrics`` are calculated in order (i.e. the order specified/added).
    Thus, any metrics before this ``JinjaMetric`` will be up-to-date when this metric is calculated,
    but any metrics ordered afterwards may be at least one time step behind.

    Parameters
    ----------
    name : str
        The name of the JinjaMetric.
    metric : Metric | dict
        A :py:class:`Metric` or a dict that can be used to initialize a :py:class:`Metric`.
        Not added to :py:attr:`world.metrics`.
        Can be referenced as ``self.metric`` or ``metric`` in the Jinja expression.
    metrics : Iterable[Metric | dict]
        Additional metrics to use in the Jinja expression. Not added to :py:attr:`world.metrics`.
        This becomes a list which can be referenced as ``self.metrics`` or ``metrics`` in the Jinja expression.
    template : str
        A Jinja template that can be used to set variables that can be used in the expression.
        The rendered result of the template is not used. The template is always evaluated,
        so variables exported can be used in ``expression``, ``eval_condition``, and ``save_condition``.
    expression : str
        The Jinja expression to use. The return value becomes the value of the metric.
    eval_condition : Callable | str | None
        A Callable or Jinja expression that evaluates to a boolean.
        If the condition evaluates to False, the submetrics and ``self.expression`` will not be evaluated.
        If None, the Jinja expression will always be evaluated.
    save_condition : Callable | str | None
        A Callable or Jinja expression that evaluates to a boolean.
        If the condition evaluates to False, ``self.set_value`` will not be called and
        the result of ``self.expression`` will not be saved.
        If None, the metric will always be saved.
    default : Any, default='__unset__'
        The initial value of the metric. If ``'__unset__'``, the value will be inherited from
        :py:class:`Metric`.
    default_aggregation : str
        The default aggregation to use for the metric.
    """

    def __init__(
        self,
        name='__class__',
        history=None,
        metric=None,
        metrics=None,
        template=None,
        expression=None,
        # aux_expressions=(),
        eval_condition=None,
        save_condition=None,
        default='__unset__',
        default_aggregation=None,
    ):
        self._default = default
        self.default_aggregation = default_aggregation
        HasSubMetrics.__init__(self, metric=metric, metrics=metrics)
        super().__init__(name=name, history_size=history)
        self.template_src = template
        self.template = None
        self._module = None
        self.expression = expression
        self.eval_condition = eval_condition
        self.save_condition = save_condition
        self.exprargs = {
            'self': self,
            'metric': self.metric,
            'metrics': self.metrics,
        }

    def reset(self):
        super().reset()
        self.t = 0
        if self._default != '__unset__':
            self.current_value = self._default
        self.time_activated = None
        if isinstance(self.metric, Metric):
            self.metric.reset()
        if self.metrics is not None:
            for metric in self.metrics:
                if isinstance(metric, Metric):
                    metric.reset()

    @Metric.world.setter
    def world(self, value):
        HasSubMetrics.world.fset(self, value)
        if self.world and self.template_src is not None:
            self.template = self.world.jenv.from_string(self.template_src)

    def make_module(self, **kwargs):
        if self.template is None:
            return None
        return self.template.make_module(self.exprargs | kwargs)

    @property
    def module(self):
        if self.template is None:
            return None
        if self.template.saved_module is not None:
            return self.template.saved_module
        elif self._module is None:
            self._module = self.make_module()
            return self._module
        return self._module

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
        value = self.eval_template(self.expression)
        if self.save_condition is not None and not self.eval_template(self.save_condition):
            return
        self.set_value(value)
