from warnings import warn
from typing import Tuple

from numpy import average

from ..util.collections import RefProp, RefListProp, RefList

import typing
if typing.TYPE_CHECKING:
    from ..world.World import World
else:
    World = None


class Metric():
    __badvars__ = ['world']  # variables that should not be pickled
    #: Set to True if the metric should not be averaged over its history
    default_aggregation = None
    _world = None

    def __init__(self, name: str = '__class__', history_size: int | None = None):
        self.name = self.__class__.__name__ if name == '__class__' else name
        self.history_size = history_size
        self._world: World = None  # pyright: ignore[reportAttributeAccessIssue]
        self._parent = None
        self.reset()

    def reset(self):
        self.current_value = None
        self.value_history = []

    def attach_world(self, world):
        warn("Calling Metric.attach_world() is deprecated. There is no need to call this method directly."
             "If you are adding run-on-attach_world logic to a metric, override the Metric.world.setter property instead."
             , DeprecationWarning, stacklevel=2)
        self.world = world

    @property
    def world(self):
        return self._world

    @world.setter
    def world(self, value):
        self._world = value

    @property
    def parent(self):
        return self._parent or self.world

    @parent.setter
    def parent(self, value):
        self._parent = value

    def set_value(self, value):
        # Keep Track of the [self.history_size] most recent values
        self.value_history.append(value)
        if self.history_size is not None and len(self.value_history) > self.history_size:
            self.value_history = self.value_history[-self.history_size:]

        self.current_value = value

    def out_current(self) -> Tuple:
        try:
            return (self.name, self.value_history[-1])
        except IndexError:
            return (self.name, None)

    @property
    def value(self):
        return self.current_value

    def out_average(self) -> Tuple:
        return (self.name, average(self.value_history))

    @property
    def average(self):
        return average(self.value_history)

    def draw(self, screen, zoom=1.0):
        pass

    def as_config_dict(self):
        return {"name": self.name, "history_size": self.history_size}

    def calculate(self):
        pass

    # prevent pickling errors
    def __getstate__(self):
        d = self.__dict__.copy()
        for k in self.__badvars__:
            d.pop(k, None)
        return d


class HasSubMetric:
    def __init__(self, metric=None):
        self.metric = metric

    @Metric.world.setter
    def world(self, value):
        Metric.world.fset(self, value)
        if isinstance(self.metric, dict):
            self._metric = self.setup_submetric(self.metric)
            return
        try:
            self.metric.world = value
        except AttributeError:
            pass

    def setup_submetric(self, metric):
        if self.world and metric is not None:
            metric = self.world.add_metric(metric, add_to_world=False)
            metric.world = self.world
        return metric

    metric = RefProp('parent', set_callbacks=[setup_submetric], extra_names={'world': 'world'})


class HasSubMetrics(HasSubMetric):
    metrics = RefListProp()

    def __init__(self, metric=None, metrics=None):
        super().__init__(metric=metric)
        self._metrics = RefList(self, 'parent', add_callbacks=[self.setup_submetric],
                                extra_names={'world': 'world'})
        self.metrics = metrics or []

    @Metric.world.setter
    def world(self, value):
        HasSubMetric.world.fset(self, value)
        self.metrics = [self.setup_submetric(metric) for metric in self.metrics]
        for metric in self.metrics:
            try:
                metric.world = value
            except AttributeError:
                pass
