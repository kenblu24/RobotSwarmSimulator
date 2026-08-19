from numbers import Number
import numpy as np
from .Metric import Metric
from ..util.geometry.euclidean import fast_pairwise_distances


class Separation(Metric):
    def __init__(
        self,
        name="Separation",
        default=0,
        linear=None,
        exponential=None,
        clamp=None,
        normalize=False,
        reduce_agent_distances=None,
        reduce_distances=None,
        history=None,
        default_aggregation='average',
    ):
        super().__init__(name=name, history_size=history, default_aggregation=default_aggregation)
        self.default = default
        if linear is not None and exponential is not None:
            raise ValueError("Separation metric has both linear and exponential specified."
                             " Only one of linear or exponential can be specified.")
        if isinstance(linear, Number):
            self.linear = linear
        elif linear is not None:
            self.linear = np.asarray(linear)
            if linear.ndim == 1:
                if len(self.linear) != 2:
                    raise ValueError("linear= argument in Separation Metric must be a 1D array of length 2.")
                if clamp is not None:
                    self.linear = np.array([self.linear, clamp])
                elif normalize == True:
                    self.linear = np.array([self.linear, [0, 1]])
            elif linear.ndim == 2:
                self.linear = linear
            else:
                raise ValueError("linear= argument in Separation Metric must be a 1D or 2D array.")






    def calculate(self):
        positions = np.array([p.position for p in self.parent.population])
        distances = fast_pairwise_distances(positions)

        if self.linear:
            if self.linear.ndim == 1:
                if




        self.set_value()
