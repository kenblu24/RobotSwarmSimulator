from numbers import Number
import numpy as np
from .metric import Metric
from ..util.geometry.euclidean import fast_pairwise_distances
from ..util.statistics_tools import RemapNP


aggregation_functions = {
    'mean': np.mean,
    'avg': np.mean,
    'average': np.mean,
    'sum': np.sum,
    'total': np.sum,
    'max': np.max,
    'maximum': np.max,
    'farthest': np.max,
    'min': np.min,
    'minimum': np.min,
    'closest': np.min,
    'nearest': np.min,
    'None': None,
    None: None,
}


class ExponentialFunction:
    def __init__(self, a, b, c, d, e):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.e = e

    def __call__(self, x):
        return self.a * np.exp(self.b * x + self.c) + self.d * x + self.e


class Separation(Metric):
    def __init__(
        self,
        name="Separation",
        default=0,
        linear=None,
        exponential=None,
        clamp=None,
        normalize=False,
        remapper=None,
        reduce_agent_distances=None,
        reduce_distances='mean',
        history=None,
        default_aggregation='average',
    ):
        super().__init__(name=name, history_size=history)
        self.default_aggregation = default_aggregation
        self.default = default
        self.clamp = np.sort(clamp) if clamp is not None else clamp
        self.normalize = normalize
        self.reduce_agent_distances = aggregation_functions.get(reduce_agent_distances, reduce_agent_distances)
        self.reduce_distances = aggregation_functions.get(reduce_distances, reduce_distances)
        if not any((linear, exponential, remapper)):
            raise ValueError("Separation metric must have at least one of linear, exponential, or remapper specified.")
        elif (linear, exponential, remapper).count(None) != 2:
            raise ValueError("Separation metric has both linear and exponential specified."
                             " Only one of linear or exponential can be specified.")

        self.p = None
        if isinstance(linear, Number):
            self.p = linear
        elif linear is not None:
            self.linear = np.asarray(linear)
            if self.linear.ndim == 1:
                if len(self.linear) != 2:
                    raise ValueError("linear= argument in Separation Metric must be a 1D array of length 2.")
                if clamp is not None:
                    self.linear = np.array([self.linear, clamp])
                elif normalize is True:
                    self.linear = np.array([self.linear, [0, 1]])
                    self.normalize = False
                else:
                    raise ValueError("linear= argument in Separation Metric must be a 1D array of length 2; however, neither clamp nor normalize are set")

            elif self.linear.ndim != 2:
                raise ValueError("linear= argument in Separation Metric must be a 1D or 2D array.")

            self.remap = RemapNP(*self.linear)
        elif exponential:
            if isinstance(exponential, dict):
                self.exponential = ExponentialFunction(**exponential)
            else:
                self.exponential = ExponentialFunction(*exponential)
            self.remap = self.exponential
        else:
            if not callable(remapper):
                raise ValueError("remapper= argument in Separation Metric must be callable.")
            self.remap = remapper

    def calculate(self):
        positions = np.array([p.position for p in self.parent.population])
        distances = fast_pairwise_distances(positions, collapse_diagonal_along=0)

        if self.p:
            remapped_distances = self.p * distances
        elif self.remap:
            # NOTE: 'mean' will mistakenly count agent-to-self distances
            remapped_distances = self.remap(distances)
        else:
            raise ValueError("Separation metric must have at least one of linear, exponential, or remapper specified.")

        if self.clamp:
            remapped_distances = np.clip(remapped_distances, *self.clamp)
        if self.normalize:
            remapped_distances /= np.linalg.norm(remapped_distances)
        if self.reduce_agent_distances:
            remapped_distances = self.reduce_agent_distances(remapped_distances, axis=1)
        self.set_value(self.reduce_distances(remapped_distances))
