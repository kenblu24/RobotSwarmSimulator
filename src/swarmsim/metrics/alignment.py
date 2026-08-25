import numpy as np
from .metric import Metric

from ..util.geometry.euclidean import vectorize

class Alignment(Metric):
    def __init__(
        self,
        name="Alignment",
        default=0,
        pre_exponent=1,
        post_exponent=-1,
        history=None,
        default_aggregation='average',
    ):
        super().__init__(name=name, history_size=history)
        self.default = default
        self.pre_exponent = pre_exponent
        self.post_exponent = post_exponent

    def calculate(self):
        population = self.parent.population
        angles = np.array([p.angle for p in population])
        # NOTE: This does not returns [[x1, y1], [x2, y2], ..., [xn, yn]];
        #       it returns [[x1, x2, ..., xn], [y1, y2, ..., yn]]
        vectors = vectorize(angles).T
        avg_heading = np.mean(vectors, axis=0)
        avg_heading /= np.linalg.norm(avg_heading)

        differences = np.dot(vectors, avg_heading) ** self.pre_exponent
        self.set_value(np.mean(differences) ** self.post_exponent)
