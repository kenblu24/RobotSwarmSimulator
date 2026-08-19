import numpy as np
from .Metric import Metric


def vectorize(angle):
    """Convert an angle to a representative unit vector.

    Parameters
    ----------
    angle : float
        Angle in radians

    Returns
    -------
    np.ndarray
        Vector
    """
    return np.array((np.cos(angle), np.sin(angle)))


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
        super().__init__(name=name, history_size=history, default_aggregation=default_aggregation)
        self.default = default
        self.pre_exponent = pre_exponent
        self.post_exponent = post_exponent

    def calculate(self):
        population = self.parent.population
        angles = np.array([p.angle for p in population])
        vectors = vectorize(angles)
        avg_heading = np.mean(vectors, axis=0)
        avg_heading /= np.linalg.norm(avg_heading)

        differences = np.dot(vectors, avg_heading) ** self.pre_exponent
        self.set_value(np.mean(differences) ** self.post_exponent)
