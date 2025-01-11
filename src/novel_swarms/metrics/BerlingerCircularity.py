import numpy as np
import math
import circle_fit
from .Circliness import RadialVarianceHelper

# typing
from typing import Callable


class InstantaneousCircularity(RadialVarianceHelper):
    circle_fit_method = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.rfit: float = None

    def _calculate(self):
        if self.circle_fit_method is None:
            raise RuntimeError("circle_fit_method is not specified. InstantaneousCircularity should not be used, use a subclass instead.")  # noqa: E501

        # def circle_fitter(positions: list) -> tuple[float, float, float, float]
        circle_fitter: Callable = getattr(circle_fit, self.circle_fit_method)

        positions = [agent.getPosition() for agent in self.population]

        # fit a circle using the method specified on self.circle_fit_method
        xc, yc, r, self.rfit = circle_fitter(positions)

        n = len(self.population)
        cfit = np.asarray([xc, yc])

        sigma = float(np.linalg.norm(positions - (n * cfit)))

        return sigma / r


class InstantLSQCircularity(InstantaneousCircularity):
    circle_fit_method = "standardLSQ"


class InstantHyperLSQCircularity(InstantaneousCircularity):
    circle_fit_method = "hyperLSQ"


class InstantRiemannCircularity(InstantaneousCircularity):
    circle_fit_method = "riemannSWFLa"


class InstantLMCircularity(InstantaneousCircularity):
    circle_fit_method = "lm"


class InstantPrattSVDCircularity(InstantaneousCircularity):
    circle_fit_method = "prattSVD"


class InstantTaubinSVDCircularity(InstantaneousCircularity):
    circle_fit_method = "taubinSVD"


class InstantHyperSVDCircularity(InstantaneousCircularity):
    circle_fit_method = "hyperSVD"


class InstantKMHCircularity(InstantaneousCircularity):
    circle_fit_method = "kmh"