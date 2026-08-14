import math
import warnings

import numpy as np
import pytest

from swarmsim.metrics.Metric import Metric
from swarmsim.metrics.RadialVariance import RadialVarianceMetric


class TestMetricAverageEmptyHistory:
    """Regression tests for issue #1: averaging an empty metric history
    used to emit a numpy "Mean of empty slice" RuntimeWarning (via
    ``np.average([])``) instead of returning ``nan`` cleanly.
    """

    def test_out_average_empty_history_returns_nan_without_warning(self):
        metric = Metric(name="test")
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            name, value = metric.out_average()
        assert name == "test"
        assert math.isnan(value)

    def test_average_property_empty_history_returns_nan_without_warning(self):
        metric = Metric(name="test")
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            value = metric.average
        assert math.isnan(value)

    def test_out_average_nonempty_history_still_averages(self):
        metric = Metric(name="test")
        metric.set_value(2.0)
        metric.set_value(4.0)
        name, value = metric.out_average()
        assert name == "test"
        assert value == 3.0
        assert metric.average == 3.0


class _FakeAgent:
    def __init__(self, pos):
        self._pos = np.array(pos, dtype=float)

    def getPosition(self):
        return self._pos


class _FakeWorldConfig:
    radius = 10.0


class _FakeWorld:
    def __init__(self, population):
        self.population = population
        self.config = _FakeWorldConfig()


class TestRadialVarianceEmptyPopulation:
    """Regression tests for issue #1: an empty population used to make
    RadialVarianceMetric.calculate() raise ZeroDivisionError (the scaling
    factor divides by ``n = len(self.population)``) and center_of_mass()
    emit a numpy "Mean of empty slice" RuntimeWarning.
    """

    def test_calculate_empty_population_sets_nan_without_crashing(self):
        metric = RadialVarianceMetric()
        metric.attach_world(_FakeWorld(population=[]))
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            metric.calculate()  # must not raise ZeroDivisionError
        assert math.isnan(metric.value)

    def test_center_of_mass_empty_population_returns_nan_without_warning(self):
        metric = RadialVarianceMetric()
        metric.population = []
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            com = metric.center_of_mass()
        assert com.shape == (2,)
        assert np.all(np.isnan(com))

    def test_calculate_nonempty_population_still_computes_a_value(self):
        population = [_FakeAgent((1.0, 0.0)), _FakeAgent((-1.0, 0.0))]
        metric = RadialVarianceMetric()
        metric.attach_world(_FakeWorld(population=population))
        metric.calculate()
        assert not math.isnan(metric.value)
        assert metric.value == pytest.approx(0.0)
