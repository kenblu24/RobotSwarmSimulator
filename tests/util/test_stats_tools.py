import numpy as np
from swarmsim.util.statistics_tools import Remap, RemapNP


def test_remap1():
    inputs = np.arange(0, 10, step=0.5)
    outputs = inputs * inputs

    remap01 = Remap(inputs, outputs)

    # TODO: create numpy-based remapper
    # TODO: compare output against the classical `Remap` class