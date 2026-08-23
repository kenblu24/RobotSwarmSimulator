from typing import Callable

import numpy as np
from numpy.typing import NDArray

from swarmsim.util.statistics_tools import Remap, RemapNP


def remapper_helper(transform_fn: Callable[[NDArray], NDArray], step=0.5):
    inputs = np.arange(-10, 10, step)
    outputs = transform_fn(inputs)

    orig_rmp = Remap(inputs, outputs)
    np_rmp = RemapNP(inputs, outputs)

    test_inputs = inputs + step * 0.5
    orig_result = np.array([orig_rmp(x) for x in test_inputs])
    np_result = np_rmp(test_inputs)

    assert np.allclose(orig_result, np_result)

def test_remappers():
    remapper_helper(transform_fn=lambda arr: arr + 0.5)
    remapper_helper(transform_fn=lambda arr: arr * arr)
    remapper_helper(transform_fn=np.sin)
    remapper_helper(transform_fn=np.exp)
    remapper_helper(transform_fn=lambda arr: np.exp(np.sin(arr)) + 0.4)
