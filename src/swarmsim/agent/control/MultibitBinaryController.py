import numpy as np

from .AbstractController import AbstractController
from ...util import statistics_tools as st

from typing import Sequence

ConstantOutputValues = tuple[float, ...] | np.ndarray


class MultiBinaryController(AbstractController):
    """Controller that returns one of two constant control inputs based on a binary sensor state.

    There are multiple ways to specify the control values to be output:

    1. If ``a`` and ``b`` are both 1D arrays, then:
        a. ``a`` is the output when the sensor is in state 0,
        b. ``b`` is the output when the sensor is in state 1.
    2. If only ``a`` is specified, then it should either be a 1D array or a 2D array.
        a. The first half or first column is the output when the sensor is in state 0,
        b. and the second half or second column is the output when the sensor is in state 1.

    If ``b`` is specified, then it should be the same length as ``a``.

    Parameters
    ----------
    a : tuple[float, ...] | tuple[tuple[float, ...], ...] | np.ndarray
    b : tuple[float, ...] | np.ndarray | None, optional
    agent : Agent, optional
    parent : Agent, optional
    sensor_id : int, default=0
        The index in ``agent.sensors`` to use for the sensor.
    sense_avg_time : int, default=1
        The number of timesteps to average over when calculating the sensor state.
        This is useful for averaging over the sensor state to smooth out noise, i.e.,
        a low-pass filter.

    """
    def __init__(self, outputs: Sequence[ConstantOutputValues], default_output: ConstantOutputValues | None = None,
                 agent=None, parent=None, sensor_ids=None, sense_avg_time=1, **kwargs):
        super().__init__(agent=agent, parent=parent, **kwargs)
        self.outputs = outputs
        self.default_output = default_output
        self.sensor_ids = sensor_ids

    @property
    def outputs(self):
        return self._outputs

    @outputs.setter
    def outputs(self, outputs):
        # normalize type of dict keying
        if isinstance(outputs, dict):
            mapit = outputs.items()
        else:
            mapit = enumerate(outputs)
        self._outputs = {int(k): v for k, v in mapit}

    def get_sensor_bitstring(self, agent):
        sensors = agent.sensors
        bits = 0
        ids = range(len(sensors)) if self.sensor_ids is None else self.sensor_ids
        for sensor_id in reversed(ids):
            sensor = sensors[sensor_id]
            bits = bits << 1
            bits |= bool(sensor.current_state)
        return bits

    def get_actions(self, agent):
        bits = self.get_sensor_bitstring(agent)
        return self.outputs.get(bits, self.default_output)
