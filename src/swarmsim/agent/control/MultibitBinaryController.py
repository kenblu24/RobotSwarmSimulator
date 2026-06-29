import numpy as np

from .AbstractController import AbstractController
from ...util import statistics_tools as st

from typing import Sequence, Mapping, SupportsIndex

ConstantOutputValues = tuple[float, ...] | np.ndarray


class MultiBinaryController(AbstractController):
    """Controller that maps multiple binary sensors to constant control inputs.

    The controller is specified by a dictionary of sensor states to output values.

    Parameters
    ----------
    outputs : tuple[tuple[float, ...] | numpy.ndarray, ...] | dict[int | str, tuple[float, ...] | numpy.ndarray]
        The output values to return for each possible sensor state.
        If it is a dict, then each key should be a combined sensor state bitstring.
        If it is a tuple, then the index is the combined sensor state bitstring.
        Each output value should be a 1D array or a scalar.
    default_output : tuple[float, ...] | numpy.ndarray | None, optional
        The output value to return if the sensor state is not in the ``outputs`` dictionary.
    sensor_ids : list[int] | None, optional
        The sensor indices to use for the sensors.
    agent : Agent, optional
    parent : Agent, optional

    """
    def __init__(self, outputs: Sequence[ConstantOutputValues] | Mapping[int | str, ConstantOutputValues],
                 default_output: ConstantOutputValues | None = None,
                 agent=None, parent=None, sensor_ids=None, **kwargs):
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
