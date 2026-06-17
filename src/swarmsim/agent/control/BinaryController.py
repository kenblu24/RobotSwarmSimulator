import numpy as np

from .AbstractController import AbstractController
from ...util import statistics_tools as st

ConstantOutputValues = tuple[float, ...] | np.ndarray
TwoConstantOutputValues = tuple[ConstantOutputValues, ConstantOutputValues] | np.ndarray


class BinaryController(AbstractController):
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
    def __init__(self, a: TwoConstantOutputValues | ConstantOutputValues, b: ConstantOutputValues | None = None,
                 agent=None, parent=None, sensor_id=0, sense_avg_time=1, **kwargs):
        super().__init__(agent=agent, parent=parent, **kwargs)
        self.sensor_id = sensor_id  # use this to determine which sensor on the agent to use
        self.sense_avg = st.Average(sense_avg_time)

        # set self.a and self.b, the two sets of constant output values.
        a = np.asarray(a, dtype='float64')
        if not 0 < len(a.shape) < 2:
            raise ValueError("Expected first argument to be a 1D or 2D array")
        if a.shape[0] == 2 and b is None:
            self.a, self.b = a
        elif len(a.shape) == 1:
            if b is None and len(a) % 2 == 0:
                self.a = a[len(a) // 2:]
                self.b = a[:len(a) // 2]
            else:
                b = np.asarray(b, dtype='float64')
                if len(b.shape) != 1 or b.shape[0] != a.shape[0]:
                    raise ValueError("Expected constant output values to be 1D arrays of same size")
                self.a, self.b = a, b
        else:
            raise ValueError("Expected argument(s) to be 1D arrays of same length or single 2D array")

        # print(self.a, self.b)

    def get_actions(self, agent):
        """
        An example of a "from scratch" controller that you can code with any information contained within the agent class
        """
        detected = agent.sensors[self.sensor_id].current_state
        smoothed_detected = self.sense_avg(detected)

        if smoothed_detected:
            return self.b
        else:
            return self.a

    def as_config_dict(self):
        return {'a': self.a, 'b': self.b, 'sensor_id': self.sensor_id}
