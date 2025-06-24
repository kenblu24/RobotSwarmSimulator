""" PID Controller class

Example
-------
.. code-block:: python

    from pid import PID
    controller = PID(p=0.07, i=0, imax=90)
    target = 100
    state = 0
    while(True):
        error = target - state
        state = controller(error)
        # control value with output

.. seealso::
    This class was adapted from the following micropython code:
    https://github.com/openmv/openmv/blob/master/scripts/libraries/pid.py
"""

import time
from math import pi, isnan


def clamp(x, xmin, xmax):
    return max(xmin, min(x, xmax))


class PID:
    reset_time = 100  # if PID hasn't been updated for this much time, reset derivative and integrator
    d_smoothing = 1 / (2 * pi * 20)  # derivative smoothing (RC time constant)
    i_decay = 1 / (2 * pi * 20)  # decay time for integrator (RC time constant)

    def __init__(self, p: float = 0, i: float = 0, d: float = 0, imax: int = 0):
        """PID Class

        Parameters
        ----------
        p : float, optional
            proportional gain, by default 0
        i : float, optional
            integrator gain, by default 0
        d : float, optional
            derivative gain, by default 0
        imax : int, optional
            integrator max value, by default 0
            The integrator will be clamped to within ``[-imax, imax]``
        """
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = float("nan")
        self._integrator = 0
        self._last_t = 0

    def __call__(self, error, time=None, dt=None):
        return self.get_pid(error, time, dt)

    def get_pid(self, error, time=None, dt=None):
        """ Calculate the PID output

        Parameters
        ----------
        error : float
            The error value
        time : float, optional
            The current time. If you specify this, don't specify ``dt``.
        dt : float, optional
            Elapsed time since the last call. If you specify this, don't specify ``time``.

        Returns
        -------
        float
            The PID output
        """
        # calculate dt, update time
        if time is not None and dt is not None:
            raise ValueError("time and dt cannot be set at the same time")
        elif time is not None:
            dt = time - self._last_t
            self._last_t = time
        elif dt is not None:
            self._last_t += dt
        else:
            dt = 1

        if self._last_t == 0 or dt > self.reset_time:
            dt = 0
            self.reset_I()

        # p term
        output = error * self._kp

        # d term
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                derivative = 0
                self._last_derivative = 0
            else:
                derivative = (error - self._last_error) / dt
            derivative = self._last_derivative + ((dt / (self._RC + dt)) * (derivative - self._last_derivative))
            self._last_derivative = derivative
            output += self._kd * derivative
        self.last_error = error

        if abs(self._ki) > 0 and dt > 0:
            self._integrator *= self.i_decay
            self._integrator += (error * self._ki) * dt
            self._integrator = clamp(self._integrator, -self._imax, self._imax)
            if isnan(self._integrator):
                self.reset_I()
            output += self._integrator
        return output

    def reset_I(self):
        self._integrator = 0
        self._last_derivative = float("nan")


class LivePID(PID):

    def __call__(self, error):
        return self.get_pid(error)

    def get_pid(self, error):
        t = time.time_ns() / 1e9
        return super().__call__(error, time=t)
