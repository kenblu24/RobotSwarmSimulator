"""Abstract Static Sensor class.

.. autoclass:: StaticSensor
    :members:
    :undoc-members:

"""

from .Sensor import Sensor


class StaticSensor(Sensor):
    """Base Static Sensor class for the agent. This class should not be instantiated directly.
    """
    def __init__(self, parent=None):

        super(StaticSensor, self).__init__(parent=parent)
        self.current_state = 0

    def step(self, population):
        pass

    def draw(self, screen, zoom=1.0):
        pass
