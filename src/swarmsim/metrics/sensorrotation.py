import math

from .metric import Metric


class SensorRotation(Metric):

    def __init__(self, sensor_index, history=None):
        super().__init__(name="Sensor_Rotation", history_size=history)
        self.i = sensor_index

    def calculate(self):
        sensor_angle = self.parent.population[0].controller[self.i] / (2 * math.pi)
        self.set_value(sensor_angle)
