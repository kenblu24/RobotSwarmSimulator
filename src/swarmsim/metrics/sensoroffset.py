import math

from .metric import Metric


class GeneElementDifference(Metric):

    def __init__(self, genome_a_index, genome_b_index, history=None):
        super().__init__(name="Sensor_Offset", history_size=history)
        self.a = genome_a_index
        self.b = genome_b_index

    def calculate(self):
        a_theta = self.parent.population[0].controller[self.a]
        b_theta = self.parent.population[0].controller[self.b]
        sensor_angle = abs(a_theta - b_theta) / (2 * math.pi)
        self.set_value(sensor_angle)
