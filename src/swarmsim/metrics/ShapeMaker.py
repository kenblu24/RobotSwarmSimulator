import numpy as np
import math
from .AbstractMetric import AbstractMetric

# typing
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None
# This metric is based of the idea that if you have one agent and take the distance of the distance of the two closest agents
# then make those distances the same as a chosen value (in this case 0.1 meters shorter than their view distance) they should be make
# a shape with sides equal to the number of agents.
class ShapeMaker(AbstractMetric):
    __badvars__ = AbstractMetric.__badvars__ + ['population']  # references to population may cause pickling errors

    def __init__(self, history=100, regularize=True):
        super().__init__(name="Shape Maker", history_size=history)
        self.population = None
        self.regularize = regularize
        self.allpairs = []
        self.lines = []

    def attach_world(self, world: RectangularWorld):
        super().attach_world(world)
        self.population = world.population
        self.world_size = world.config.size
        self.world_radius = world.config.radius    

    def calculate(self):
        side_distance = 0.9
        positions = [agent.getPosition() for agent in self.population]
        centroid = np.mean(positions, axis=0)
        vectors = positions - centroid
        
        angles = [a + 2 * np.pi if a < 0 else a for a in np.arctan2(vectors[:, 1], vectors[:, 0])]
        
        # angle_tups = np.array([a for a in zip(angles, vectors)])
        angle_tups = [a for a in zip(angles, vectors)]

        sorted_angle_tups = sorted(angle_tups, key=lambda x: x[0])
        
        angle_differences = []
        vector_differences = []
        for i in range(len(sorted_angle_tups) - 1):
            
            angle_differences.append(sorted_angle_tups[i + 1][0] - sorted_angle_tups[i][0])
            vector_differences.append(np.linalg.norm(sorted_angle_tups[i + 1][1] - sorted_angle_tups[i][1]))

        angle_differences.append(2 * np.pi - sorted_angle_tups[-1][0] + sorted_angle_tups[0][0])
        vector_differences.append(np.linalg.norm(sorted_angle_tups[-1][1] - sorted_angle_tups[0][1]))

        get_deviation = lambda arr, val: np.sqrt(np.sum(np.square(np.asarray(arr) - val)) / (len(arr) - 1))
        
        vector_deviation = get_deviation(vector_differences, side_distance)
        angle_deviation = np.std(np.asarray(angle_differences))
        
        total_deviation = -1 * (vector_deviation + angle_deviation)
        self.set_value(total_deviation)
       
    
    