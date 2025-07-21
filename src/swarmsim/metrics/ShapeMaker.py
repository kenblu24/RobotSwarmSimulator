import numpy as np
import math
from .AbstractMetric import AbstractMetric
import pygame

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

    # By taking the centroid of the population and finding the vectors from the centroid to each agent, we can find the angles between the vectors.
    # We then sort the vectors by angle and take the difference between the angles between each vector and the next to get every angle between each vector.
    # We then take the difference between each vector to get a vector from each agent to the next closest agent.
    # Finally, we take the deviation of the vector differences and the view distance of the agents.
    # And the standard deviation of the angle differences. Returning the sum of these two.
    # This gives a value describing the relationship between the distance between an agent and its closest neightbor.
    # By trying to maximize this value, it will try to make the agents for a polygon/shape with as many sides as agents, 
    # with each side length being just under their view distance.
    # i.e. 4 agents = square like shape, 5 agents = pentagon, etc.
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
        lines = []
        for i in range(len(sorted_angle_tups) - 1):
            
            angle_differences.append(sorted_angle_tups[i + 1][0] - sorted_angle_tups[i][0])
            vector_differences.append(np.linalg.norm(sorted_angle_tups[i + 1][1] - sorted_angle_tups[i][1]))
            lines.append([sorted_angle_tups[i][1], sorted_angle_tups[i + 1][1]])

        angle_differences.append(2 * np.pi - sorted_angle_tups[-1][0] + sorted_angle_tups[0][0])
        vector_differences.append(np.linalg.norm(sorted_angle_tups[-1][1] - sorted_angle_tups[0][1]))
        lines.append([sorted_angle_tups[-1][1], sorted_angle_tups[0][1]])
        self.lines = np.asarray(lines + centroid)

        get_deviation = lambda arr, val: np.sqrt(np.sum(np.square(np.asarray(arr) - val)) / (len(arr) - 1))
        
        vector_deviation = get_deviation(vector_differences, side_distance)
        angle_deviation = np.std(np.asarray(angle_differences))

        total_deviation = -1 * (vector_deviation + angle_deviation)
        self.set_value(total_deviation)
       
    def draw(self, screen, offset):
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen, offset)

        for line in self.lines:
            pygame.draw.line(screen, (128, 128, 128), *line * zoom + pan, width=1)

        # for centroid in self.centroids:
        #     pygame.draw.circle(screen, (128, 128, 128), centroid * zoom + pan, radius=5, width=1)
    

                

        
                    
                
        
            
            
            
            


    