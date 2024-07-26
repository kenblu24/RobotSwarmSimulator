import numpy as np
from .Circliness import RadialVarianceHelper
import rss
from random import sample

class RandomBoxAggregation(RadialVarianceHelper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = self.__class__.__name__
        
    def _calculate(self):
        world = self.world.config


        boxes = []
        
        pop_count = len(self.population)
        for i in range(world.w // pop_count):
            for j in range(world.h // pop_count):
                boxes.append([i * world.w / pop_count, j * world.h / pop_count])

        box = sample(boxes, 1)[0]
        box_center = [box[0] + 5, box[1] + 5]
        
        positions = [agent.getPosition() for agent in self.population]

        for agent in self.population:
            distances = []
            
            for position in positions:
                distance = self.distance(position, box_center)
                
                if (distance != 0):
                    distances.append(distance)
            
            distances.sort()

            return -1 * distances[-1]


    
    @staticmethod
    def distance(a, b):
        return np.linalg.norm(a - b)