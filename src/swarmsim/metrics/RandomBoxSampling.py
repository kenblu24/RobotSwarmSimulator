import numpy as np
from .Circliness import RadialVarianceHelper
from random import sample

class RandomBoxSampling(RadialVarianceHelper):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = self.__class__.__name__
        
    def _calculate(self):
        world = self.world.config


        boxes = []
        locations = []
        pop_count = len(self.population)
        for i in range(world.w // pop_count):
            for j in range(world.h // pop_count):
                boxes.append([i * world.w / pop_count, j * world.h / pop_count])
        
        # The for loop below uses the positions array to see if the agent is in a box
        # If it is, it adds the box to the list of boxes
        for agent in self.population:
            for box in boxes:
                if (agent.getPosition()[0] > box[0] and agent.getPosition()[0] < box[0] + world.w / pop_count and agent.getPosition()[1] > box[1] and agent.getPosition()[1] < box[1] + world.h / pop_count):
                    locations.append(box)

        #the next part of code returns the metric for the simulation, the metric needs to be a number that increases as long as there is only one agent in each box
        temp = 0
        for location in locations:
            if location in locations[:-1]:
                temp -= 1

        return len(locations) + temp
        
        
        
                
            
            
            
            
                    

    @staticmethod
    def distance(a, b):
        return np.linalg.norm(a - b)
            
    