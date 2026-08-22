import numpy as np
import math
from .Circliness import RadialVarianceHelper


class CirclePacking(RadialVarianceHelper):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = self.__class__.__name__
    
    def _calculate(self):
        world = self.world.config
        agent_radii = []

        positions = [agent.getPosition() for agent in self.population]

        for agent in self.population:
            distances = []
            
            for position in positions:
                distance = self.distance(agent.getPosition(), position)
                
                if (distance != 0):
                    distances.append(distance)
            
            distances.sort()
            
            # TODO: this is a bit of a hack, but it works
            # breakpoint()
            if (agent.getPosition()[0] < (world.w / 2) and agent.getPosition()[1] < (world.h / 2)): # bottom left
                if(agent.getPosition()[0] < distances[0]):
                    agent_radii.append(agent.getPosition()[0])
                elif(agent.getPosition()[1] < distances[0]):
                    agent_radii.append(agent.getPosition()[1])
                else:
                    agent_radii.append(distances[0])
                

            elif (agent.getPosition()[0] > (world.w / 2) and agent.getPosition()[1] < (world.h / 2)): # bottom right
                if(world.w - agent.getPosition()[0] < distances[0]):
                    agent_radii.append(world.w - agent.getPosition()[0])
                elif(agent.getPosition()[1] < distances[0]):
                    agent_radii.append(agent.getPosition()[1])
                else:
                    agent_radii.append(distances[0])

            elif (agent.getPosition()[0] < (world.w / 2) and agent.getPosition()[1] > (world.h / 2)): # top left
                if(agent.getPosition()[0] < distances[0]):
                    agent_radii.append(agent.getPosition()[0])
                elif(world.h - agent.getPosition()[1] < distances[0]):
                    agent_radii.append(world.h - agent.getPosition()[1])
                else:
                    agent_radii.append(distances[0])
            
            elif (agent.getPosition()[0] > (world.w / 2) and agent.getPosition()[1] > (world.h / 2)): # top right
                if(world.w - agent.getPosition()[0] < distances[0]):
                    agent_radii.append(world.w - agent.getPosition()[0])
                elif(world.h - agent.getPosition()[1] < distances[0]):
                    agent_radii.append(world.h - agent.getPosition()[1])
                else:
                    agent_radii.append(distances[0])
            
        agent_radii.sort()
        
        return agent_radii[0]

    @staticmethod
    def distance(a, b):
        return np.linalg.norm(a - b)
    

