import numpy as np
from .Circliness import RadialVarianceHelper
import rss

class CoordinateTest(RadialVarianceHelper):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = self.__class__.__name__

    # This function creates a metric for the agents to follow
    
    def _calculate(self):
        
        
        # breakpoint()
    
            
        distances = [self.distance(agent.getPosition(), np.asarray([0, 0])) for agent in self.population]
        
        sum_distances = np.sum(distances, axis=0)
        # breakpoint()            
        
        return ((-1 * sum_distances))
    
    @staticmethod
    def distance(a, b):
        return np.linalg.norm(a - b)
    

