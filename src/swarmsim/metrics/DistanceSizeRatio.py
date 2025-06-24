import numpy as np
import math
from .RadialVariance import RadialVarianceMetric


class DistanceSizeRatio(RadialVarianceMetric):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = self.__class__.__name__

    # This function creates a metric for the agents to follow

    def _calculate(self, world):
        agent_radii = []

        positions = [np.asarray(agent.getPosition()) for agent in self.population]

        for pos in positions:
            distances = [self.distance(pos, other_pos) for other_pos in positions if other_pos != pos]

            # distance to nearest wall
            # https://gamedev.stackexchange.com/questions/44483/how-do-i-calculate-distance-between-a-point-and-an-axis-aligned-rectangle
            # clamp point to within rectangle, then calculate distance
            # slightly less optimized but MILES more readable
            w, h, wd = world.config.size[0], world.config.size[1], world.config.size
        
            # x, y = pos
            # is_inside_world = (pos >= np.zeros(2)).all() and (pos <= wd).all()
            # if not is_inside_world:
            #     # nearest_wall_point = np.array((np.clip(x, 0, w), np.clip(y, 0, h)))
            #     # distance_to_wall = self.distance(pos, nearest_wall_point)
            #     # smallest_distance = min(distance_to_wall, *distances)
            #     smallest_distance = 0
            # else:
            #     smallest_distance = min(distances)

            agent_radii.append(min(distances))
            

        diffusal_uneveness = np.std(agent_radii) # variability in distances

        self.set_value(-diffusal_uneveness)
        # self.set_value(agent_radii[0])

    def distance(a, b):
        return np.linalg.norm(a - b)
