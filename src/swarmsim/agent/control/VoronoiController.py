import numpy as np
import pygame
from scipy import spatial

from swarmsim.agent.control.AbstractController import AbstractController
import swarmsim.util.statistics_tools as st
from math import copysign


trigger_remap = st.Remap([-1, 1], [0, 1])


def decay(x, decay=0.1):
    magnitude = np.clip(abs(x) - decay, 0., None)
    return copysign(magnitude, x)


class VoronoiController(AbstractController):
    def __init__(
        self, agent=None, parent=None,

        agent_memory= None,
        
    ):
        super().__init__(agent=agent, parent=parent)



    def get_actions(self, agent):
        

        return v, w
    
    
    
    
    # def draw(self, screen, offset):
    #     pan, zoom = np.asarray(offset[0]), offset[1]
    #     super().draw(screen, offset)

    #     for line in self.lines:
    #         pygame.draw.line(screen, (128, 128, 128), *line * zoom + pan, width=1)

    #     for centroid in self.centroids:
    #         pygame.draw.circle(screen, (128, 128, 128), centroid * zoom + pan, radius=5, width=1)

    # This is a modified version of the code from here:
    # https://stackoverflow.com/questions/28665491/getting-a-bounded-polygon-coordinates-from-voronoi-cells
    @staticmethod
    def in_box(towers, bounding_box):
        return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                            towers[:, 0] <= bounding_box[1]),
                            np.logical_and(bounding_box[2] <= towers[:, 1],
                                            towers[:, 1] <= bounding_box[3]))

    @staticmethod
    def voronoi(towers, bounding_box):
        # eps = sys.float_info.epsilon
        eps = 0.01
        # Select towers inside the bounding box
        i = VoronoiController.in_box(towers, bounding_box)
        # Mirror points
        points_center = towers[i, :]
        points_left = np.copy(points_center)
        points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
        points_right = np.copy(points_center)
        points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
        points_down = np.copy(points_center)
        points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
        points_up = np.copy(points_center)
        points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
        points = np.append(points_center,
                        np.append(np.append(points_left,
                                            points_right,
                                            axis=0),
                                    np.append(points_down,
                                            points_up,
                                            axis=0),
                                    axis=0),
                        axis=0)
        # Compute Voronoi
        if len(points) != 0:
            vor = spatial.Voronoi(points)
        else:
            return []
        # Filter regions
        regions = []
        for region in vor.regions:
            flag = True
            for index in region:
                if index == -1:
                    flag = False
                    break
                else:
                    x = vor.vertices[index, 0]
                    y = vor.vertices[index, 1]
                    if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                        bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
                        flag = False
                        break
            if region != [] and flag:
                regions.append(region)
        vor.filtered_points = points_center
        vor.filtered_regions = regions
        return vor
    
    @staticmethod
    def centroid_region(vertices):
        # Polygon's signed area
        A = 0
        # Centroid's x
        C_x = 0
        # Centroid's y
        C_y = 0
        for i in range(0, len(vertices) - 1):
            s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
            A = A + s
            C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
            C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
        A = 0.5 * A
        C_x = (1.0 / (6.0 * A)) * C_x
        C_y = (1.0 / (6.0 * A)) * C_y
        return np.array([[C_x, C_y]])
