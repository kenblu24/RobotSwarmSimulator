import itertools

import pygame
import numpy as np
from .AbstractMetric import AbstractMetric

import scipy as sp
from scipy import spatial
import sys

# typing
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None


class VoronoiRelaxation(AbstractMetric):
    __badvars__ = AbstractMetric.__badvars__ + ['population']  # references to population may cause pickling errors

    def __init__(self, history=100, regularize=True):
        super().__init__(name="Voronoi Dispersal", history_size=history)
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
        

        bounding_box = np.array([0.0, self.world_size[0], 0.0, self.world_size[0]])  # [x_min, x_max, y_min, y_max]
        
        points = np.array([agent.getPosition() for agent in self.population])
        # self.v = v = spatial.Voronoi(points)
        # allpairs = set()
        # for vertex in v.ridge_vertices:
        #     vertex = np.asarray(vertex)
        #     if np.all(vertex >= 0):
        #         allpairs.add(tuple(sorted(vertex)))
        
        # self.allpairs = np.asarray(list(allpairs), dtype=np.int32)
        
        self.vor = vor = self.voronoi(points, bounding_box)
      
        allpairs = []
        centroids = []
        if self.vor != []:
            for region in self.vor.filtered_regions:
                vertices = self.vor.vertices[region + [region[0]], :]
                # centroid_vertices = self.vor.vertices[]
                centroid = self.centroid_region(vertices)
                centroids.append(list(centroid[0, :]))
                
                for i in range(1, len(vertices)):
                    allpairs.append([vertices[i-1], vertices[i]])

        self.lines = np.asarray(allpairs)
        self.centroids = np.asarray(centroids)


        # distances = np.array([d.plane_distance(p) for p in points])
        min_distances = []
        # assert len(self.centroids) == len(points)
        for centroid in self.centroids:
            distances = []
            for point in points:
                distances.append(np.linalg.norm(centroid - point))
            
            min_distances.append(min(distances))

            # for a, b in zip(centroid, points):
            #     distances = np.asarray([np.linalg.norm(a - b)])
            
            #     min_distances.append(distances.min())
        
        
        # distances = np.array([np.linalg.norm(a - b) for centroid in self.centroids for a, b in zip(centroid, points)])
        # var = distances.var()
        # mean = distances.mean()
        # if len(self.centroids) == len(self.population):
        #     self.set_value(-sum(min_distances))
        # else:
        #     self.set_value(-1000)
        if np.all(VoronoiRelaxation.in_box(points, bounding_box)):
            self.set_value(-sum(min_distances))
        else:
            self.set_value(-1000)
        

    def draw(self, screen, offset):
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen, offset)

        for line in self.lines:
            pygame.draw.line(screen, (128, 128, 128), *line * zoom + pan, width=1)

        for centroid in self.centroids:
            pygame.draw.circle(screen, (128, 128, 128), centroid * zoom + pan, radius=5, width=1)

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
        i = VoronoiRelaxation.in_box(towers, bounding_box)
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