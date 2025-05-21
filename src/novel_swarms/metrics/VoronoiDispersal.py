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
        
        points = np.array([agent.getPosition() for agent in self.population])
        self.v = v = spatial.Voronoi(points)
        allpairs = set()
        for vertex in v.ridge_vertices:
            vertex = np.asarray(vertex)
            if np.all(vertex >= 0):
                allpairs.add(tuple(sorted(vertex)))
        
        self.allpairs = np.asarray(list(allpairs), dtype=np.int32)
        self.lines = self.v.vertices[self.allpairs]
        self.lines = np.clip(self.lines, 0, self.world_size)
        
        
        # self.lines = np.asarray([[self.v.points[ridge_point[0]], self.v.points[ridge_point[1]]] for ridge_point in self.v.ridge_points], dtype=np.int32)

        # distances = np.array([d.plane_distance(p) for p in points])
        distances = np.array([np.linalg.norm(a - b) for a, b in self.lines])
        var = distances.var()
        mean = distances.mean()
        # bbox_size = (d.max_bound - d.min_bound)
        # bbox_ratio = min(bbox_size) / max(bbox_size)
        # self.set_value(bbox_area / 10 - ((1 + var * 10) * mean))
        # dispersal = bbox_size.prod() * bbox_ratio / (1 + var * 10)
        # self.set_value(dispersal if dispersal is not None else 0)

    def draw(self, screen, offset):
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen, offset)

        for line in self.lines:
            pygame.draw.line(screen, (128, 128, 128), *line * zoom + pan, width=1)

# This is a modified version of the code from here:
# https://stackoverflow.com/questions/28665491/getting-a-bounded-polygon-coordinates-from-voronoi-cells

    

# eps = sys.float_info.epsilon

# n_towers = 100
# towers = np.random.rand(n_towers, 2)
# bounding_box = np.array([0., 1., 0., 1.]) # [x_min, x_max, y_min, y_max]

# def in_box(towers, bounding_box):
#     return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
#                                         towers[:, 0] <= bounding_box[1]),
#                         np.logical_and(bounding_box[2] <= towers[:, 1],
#                                         towers[:, 1] <= bounding_box[3]))


# def voronoi(towers, bounding_box):
#     # Select towers inside the bounding box
#     i = in_box(towers, bounding_box)
#     # Mirror points
#     points_center = towers[i, :]
#     points_left = np.copy(points_center)
#     points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
#     points_right = np.copy(points_center)
#     points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
#     points_down = np.copy(points_center)
#     points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
#     points_up = np.copy(points_center)
#     points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
#     points = np.append(points_center,
#                     np.append(np.append(points_left,
#                                         points_right,
#                                         axis=0),
#                                 np.append(points_down,
#                                         points_up,
#                                         axis=0),
#                                 axis=0),
#                     axis=0)
#     # Compute Voronoi
#     vor = spatial.Voronoi(points)
#     # Filter regions
#     regions = []
#     for region in vor.regions:
#         flag = True
#         for index in region:
#             if index == -1:
#                 flag = False
#                 break
#             else:
#                 x = vor.vertices[index, 0]
#                 y = vor.vertices[index, 1]
#                 if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
#                     bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
#                     flag = False
#                     break
#         if region != [] and flag:
#             regions.append(region)
#     vor.filtered_points = points_center
#     vor.filtered_regions = regions
#     return vor

# def centroid_region(vertices):
#     # Polygon's signed area
#     A = 0
#     # Centroid's x
#     C_x = 0
#     # Centroid's y
#     C_y = 0
#     for i in range(0, len(vertices) - 1):
#         s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
#         A = A + s
#         C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
#         C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
#     A = 0.5 * A
#     C_x = (1.0 / (6.0 * A)) * C_x
#     C_y = (1.0 / (6.0 * A)) * C_y
#     return np.array([[C_x, C_y]])

# vor = voronoi(towers, bounding_box)
