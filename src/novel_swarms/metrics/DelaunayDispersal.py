import itertools

import pygame
import numpy as np
from scipy.spatial import Delaunay
from .AbstractMetric import AbstractMetric

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None


class Dispersal(AbstractMetric):
    __badvars__ = AbstractMetric.__badvars__ + ['population']  # references to population may cause pickling errors

    def __init__(self, history=100, regularize=True):
        super().__init__(name="Delaunay Dispersal", history_size=history)
        self.population = None
        self.regularize = regularize
        self.allpairs = []
        self.lines = []

    def attach_world(self, world: RectangularWorld):
        super().attach_world(world)
        self.population = world.population
        self.world_radius = world.config.radius

    def calculate(self):
        points = np.array([agent.getPosition() for agent in self.population])
        self.d = d = Delaunay(points)

        allpairs = set()
        for tri in self.d.simplices:
            pairs = itertools.combinations(tri, 2)
            pairs = [tuple(sorted(pair)) for pair in pairs]
            allpairs.update(pairs)
        self.allpairs = np.asarray(list(allpairs), dtype=np.int32)
        self.lines = self.d.points[self.allpairs]

        # distances = np.array([d.plane_distance(p) for p in points])
        distances = np.array([np.linalg.norm(a - b) for a, b in self.lines])
        var = distances.var()
        mean = distances.mean()
        bbox_size = (d.max_bound - d.min_bound)
        bbox_ratio = min(bbox_size) / max(bbox_size)
        # self.set_value(bbox_area / 10 - ((1 + var * 10) * mean))
        dispersal = bbox_size.prod() * bbox_ratio / (1 + var * 10)
        self.set_value(dispersal if dispersal is not None else 0)

    def draw(self, screen, offset):
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen, offset)

        for line in self.lines:
            pygame.draw.line(screen, (128, 128, 128), *line * zoom + pan, width=1)
