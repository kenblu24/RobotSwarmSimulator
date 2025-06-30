from itertools import product
from functools import cached_property

import pygame
import numpy as np
from shapely.coordinates import get_coordinates
from shapely.geometry.base import BaseGeometry


class AABB:
    def __init__(self, p):
        p = np.asarray(p, dtype=np.float64)
        if p.ndim == 1:
            p = p.reshape(len(p) // 2, 2)
        self._min = p.min(axis=0)
        self._max = p.max(axis=0)
        self._cs = np.array((self._min, self._max))
        self._axs = self._cs.T  # of shape [X, Y, ...]
        self._size = self._max - self._min
        self.is_intersected = False

    def is_point_inside(self, p):
        return all(p >= self._min) and all(p <= self._max)

    def intersects_bb(self, other) -> bool:
        # TODO: Check if this works in 3D
        cleared = (
            any(other._min > self._max) or
            any(other._max < self._min)
        )
        return not cleared

    @cached_property
    def corners(self):
        # return the vertices of the bounding box.
        # for points/box with N dimensions, 2 ** N vertices (of size N) are returned.
        # i.e. 2D box, a list of 4 tuples of length 2. 3D box: a list of 8 tuples of length 3.
        return np.array([np.array([j[i] for i, j in zip(idx, self._axs)])  # choose the min or max value for each dimension
                for idx in product([0, 1], repeat=len(self._min))])  # each possible combination of min, max

    @cached_property
    def width(self):
        return self._max[0] - self._min[0]

    @cached_property
    def height(self):
        return self._max[1] - self._min[1]

    def toggle_intersection(self):
        self.is_intersected = True

    def __repr__(self):
        return f"AABB({self._min}, {self._max})"

    @classmethod
    def from_wh(cls, p, size):
        p = np.asarray(p, dtype=np.float64)
        size = np.asarray(size, dtype=np.float64)
        return cls([p, p + size])

    def draw(self, screen, offset=((0, 0), 1.0), color=(255, 255, 0)):
        pan, zoom = np.asarray(offset[0]), offset[1]
        if self.is_intersected:
            color = (0, 255, 0)
        if len(self._min) != 2:
            raise NotImplementedError("AABB.draw() only supports 2D")
        pos = self._min * zoom + pan
        size = self._size * zoom
        pygame.draw.rect(screen, color, pygame.Rect(*pos, *size), 1)

    def is_mungible(self, points, tolerance=0.001):
        if isinstance(points, (np.ndarray, list, tuple)):
            points = np.asarray(points, dtype=np.float64)
        elif isinstance(points, BaseGeometry):
            points = np.asarray(get_coordinates(points), dtype=np.float64)
        else:
            return
        if points.shape != (4, 2):
            return False
        points = points.tolist()
        for corner in self.corners:
            for point in points:
                if np.linalg.norm(point - corner) < tolerance:
                    points.remove(point)
                    break
            else:
                return False
        return not points  # points is empty if each point can be assigned to a corner
