import pygame
import numpy as np
import shapely.geometry as sg
from shapely.ops import nearest_points


class Collider:
    infinitesmimal = 0.0001
    shake_amount = 0.001

    def __init__(self, x, y, r):
        self.color = (0, 255, 0)
        self.collision_flag = False

    def update(self):
        pass

    def flag_collision(self):
        self.collision_flag = True

    def correction(self, other, rng=None):
        pass

    def draw(self, screen, color=(0, 255, 0)):
        self.color = (255, 0, 0) if self.collision_flag else color


class CircularCollider(Collider):
    def __init__(self, x, y, r):
        self.x: float
        self.y: float
        self.r: float
        self.c: np.ndarray
        self.update(x, y, r)

    def update(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
        self.c = np.array([x, y])

    def correction(self, other, rng=None):
        if rng is None:
            rng = np.random.default_rng(0)

        if isinstance(other, CircularCollider):
            return self.vs_circle(other, rng)
        elif isinstance(other, PolyCollider):
            return self.vs_convex(other, rng)
        return np.empty(2) * np.nan

    def vs_circle(self, other, rng):
        shape = self.c.shape
        correction_vector = np.zeros(shape)
        dist_between_radii = np.linalg.norm(other.c - self.c)
        dist_difference = (self.r + other.r) - dist_between_radii
        if dist_difference < 0:
            return np.empty(shape) * np.nan
        elif dist_between_radii < self.infinitesmimal:
            amount = np.ones(shape) * self.shake_amount
            correction_vector += rng.uniform(-amount, amount)
        correction_vector += ((other.c - self.c) / (dist_between_radii + 0.001)) * (dist_difference + self.infinitesmimal)
        return -correction_vector

    def vs_convex(self, other, rng):
        shape = self.c.shape
        correction_vector = np.zeros(shape)
        self2 = sg.Point(self.c).buffer(self.r)
        other2 = sg.Polygon(other.points)
        distance = self2.distance(other2)
        if distance > self.infinitesmimal:
            return np.empty(shape) * np.nan
        a, b = nearest_points(self2, other2)
        return np.empty(shape) * np.nan  # TODO: this is a placeholder line.
        a.wkt

    def draw(self, screen, color=(0, 255, 0)):
        super().draw(screen, color)
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.r, 3)


class PolyCollider(Collider):
    def __init__(self, points):
        self.points: np.ndarray
        self.update(points)

    def update(self, points):
        # self.points = np.asarray(points, dtype=np.float64)[::-1]  # CW -> CCW winding
        # self.points = np.flip(np.asarray(points, dtype=np.float64))
        self.points = np.asarray(points, dtype=np.float64)

    def correction(self, other, rng=None):
        if rng is None:
            rng = np.random.default_rng(0)

        pass  # TODO: implement this

        return np.empty(2) * np.nan
