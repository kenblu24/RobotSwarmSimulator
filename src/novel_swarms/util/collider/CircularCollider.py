import pygame
import numpy as np

class CircularCollider:
    infinitesmimal = 0.0001
    shake_amount = 0.001

    def __init__(self, x, y, r, rng=np.random.default_rng(0)):
        self.x = x
        self.y = y
        self.r = r
        self.v = np.array([x, y])
        self.rng = rng
        self.collision_flag = False

    def update(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
        self.v = np.array([x, y])

    def flag_collision(self):
        self.collision_flag = True

    def correction(self, other):
        shape = self.v.shape
        correction_vector = np.zeros(shape)
        dist_between_radii = self.dist(other)
        dist_difference = (self.r + other.r) - dist_between_radii
        if dist_difference < 0:
            return np.empty(shape) * np.nan
        elif dist_between_radii < self.infinitesmimal:
            amount = np.ones(shape) * self.shake_amount
            correction_vector += self.rng.uniform(-amount, amount)
        correction_vector += ((other.v - self.v) / (dist_between_radii + 0.001)) * (dist_difference + self.infinitesmimal)
        return -correction_vector

    def dist(self, other):
        return np.linalg.norm(other.v - self.v)

    def draw(self, screen, color=(0, 255, 0)):
        if self.collision_flag:
            color = (255, 0, 0)
        pygame.draw.circle(screen, color, (self.x, self.y), self.r, 3)
