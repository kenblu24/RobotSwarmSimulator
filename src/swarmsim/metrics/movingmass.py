import pygame
import numpy as np

from ..util.statistics_tools import Remap
from .aggregation import Aggregation


class MovingMass(Aggregation):
    def __init__(self, linear: tuple[float, float] = (1, 1), history_size: int = 450, **kwargs):
        super().__init__(history_size=history_size, **kwargs)

        self.centroids = []
        self.linear = linear

    @property
    def positions(self):
        return np.asarray([agent.getPosition() for agent in self.parent.population])

    @property
    def min_travel(self):
        # TODO: This is meant to be an agent's radius
        return 0.1

    def center_of_mass(self):
        # NOTE(mabay): Copied this over from 'RadialVarianceMetric'
        return self.positions.mean(axis=0)

    def _calculate(self):
        agg_wt, disp_wt = self.linear
        centroid = self.center_of_mass()

        return agg_wt * super()._calculate() + disp_wt * self.f(centroid)

    # TODO: Give this method a better name
    def f(self, curr_centroid) -> float:
        # Update centroids list
        self.centroids.append(curr_centroid)

        T = self.history_size
        prev_centroid = self.centroids[-T if len(self.centroids) >= T else 0]
        dist = np.linalg.norm(curr_centroid - prev_centroid)

        score = dist
        if dist < self.min_travel:
            score = dist / self.min_travel
        return score

    def draw(self, screen, zoom=1.0):
        if len(self.centroids) == 0:
            return

        pan, zoom = self.world.pos, self.world.zoom
        first_centroid, curr_centroid = np.array(self.centroids[0]), np.array(self.centroids[-1])
        pygame.draw.circle(screen, "#ffff00", first_centroid * zoom + pan, 0.05 * zoom, width=2)
        pygame.draw.circle(screen, "#ffff00", curr_centroid * zoom + pan, 0.05 * zoom, width=2)

        