import pygame
import numpy as np

from .aggregation import Aggregation


class MovingMass(Aggregation):
    def __init__(self, weights: tuple[float, float] = (1, 1), *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.centroids = []
        self.weights = weights
        self._invert_multiplier = 1

    @property
    def positions(self):
        return np.asarray([agent.getPosition() for agent in self.parent.population])

    @property
    def agg_value(self):
        return 1. - super()._calculate()

    def center_of_mass(self):
        # NOTE(mabay): Copied this over from 'RadialVarianceMetric'
        return self.positions.mean(axis=0)

    def _calculate(self):
        agg_wt, disp_wt = self.weights
        centroid = self.center_of_mass()
        if len(self.centroids) == 0:
            # NOTE(mabay): Insert centroid twice, at first, because I need two values to start
            # calculating
            self.centroids.append(centroid)
            self.centroids.append(centroid)
        else:
            self.centroids.append(centroid)

        disp_value = np.linalg.norm(self.centroids[-1] - self.centroids[-2])
        return agg_wt * self.agg_value + disp_wt * disp_value

    def draw(self, screen, zoom=1.0):
        pan, zoom = self.world.pos, self.world.zoom
        last_centroid, last_last_centroid = self.centroids[-1], self.centroids[-2]
        pygame.draw.line(screen, "#00ffff", last_centroid * zoom + pan, last_last_centroid * zoom + pan, width=2)
        pygame.draw.circle(screen, "#ffff00", last_centroid * zoom + pan, 0.05 * zoom)
        pygame.draw.circle(screen, "#ffff00", last_last_centroid * zoom + pan, 0.05 * zoom)

        