import numpy as np
import pygame

from shapely.geometry import Polygon


from .AbstractMetric import AbstractMetric

# typing
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None

class CoverageEfficiency(AbstractMetric):
    def __init__(self, history=100, regularize=True):
        super().__init__(name="Coverage Efficiency", history_size=history)
        self.population = None
        self.regularize = regularize

        self.communications = 0
        self.H_initial = None
        self.H_best = None
        
        self.domain = None
        self.density_function = lambda pts: np.ones(len(pts))
        
    def attach_world(self, world: RectangularWorld):
        super().attach_world(world)
        self.population = world.population
        self.world_size = world.config.size
        self.world_radius = world.config.radius

        self.domain = Polygon([
            (0, 0),
            (self.world_size[0], 0),
            (self.world_size[0], self.world_size[1]),
            (0, self.world_size[1])
        ])

    def calculate(self):
        # Current coverage quality
        H_current = self.compute_H_function()
        
        # Initialize on first call
        if self.H_initial is None:
            self.H_initial = H_current
            self.H_best = H_current
            self.set_value(0)  # No score yet
        
        # Track best H seen (for measuring progress)
        if H_current < self.H_best:
            self.H_best = H_current
        
        coverage_improvement = self.H_initial - H_current
        
        total_updates = self.count_total_communication()
        
        if coverage_improvement <= 0:
            self.set_value(0)  # No improvement yet
        
        elif total_updates == 0:
            self.set_value(0)  # No improvement yet
        else:
            efficiency = coverage_improvement # / total_updates
            # Scale to positive range
            score = max(0, efficiency)
            self.set_value(score)

    def count_total_communication(self):
        total = 0
        for agent in self.population:
            controller = agent.controller
            if hasattr(controller, "communication_count"):
                total += controller.communication_count
        return total

    def compute_H_function(self):
        """
        Compute coverage cost function H
        
        H = ∫_S min_i ||q - pi||² φ(q) dq
        """
        # Get all agent positions
        all_positions = np.array([agent.getPosition() 
                                for agent in self.population])
        
        # Sample domain
        minx, miny, maxx, maxy = self.domain.bounds
        num_samples = 50
        
        x = np.linspace(minx, maxx, num_samples)
        y = np.linspace(miny, maxy, num_samples)
        X, Y = np.meshgrid(x, y)
        points = np.stack([X.ravel(), Y.ravel()], axis=1)  # (N, 2)
        
        # Compute distances: points (N, 2) vs agents (M, 2)
        # Result: (N, M) matrix of distances
        # Using broadcasting: points[:, None, :] - all_positions[None, :, :]
        diff = points[:, np.newaxis, :] - all_positions[np.newaxis, :, :]
        distances = np.linalg.norm(diff, axis=2)  # Shape: (N, M)
        
        # Find minimum distance for each point
        min_distances = np.min(distances, axis=1)  # Shape: (N,)
        min_dist_squared = min_distances ** 2
        
        # Get density at all points
        densities = self.density_function(points)
        
        # Numerical integration
        dx = (maxx - minx) / (num_samples - 1)
        dy = (maxy - miny) / (num_samples - 1)
        dA = dx * dy
        
        # Sum: ∫ min_dist² * φ(q) dq
        H = np.sum(min_dist_squared * densities) * dA
        
        return H