import numpy as np
import pygame
from dataclasses import dataclass
from shapely.geometry import Point, Polygon, box

from swarmsim.agent.control.AbstractController import AbstractController
import swarmsim.util.statistics_tools as st

# typing
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None

class VoronoiController(AbstractController):
    def __init__(self, agent=None, parent=None):
        self.bounding_box = Polygon([(0, 0), (0, self.world_size[0]), (self.world_size[1], self.world_size[0]), (self.world_size[1], 0)])
        self.agent_memory = None
        self.active_agents = None
        self.working_memory = None
        self.v_max = 0.3
        self.once = False

        super().__init__(agent=agent, parent=parent)

    def attach_world(self, world: RectangularWorld):
        super().attach_world(world)
        self.population = world.population
        self.world_size = world.config.size
        self.world_radius = world.config.radius
        self.world_dt = world.dt

    def get_actions(self, agent):
        if not self.once:
            self.active_agents = np.array([[agent.getPosition(), 0.0]])
            voronoi_poly, neighbors, refreshed = self.voronoi_refresh(agent, self.agent_memory, bounds_poly=self.bounding_box)
            np.append(self.active_agents, neighbors)
            self.agent_memory = np.array(refreshed)
            self.once = True

        working_memory = self.
        #TODO: once done with initial memory stuff, figure out how to update position of self after movement from inside the controller
        
        points = np.array([agent.getPosition() for agent in self.population])
        points = np.delete(points, agent.getPosition())
        
        
        return v, w
    
    
    @staticmethod
    def distance(a, b):
        return np.linalg.norm(a - b)
    
    def closer_to_me_halfspace(self, pi, pj, bounds_poly, eps=1e-12):
        a = pj - pi
        nrm = np.linalg.norm(a)
        if nrm < eps:
            return bounds_poly

        a_hat = a / nrm
        c = 0.5 * (np.dot(pj, pj) - np.dot(pi, pi))

        # Find a point q0 on the line a·q = c by shifting the bounds centroid along a_hat
        center = np.asarray(bounds_poly.centroid.coords[0])
        # because a_hat is unit, denominator is 1:
        q0 = center + (c - np.dot(a_hat, center)) * a_hat

        # Build a long segment along the line direction (perpendicular to a_hat)
        t = np.array([-a_hat[1], a_hat[0]])  # unit vector along the line
        minx, miny, maxx, maxy = bounds_poly.bounds
        diag = np.hypot(maxx - minx, maxy - miny)

        L = 2.0 * diag + 1.0  # length along the line
        H = 2.0 * diag + 1.0  # depth away from the line

        p1 = q0 - L * t
        p2 = q0 + L * t

        # Half-plane for a·q <= c is the side in the -a direction (since a·(q - q0) <= 0)
        halfplane = Polygon([tuple(p1),
                            tuple(p2),
                            tuple(p2 - H * a_hat),
                            tuple(p1 - H * a_hat)])
        
        return bounds_poly.intersection(halfplane)

    def max_rad_from_me(poly, pi, samples=200):
        # Approximate max_{q in poly} ||q - pi|| by sampling boundary
        if poly.is_empty:
            return 0.0
        boundary = poly.boundary
        # Sample equidistant points along boundary length
        L = boundary.length
        if L == 0:
            return np.linalg.norm(np.array(boundary.coords[0]) - pi)
        ts = np.linspace(0, L, samples, endpoint=False)
        dmax = 0.0
        for t in ts:
            q = np.array(boundary.interpolate(t).coords[0])
            dmax = max(dmax, np.linalg.norm(q - pi))
        return dmax
    

    def voronoi_refresh(self, agent, agent_memory, vmax, dt, bounds_poly, eps=1e-9):
        pi = agent.getPosition()
        candidates = np.array([])

        for pos, radius in agent_memory:
            if pos == pi:
                continue
            d = self.distance(pos, pi) + radius
            np.append(candidates, d)
        R = np.min(candidates)

        def provisional_cell(Ri):
            disk_region = Point(pi[0], pi[1]).buffer(Ri)
            W = disk_region.intersection(bounds_poly)
            in_radius = [[pj, rad] for pj, rad in agent_memory if pj != pi and self.distance(pj, pi) <= Ri + eps]


            for pj, rad in in_radius:
                halfplane_poly = VoronoiController.closer_to_me_halfspace(pi, pj, bounds_poly)
                W = W.intersection(halfplane_poly)
                if W.is_empty:
                    break
            return W, in_radius

        W, contacts = provisional_cell(R)

        while True:
            rad_needed = 2.0 * self.max_rad_from_me(W, pi)
            if R + eps >= rad_needed:
                break
            R *= 2.0
            W, contacts = provisional_cell(R)

        voronoi_poly = W
        
        neighbors = np.array([])

        W_full = Point(pi[0], pi[1]).buffer(R).intersection(bounds_poly)
        for j in contacts:
            H = self.closer_to_me_halfspace(pi, j[0], bounds_poly)
            W_new = W_full.intersection(H)
            # If intersecting with H shrinks W_full and the boundary intersects S, j is a neighbor.
            if not W_new.equals(W_full) and not W_new.is_empty:
                np.append(neighbors, j) # might be a problem later on with how index of agents are found and stored
            W_full = W_new  # progressively build as in the loop

        refreshed = np.array([[pj, 0.0] for pj, rad in neighbors])

        return voronoi_poly, neighbors, refreshed