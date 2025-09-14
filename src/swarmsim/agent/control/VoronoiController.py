import numpy as np
import pygame
from dataclasses import dataclass
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union, nearest_points


from swarmsim.agent.control.AbstractController import AbstractController
import swarmsim.util.statistics_tools as st
from swarmsim.util.pid import PID

# typing
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None

class VoronoiController(AbstractController):
    def __init__(self, agent=None, parent=None, world: RectangularWorld = None):
        self.population = world.population
        self.world_size = world.config.size
        self.world_radius = world.config.radius
        self.world_dt = world.dt

        self.bounding_box = Polygon([(0, 0), (0, self.world_size[0]), (self.world_size[1], self.world_size[0]), (self.world_size[1], 0)])

        self.agent_memory = None
        self.active_agents = np.array([0], dtype=int) # row indices of agents in agent_memory, starting with self (0)
        self.working_memory = None
        
        self.v_max = 0.3 # maybe change to 0.03
        self.grid_dx = 0.1 # good enough for the 8 by 8 world maybe change to min(0.5 * agent_radius, vmax * world_dt) 
        self.grid_dy = 0.1 # since you dont need a higher resolution than the smallest meaningful change in one timestep
        self.tracking_pid = PID(p=1.0, i=0.01, d=0.0)
        self.once = False

        

        super().__init__(agent=agent, parent=parent)


        

    def get_actions(self, agent):
        if not self.once:
            self.agent_memory = np.array([[np.asarray(agent.getPosition(), dtype=float), 0.0]], dtype=object)

            _, _, refreshed = self.voronoi_refresh(agent, self.agent_memory, bounds_poly=self.bounding_box)
            
            self_row = [np.asarray(agent.getPosition(), dtype=float), 0.0]
            mem_rows = [self_row] + (refreshed.tolist() if refreshed.size else [])
            
            self.agent_memory = np.array(mem_rows, dtype=object)
            self.active_agents = np.arange(self.agent_memory.shape[0], dtype=int)
            self.once = True

        self.agent_memory[0] = [np.asarray(agent.getPosition(), dtype=float), 0.0]
        gVi_poly, dgVi_poly = self.guaranteed_and_dual_gauranteed_cells(self.agent_memory, self.bounding_box, self.grid_dx, self.grid_dy)
        target_pt, bound = self.centroid_and_bound(gVi_poly, dgVi_poly, self.grid_dx, self.grid_dy)
        
        if self.self_trigger(agent.getPosition(), target_pt, bound):
            _, _, refreshed = self.voronoi_refresh(agent, self.agent_memory, bounds_poly=self.bounding_box)

            self_row = [np.asarray(agent.getPosition(), dtype=float), 0.0]
            mem_rows = [self_row] + (refreshed.tolist() if refreshed.size else [])
            self.agent_memory = np.array(mem_rows, dtype=object)
            self.active_agents = np.arange(self.agent_memory.shape[0], dtype=int)
            
            wm = [self.agent_memory[i] for i in self.active_agents]
            self.working_memory = np.array(wm, dtype=object)
            
            gVi_poly, dgVi_poly = self.guaranteed_and_dual_gauranteed_cells(self.working_memory, self.bounding_box, self.grid_dx, self.grid_dy)
            
            target_pt, bound = self.centroid_and_bound(gVi_poly, dgVi_poly, self.grid_dx, self.grid_dy)
        
        v, omega = self.tbb_controller(agent.getPosition(), agent.angle, target_pt, bound, self.v_max, self.world_dt)

        for i in range(1, len(self.agent_memory)):
            self.agent_memory[i][1] += self.v_max * self.world_dt

        
        v, omega = 0.0, 0.0
        return v, omega
    
    
    @staticmethod
    def distance(a, b):
        return np.linalg.norm(a - b)
    
    def closer_to_me_halfspace(self, pi, pj, bounds_poly, eps=1e-12):
        """
        Parameters:
            pi: np.ndarray shape (2,)
            pj: np.ndarray shape (2,)
            bounds_poly: shapely Polygon
            eps: float
        Returns:
            halfplane: shapely Polygon
        """
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
    
    @staticmethod
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
    

    def voronoi_refresh(self, agent, agent_memory, bounds_poly, eps=1e-9): # removed vmax and dt as they are not used
        pi = np.asarray(agent.getPosition(), float)
        candidates = []


        # for pos, radius in agent_memory:
        #     if pos == pi:
        #         continue
        #     d = self.distance(pos, pi) + radius
        #     candidates.append(d)
        
        for row in agent_memory:
            pos = np.asarray(row[0], float); rad = float(row[1])
            if np.allclose(pos, pi): continue
            d = self.distance(pos, pi) + rad
            candidates.append(d)
        R = np.min(candidates) if candidates else 0.0
        
        def provisional_cell(Ri):
            """
            Parameters:
                Ri: float
            Returns:
                W: shapely Polygon
                contacts: np.ndarray shape (n, 2) where n >= 1
            """
            disk_region = Point(pi[0], pi[1]).buffer(Ri)
            W = disk_region.intersection(bounds_poly)
            big_bounds = bounds_poly.buffer(3*max(bounds_poly.bounds[2]-bounds_poly.bounds[0], bounds_poly.bounds[3]-bounds_poly.bounds[1]))
            # in_radius = [[pj, rad] for pj, rad in agent_memory if pj != pi and self.distance(pj, pi) <= Ri + eps]


            # for pj, rad in in_radius:
            #     halfplane_poly = VoronoiController.closer_to_me_halfspace(pi, pj, bounds_poly)
            #     W = W.intersection(halfplane_poly)
            #     if W.is_empty:
            #         break
            in_radius = []
            for row in agent_memory:
                pos = np.asarray(row[0], float); rad = float(row[1])
                if not np.allclose(pos, pi) and self.distance(pos, pi) <= Ri + eps:
                    in_radius.append([pos, rad])
            contacts = np.array(in_radius, dtype=object)

            for row in contacts:
                pj = np.asarray(row[0], float)
                H = self.closer_to_me_halfspace(pi, pj, big_bounds)
                W = W.intersection(H)
                if W.is_empty:
                    break # if the halfplane intersects the boundary, then the cell is empty

            return W, contacts

        W, contacts = provisional_cell(R)

        while True:
            rad_needed = 0.0 if W.is_empty else 2.0 * self.max_rad_from_me(W, pi)
            if R + eps >= rad_needed:
                break
            R *= 2.0
            W, contacts = provisional_cell(R)

        voronoi_poly = W
        
        neighbors = []

        # TODO: upgrade to test if the bisector line with pj intersects voronoi_poly with positive length -> neighbor.
        W_full = Point(pi[0], pi[1]).buffer(R).intersection(bounds_poly)
        for j in contacts:
            H = self.closer_to_me_halfspace(pi, j[0], bounds_poly)
            W_new = W_full.intersection(H)
            # If intersecting with H shrinks W_full and the boundary intersects S, j is a neighbor.
            if not W_new.equals(W_full) and not W_new.is_empty:
                neighbors.append(j) # might be a problem later on with how index of agents are found and stored
            W_full = W_new  # progressively build as in the loop
        
        neighbors = np.array(neighbors, dtype=object) if neighbors else np.empty((0,), dtype=object)
        refreshed = np.array([[pj, 0.0] for pj, rad in neighbors]) # this is the agents that will be used by working_memory (doesnt include self)
        
        return voronoi_poly, neighbors, refreshed
    
    def guaranteed_and_dual_gauranteed_cells(self, working_memory, bounds_poly, grid_dx, grid_dy):
        """
        Parameters:
            working_memory: np.ndarray shape (n, 2) where n >= 1
            bounds_poly: shapely Polygon
            grid_dx: float
            grid_dy: float
        Returns:
            gVi_poly: shapely Polygon
            dgVi_poly: shapely Polygon
        """
        p_self = np.asarray(working_memory[0][0], float)
        r_self = float(working_memory[0][1]) # should be zero

        points_g = []
        points_dg = []
        for x in np.arange(bounds_poly.bounds[0], bounds_poly.bounds[2], grid_dx):
            for y in np.arange(bounds_poly.bounds[1], bounds_poly.bounds[3], grid_dy):
                q = [x, y]
                if not bounds_poly.contains(Point(q)):
                    continue
                
                left_g = self.distance(q, p_self) + r_self
                left_dg = max(0.0, self.distance(q, p_self) - r_self)
                right_g = np.inf
                right_dg = np.inf
                for row in working_memory[1:]:
                    pj = np.asarray(row[0], float)
                    rj = float(row[1])
                    right_g = min(right_g, max(0.0, self.distance(q, pj) - rj))
                    right_dg = min(right_dg, self.distance(q, pj) + rj)

                if left_g <= right_g:
                    points_g.append(q)
                if left_dg <= right_dg:
                    points_dg.append(q)
            
        # take all accepted grid points, make tiny cells around them, and union them to approximate the region
        def union_of_squares(points, grid_dx, grid_dy):
            if not points:
                return Polygon()
            
            half_dx, half_dy = grid_dx / 2.0, grid_dy / 2.0
            cells = []
            for (x, y) in points:
                cell = Polygon([
                    (x - half_dx, y - half_dy),
                    (x - half_dx, y + half_dy),
                    (x + half_dx, y + half_dy),
                    (x + half_dx, y - half_dy)
                ])
                cells.append(cell)
                if not cells:
                    return Polygon()
            return unary_union(cells)

        gVi_poly = union_of_squares(points_g, grid_dx, grid_dy).intersection(bounds_poly)
        dgVi_poly = union_of_squares(points_dg, grid_dx, grid_dy).intersection(bounds_poly)

        return gVi_poly, dgVi_poly
    
    
    def centroid_and_bound(self, gVi_poly, dgVi_poly, grid_dx=None, grid_dy=None):
        """
        Returns:
            target_pt: shapely Point (prefer interior if possible)
            bound:     float
        """
        # Empty gVi => force refresh upstream
        if gVi_poly.is_empty:
            return None, np.inf
        
        # prefer geometric centroid else snap to an interior point
        target_pt = gVi_poly.centroid 
        if not target_pt.within(gVi_poly):
            try:
                target_pt = nearest_points(gVi_poly, target_pt)[0]
            except Exception:
                target_pt = gVi_poly.representative_point()

        area_g = gVi_poly.area
        area_dg = 0.0 if (dgVi_poly is None or dgVi_poly.is_empty) else dgVi_poly.area

        # threshold for degenerate dgVi
        tiny = (grid_dx * grid_dy) if (grid_dx and grid_dy) else 1e-12
        if (area_dg <= max(tiny, 1e-12)):
            return target_pt, np.inf

        # Aprrox circumradius of dgVi about its centroid
        C = dgVi_poly.centroid
        Cxy = np.array([C.x, C.y], dtype=float)
        
        def max_rad_from_component(poly):
            rmax = 0.0
            for v in poly.exterior.coords:
                rmax = max(rmax, self.distance(np.asarray(v, float), Cxy))
            return rmax
        
        cr = 0.0
        if dgVi_poly.geom_type == 'Polygon':
            cr = max_rad_from_component(dgVi_poly)
        elif dgVi_poly.geom_type == 'MultiPolygon':
            for P in dgVi_poly.geoms:
                cr = max(cr, max_rad_from_component(P))
        else:
            # Fallback for unexpected type
            return target_pt, np.inf

        # Conservative bound
        bound = 2.0 * cr * max(0.0, (1.0 - area_g / area_dg))
        return target_pt, bound
    
    def self_trigger(self, pi, target_pt, bound, eps=1e-6, hysteresis=1.05):
        """
        Decide whether to refresh now (Algorithm 4 step 3).
        - pi: np.ndarray shape (2,) current position
        - target_pt: shapely Point (or None)
        - bound: float (bnd(gVi, dgVi))
        - eps: small floor for distance comparison (units of world coords)
        - hysteresis: ≥1.0; set slightly >1 (e.g., 1.05) to avoid chatter
        """
        if (target_pt is None) or (not np.isfinite(bound)):
            return True
        
        q = np.array([target_pt.x, target_pt.y], dtype=float)
        pi = np.asarray(pi, dtype=float)
        d = self.distance(q, pi)

        threshold = max(d, eps) * float(hysteresis)
        return bound >= threshold


    def tbb_controller(self, pi, heading, target_pt, bound, vmax, dt):
        """
        Parameters:
            pi: np.ndarray shape (2,)
            heading: float
            target_pt: shapely Point
            bound: float
            vmax: float
            dt: float
        Returns:
            v: float
            omega: float
        """
        if target_pt is None:
            return 0.0, 0.0
        
        q = np.array([target_pt.x, target_pt.y], dtype=float)
        pi = np.asarray(pi, dtype=float)
        dir_vec = q - pi
        d = np.linalg.norm(dir_vec)
        if d <= 1e-12:
            return 0.0, 0.0
        
        step_cap = vmax * dt
        step_goal = max(0.0, d - bound)
        s = min(step_cap, step_goal)

        def wrap_angle(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi

        desired_heading = np.arctan2(dir_vec[1], dir_vec[0])
        heading_err = wrap_angle(desired_heading - heading)
        omega = self.tracking_pid(np.clip(heading_err, -2, 2))

        v = s / dt

        return v, omega
    
