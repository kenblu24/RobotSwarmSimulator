import itertools

import pygame
import numpy as np
from swarmsim.agent.control.AbstractController import AbstractController
from swarmsim.agent.MazeAgent import MazeAgent

import scipy as sp
from scipy import spatial
import sys
from swarmsim.util.pid import PID

# typing
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None


class VoronoiController(AbstractController):
    def __init__(self, agent=None, parent=None):
        self.world = None
        self.population = None
        self.world_size = None
        self.world_radius = None
        self.world_dt = None
        self.allpairs = []
        self.lines = []
        self.centroids = []
        
        # Visualization control
        self.show_mode = "guaranteed"  # Options: "guaranteed", "dual_guaranteed", "all", always shows the regular
        self.uncertainty_radius = 0.2  # Fixed uncertainty radius for visualization
        
        # Store computed boundaries
        self.guaranteed_boundaries = []
        self.dual_guaranteed_boundaries = []
        
        self.tracking_pid = PID(p=0.3, i=0.005, d=0.1)
        super().__init__(agent=agent, parent=parent)

    def attach_world(self, world: RectangularWorld):
        self.world = world
        self.population = world.population
        self.world_size = world.config.size
        self.world_radius = world.config.radius
        self.world_dt = world.dt

    

    def get_actions(self, agent):
        if not self.world or not hasattr(self.world, 'population'):
            return 0, 0
        
        # Compute boundaries for visualization
        self.compute_guaranteed_dual_boundaries()
        
        # Standard Voronoi computation for control
        bounding_box = np.array([0.0, self.world_size[0], 0.0, self.world_size[0]])
        
        agent_positions = []
        position_to_agent_id = {}
        
        for pop_agent in self.population:
            pos = pop_agent.getPosition()
            agent_id = int(pop_agent.name)
            agent_positions.append(pos)
            position_to_agent_id[tuple(pos)] = agent_id
        
        points = np.array(agent_positions)
        self.vor = vor = self.voronoi(points, bounding_box)

        allpairs = []
        centroids = []
        current_agent_centroid = None
        
        if self.vor != []:
            current_agent_id = int(agent.name)
            
            for i, region in enumerate(self.vor.filtered_regions):
                vertices = self.vor.vertices[region + [region[0]], :]
                centroid = self.centroid_region(vertices)
                centroids.append(list(centroid[0, :]))
                
                # Find which agent this region belongs to
                region_center = np.mean(self.vor.vertices[region], axis=0)
                distances = [np.linalg.norm(region_center - point) for point in self.vor.filtered_points]
                closest_point_idx = np.argmin(distances)
                closest_point = tuple(self.vor.filtered_points[closest_point_idx])
                
                if closest_point in position_to_agent_id:
                    region_agent_id = position_to_agent_id[closest_point]
                    if region_agent_id == current_agent_id:
                        current_agent_centroid = np.array(centroid[0, :])
                
                for j in range(1, len(vertices)):
                    allpairs.append([vertices[j-1], vertices[j]])

            self.lines = np.asarray(allpairs)
            self.centroids = np.asarray(centroids)

            if current_agent_centroid is None:
                return 0, 0
                
            goal_position = current_agent_centroid
        else:
            return 0, 0
        
        # Control logic
        v, omega = 0, 0
        dist_to_goal = np.linalg.norm(agent.pos - goal_position)
        radians_to_goal = np.arctan2(goal_position[1] - agent.pos[1], goal_position[0] - agent.pos[0]) - agent.angle
        
        # Normalize angle
        while radians_to_goal > np.pi:
            radians_to_goal -= 2 * np.pi
        while radians_to_goal < -np.pi:
            radians_to_goal += 2 * np.pi
        
        if dist_to_goal > agent.radius:
            v = 0.3 * np.clip(dist_to_goal, 0, 1)
            
            if abs(radians_to_goal) > 0.1:
                omega = np.clip(radians_to_goal, -2, 2)
            else:
                omega = 0

        return v, omega

    def draw(self, screen, offset):
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen, offset)

        # Draw uncertainty circles around agents
        if self.population and (self.show_mode in ["guaranteed", "dual_guaranteed", "all"]):
            for pop_agent in self.population:
                pos = np.array(pop_agent.getPosition())
                pygame.draw.circle(screen, (255, 255, 0), (pos * zoom + pan).astype(int), 
                                 int(self.uncertainty_radius * zoom), width=1)

        # Draw regular Voronoi diagram
        for line in self.lines:
            pygame.draw.line(screen, (128, 128, 128), 
                            (line[0] * zoom + pan).astype(int), 
                            (line[1] * zoom + pan).astype(int), width=2)

        for centroid in self.centroids:
            pygame.draw.circle(screen, (255, 255, 255), 
                                (centroid * zoom + pan).astype(int), radius=4, width=2)

        # Draw guaranteed Voronoi boundaries
        if self.show_mode in ["guaranteed", "all"]:
            for boundary in self.guaranteed_boundaries:
                points = boundary['points']
                if len(points) > 1:
                    # Convert points to screen coordinates
                    screen_points = [(np.array(point) * zoom + pan).astype(int) for point in points]
                    
                    # Draw the hyperbola curve
                    for i in range(len(screen_points) - 1):
                        pygame.draw.line(screen, (0, 255, 0), screen_points[i], screen_points[i + 1], width=3)

        # Draw dual guaranteed Voronoi boundaries  
        if self.show_mode in ["dual_guaranteed", "all"]:
            for boundary in self.dual_guaranteed_boundaries:
                points = boundary['points']
                if len(points) > 1:
                    # Convert points to screen coordinates
                    screen_points = [(np.array(point) * zoom + pan).astype(int) for point in points]
                    
                    # Draw the hyperbola curve
                    for i in range(len(screen_points) - 1):
                        pygame.draw.line(screen, (0, 0, 255), screen_points[i], screen_points[i + 1], width=3)

    @staticmethod
    def in_box(towers, bounding_box):
        if towers.size == 0 or len(towers.shape) != 2 or towers.shape[1] < 2:
            return np.array([], dtype=bool)
        return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                            towers[:, 0] <= bounding_box[1]),
                            np.logical_and(bounding_box[2] <= towers[:, 1],
                                            towers[:, 1] <= bounding_box[3]))

    @staticmethod
    def voronoi(towers, bounding_box):
        if towers.size == 0 or len(towers.shape) != 2 or towers.shape[1] < 2:
            return []
        
        eps = 0.01
        i = VoronoiController.in_box(towers, bounding_box)
        
        if not np.any(i):
            return []
        
        points_center = towers[i, :]
        points_left = np.copy(points_center)
        points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
        points_right = np.copy(points_center)
        points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
        points_down = np.copy(points_center)
        points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
        points_up = np.copy(points_center)
        points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
        points = np.append(points_center,
                        np.append(np.append(points_left,
                                            points_right,
                                            axis=0),
                                    np.append(points_down,
                                            points_up,
                                            axis=0),
                                    axis=0),
                        axis=0)
        
        if len(points) != 0:
            vor = spatial.Voronoi(points)
        else:
            return []
        
        regions = []
        for region in vor.regions:
            flag = True
            for index in region:
                if index == -1:
                    flag = False
                    break
                else:
                    x = vor.vertices[index, 0]
                    y = vor.vertices[index, 1]
                    if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                        bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
                        flag = False
                        break
            if region != [] and flag:
                regions.append(region)
        vor.filtered_points = points_center
        vor.filtered_regions = regions
        return vor
    
    @staticmethod
    def centroid_region(vertices):
        A = 0
        C_x = 0
        C_y = 0
        for i in range(0, len(vertices) - 1):
            s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
            A = A + s
            C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
            C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
        A = 0.5 * A
        if abs(A) < 1e-10:
            return np.array([[np.mean(vertices[:-1, 0]), np.mean(vertices[:-1, 1])]])
        C_x = (1.0 / (6.0 * A)) * C_x
        C_y = (1.0 / (6.0 * A)) * C_y
        return np.array([[C_x, C_y]])
    
    def compute_hyperbola_points(self, p1, r1, p2, r2, guaranteed=True):
        """
        Compute points on the hyperbola boundary between two uncertain circles.
        For guaranteed: ||q - p1|| + r1 = ||q - p2|| - r2
        For dual guaranteed: ||q - p1|| - r1 = ||q - p2|| + r2
        """
        p1, p2 = np.array(p1), np.array(p2)
        
        if guaranteed:
            # Guaranteed boundary: max_dist_to_1 = min_dist_to_2
            # ||q - p1|| + r1 = ||q - p2|| - r2
            # ||q - p1|| - ||q - p2|| = -(r1 + r2)
            a = (r1 + r2) / 2  # semi-major axis
            c = np.linalg.norm(p2 - p1) / 2  # focal distance
        else:
            # Dual guaranteed boundary: min_dist_to_1 = max_dist_to_2  
            # ||q - p1|| - r1 = ||q - p2|| + r2
            # ||q - p1|| - ||q - p2|| = r1 + r2
            a = (r1 + r2) / 2
            c = np.linalg.norm(p2 - p1) / 2
        
        if a >= c:  # No hyperbola exists
            return []
        
        b = np.sqrt(c*c - a*a)  # semi-minor axis
        
        # Center of hyperbola
        center = (p1 + p2) / 2
        
        # Direction vector from p1 to p2
        direction = (p2 - p1) / np.linalg.norm(p2 - p1)
        perpendicular = np.array([-direction[1], direction[0]])
        
        # Generate hyperbola points
        points = []
        t_values = np.linspace(-3, 3, 50)  # Parameter range
        
        for t in t_values:
            # Hyperbola equation in standard form: x²/a² - y²/b² = 1
            x = a * np.cosh(t) if guaranteed else -a * np.cosh(t)
            y = b * np.sinh(t)
            
            # Transform to world coordinates
            point = center + x * direction + y * perpendicular
            
            # Check if point is within world bounds
            if (0 <= point[0] <= self.world_size[0] and 
                0 <= point[1] <= self.world_size[1]):
                points.append(point)
        
        return points

    def compute_guaranteed_dual_boundaries(self):
        """Compute analytical boundaries for guaranteed and dual guaranteed regions"""
        if len(self.population) < 2:
            return
            
        self.guaranteed_boundaries = []
        self.dual_guaranteed_boundaries = []
        
        # Get agent positions
        agents = [(int(agent.name), np.array(agent.getPosition())) for agent in self.population]
        
        # For each pair of agents, compute the boundary
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                id1, pos1 = agents[i]
                id2, pos2 = agents[j]
                
                # Guaranteed boundary (hyperbola closer to agent with larger uncertainty + radius)
                guaranteed_points = self.compute_hyperbola_points(
                    pos1, self.uncertainty_radius, 
                    pos2, self.uncertainty_radius, 
                    guaranteed=True
                )
                
                if guaranteed_points:
                    self.guaranteed_boundaries.append({
                        'agent_pair': (id1, id2),
                        'points': guaranteed_points
                    })
                
                # Dual guaranteed boundary
                dual_guaranteed_points = self.compute_hyperbola_points(
                    pos1, self.uncertainty_radius,
                    pos2, self.uncertainty_radius,
                    guaranteed=False
                )
                
                if dual_guaranteed_points:
                    self.dual_guaranteed_boundaries.append({
                        'agent_pair': (id1, id2),
                        'points': dual_guaranteed_points
                    })

        