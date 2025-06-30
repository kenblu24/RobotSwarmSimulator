import pygame
import numpy as np
import math
from .AbstractSensor import AbstractSensor
from typing import List
from ..world.goals.Goal import CylinderGoal

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..world.World import World
else:
    World = None

import warnings
import quads


def vectorize(angle) -> None:
    return np.array((np.cos(angle), np.sin(angle)))

def turn(p1, p2):
    return p1[0] * p2[1] - p2[0] * p1[1]

def project(a, b):
    return b * (np.dot(a, b) / np.dot(b, b))

def lineCircleIntersect(line, center, radius):
    clDiffVec = center - project(center, line)
    return np.dot(clDiffVec, clDiffVec) <= radius**2

class BinaryFOVSensor(AbstractSensor):
    config_vars = AbstractSensor.config_vars + [
        'theta', 'distance', 'bias', 'false_positive', 'false_negative',
        'walls', 'wall_sensing_range', 'time_step_between_sensing', 'invert',
        'store_history', 'detect_goal_with_added_state', 'show'
    ]

    def __init__(
        self,
        agent=None,
        parent=None,
        theta=10.0,
        distance=100.0,
        bias=0.0,
        false_positive=0.0,
        false_negative=0.0,
        walls=None,
        goal_sensing_range=10.0,
        wall_sensing_range=10.0,
        time_step_between_sensing=1,
        invert=False,
        store_history=False,
        detect_goal_with_added_state=False,
        show=True,
        seed=None,
        **kwargs
    ):
        super().__init__(agent=agent, parent=parent)
        self.angle = 0.0
        self.theta = theta
        self.bias = bias
        self.fp = false_positive
        self.fn = false_negative
        self.walls = walls
        self.wall_sensing_range = wall_sensing_range
        self.time_step_between_sensing = time_step_between_sensing
        self.time_since_last_sensing = 0
        self.history = []
        self.store_history = store_history
        self.use_goal_state = detect_goal_with_added_state
        self.goal_sensing_range = goal_sensing_range
        self.show = show
        self.invert = invert
        self.goal_detected = False
        self.detection_id = 0

        self.detectOnlyOrigins = False

        NOTFOUND = object()
        if (degrees := kwargs.pop('degrees', NOTFOUND)) is not NOTFOUND:
            warnings.warn("The 'degrees' kwarg is deprecated.", FutureWarning, stacklevel=1)
            if degrees:
                self.theta = np.radians(self.theta)

        self.r = distance

        self.seed = seed
        if self.seed is not None:
            np.random.seed(self.seed)

    def checkForLOSCollisions(self, world: World) -> None:
        # Mathematics obtained from Sundaram Ramaswamy
        # https://legends2k.github.io/2d-fov/design.html
        # See section 3.1.1.2
        self.time_since_last_sensing += 1
        if self.time_since_last_sensing % self.time_step_between_sensing != 0:
            # Our sensing rate occurs less frequently than our dt physics update, so we need to
            #   only check for LOS collisions every n timesteps.
            return

        self.time_since_last_sensing = 0
        sensor_origin = self.agent.getPosition()

        # First, bag all agents that lie within radius r of the parent
        bag = []
        self.getAARectContainingCone()
        quad: quads.QuadTree = world.quad
        quadpoints = [point.data for point in quad.within_bb(quads.BoundingBox(*self.getAARectContainingCone()))]
        for agent in quadpoints:
            if self.getDistance(sensor_origin, agent.getPosition()) < self.r:
                bag.append(agent)

        e_left, e_right = self.getSectorVectors()

        # Detect Outer Walls
        # TODO: Rewrite all this to use WorldObjects
        # see https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
        consideration_set = []
        if self.walls is not None:
            # Get e_left, e_right line_segments
            l = [sensor_origin, sensor_origin + (e_left[:2] * self.wall_sensing_range)]  # noqa: E741
            r = [sensor_origin, sensor_origin + (e_right[:2] * self.wall_sensing_range)]
            wall_top = [self.walls[0], [self.walls[1][0], self.walls[0][1]]]
            wall_right = [[self.walls[1][0], self.walls[0][1]], self.walls[1]]
            wall_bottom = [self.walls[1], [self.walls[0][0], self.walls[1][1]]]
            wall_left = [[self.walls[0][0], self.walls[1][1]], self.walls[0]]

            # Brute Check for intersection with each wall
            for wall in [wall_top, wall_right, wall_bottom, wall_left]:
                for line in [l, r]:
                    if self.lines_segments_intersect(line, wall):
                        d_to_inter = np.linalg.norm(
                            np.array(self.line_seg_int_point(line, wall)) - np.array(sensor_origin))
                        consideration_set.append((d_to_inter, None))

        # Detect for World Objects
        # TODO: use this again
        # for world_obj in world.objects:
        #     if not world_obj.detectable:
        #         continue
        #     l = [sensor_origin, sensor_origin + (e_left[:2] * self.wall_sensing_range)]
        #     r = [sensor_origin, sensor_origin + (e_right[:2] * self.wall_sensing_range)]
        #     for segment in world_obj.get_sensing_segments():
        #         if self.lines_segments_intersect(segment, l):
        #             d_to_inter = np.linalg.norm(np.array(self.line_seg_int_point(segment, l)) - np.array(sensor_origin))
        #             consideration_set.append((d_to_inter, None))

        #         if self.lines_segments_intersect(segment, r):
        #             d_to_inter = np.linalg.norm(np.array(self.line_seg_int_point(segment, r)) - np.array(sensor_origin))
        #             consideration_set.append((d_to_inter, None))
        # Detect Other Agents
        l180 = self.theta * 2 < np.pi

        for agent in bag:
            if agent is self.agent:
                continue
            
            u = agent.getPosition() - sensor_origin
            leftTurn = turn(u, e_left)
            rightTurn = turn(u, e_right)
            
            # if fov < 180 use between minor arc, otherwise use not between minor arc
            if rightTurn <= 0 and 0 <= leftTurn if l180 else not (leftTurn < 0 and 0 < rightTurn):
                self.determineState(True, agent, world)
                return
            elif not self.detectOnlyOrigins:
                # circle whisker intercept correction
                if (0 < np.dot(u, e_left[:2]) and lineCircleIntersect(e_left[:2], u, agent.radius)) or (0 < np.dot(u, e_right[:2]) and lineCircleIntersect(e_right[:2], u, agent.radius)):
                    self.determineState(True, agent, world)
                    return

            # d = self.circle_interesect_sensing_cone(u, self.agent.radius)
            # if d is not None:
            #     consideration_set.append((d, agent))
            #     self.determineState(True, agent, world)
            #     return

        # if not consideration_set:
        self.determineState(False, None, world)
        return

        # consideration_set.sort()
        # print(consideration_set)
        _score, val = consideration_set.pop(0)
        self.determineState(True, val, world)

    def getAARectContainingCone(self):
        angle: float = self.agent.angle + self.bias
        span: float = self.theta
        radius: float = self.r
        position: list[float] = self.agent.pos.tolist()

        over180 = np.pi <= span*2

        center = vectorize(angle)
        leftBorder = vectorize(angle + span) * radius
        rightBorder = vectorize(angle - span) * radius
        xaxis = (1, 0)
        yaxis = (0, 1)

        xt = turn(xaxis, center)
        xlt = turn(xaxis, leftBorder)
        xrt = turn(xaxis, rightBorder)
        fovOverXAxis = np.sign(xlt) != np.sign(xrt)
        
        yt = turn(yaxis, center)
        ylt = turn(yaxis, leftBorder)
        yrt = turn(yaxis, rightBorder)
        fovOverYAxis = np.sign(ylt) != np.sign(yrt)
        
        xvals = [0, leftBorder[0], rightBorder[0]]
        yvals = [0, leftBorder[1], rightBorder[1]]

        if fovOverXAxis:
            xvals.append(radius * -np.sign(yt))
        elif over180:
            xvals.extend((radius, -radius))

        if fovOverYAxis:
            yvals.append(radius * np.sign(xt))
        elif over180:
            yvals.extend((radius, -radius))
        
        padding = self.agent.radius

        xmin = min(xvals) - padding 
        xmax = max(xvals) + padding
        ymin = min(yvals) - padding
        ymax = max(yvals) + padding

        # positions relative until now, make them absolute for the return
        # xmin, ymin, xmax, ymax 
        return [position[0] + xmin, position[1] + ymin, position[0] + xmax, position[1] + ymax]


    def check_goals(self, world):
        # Add this to its own class later -- need to separate the binary from the trinary sensors
        if self.use_goal_state:
            sensor_origin = self.agent.getPosition()
            for world_goal in world.goals:
                if isinstance(world_goal, CylinderGoal):
                    u = np.array(world_goal.center) - sensor_origin
                    if np.linalg.norm(u) < self.goal_sensing_range + world_goal.r:
                        d = self.circle_interesect_sensing_cone(u, world_goal.r)
                        if d is not None:
                            self.agent.agent_in_sight = None
                            self.current_state = 2
                            self.goal_detected = True
                            return self.goal_detected
        self.goal_detected = False
        return self.goal_detected

    def lines_segments_intersect(self, l1, l2):
        p1, q1 = l1
        p2, q2 = l2
        o1 = self.point_orientation(p1, q1, p2)
        o2 = self.point_orientation(p1, q1, q2)
        o3 = self.point_orientation(p2, q2, p1)
        o4 = self.point_orientation(p2, q2, q1)
        checkA = o1 != o2
        checkB = o3 != o4
        if checkA and checkB:
            return True
        return False

    def line_seg_int_point(self, line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            raise Exception('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

    def point_orientation(self, p1, p2, p3):
        """
        Used in calculating Line Segment Intersection
        See: https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/%C2%A0/
        Motivation: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
        """
        val = (float(p2[1] - p1[1]) * (p3[0] - p2[0])) - (float(p2[0] - p1[0]) * (p3[1] - p2[1]))
        rot = 0
        if val > 0:
            rot = 1
        elif val < 0:
            rot = -1
        return rot

    def determineState(self, real_value, agent, world=None):
        invert = self.invert
        if real_value:
            # Consider Reporting False Negative
            if np.random.random_sample() < self.fn:
                self.agent_in_sight = None
                self.current_state = 1 if invert else 0
                self.detection_id = 0
            else:
                self.agent_in_sight = agent
                self.current_state = 0 if invert else 1
                if agent:
                    self.detection_id = agent.detection_id

        else:
            # Consider Reporting False Positive
            if np.random.random_sample() < self.fp:
                self.agent_in_sight = None
                self.detection_id = 0
                self.current_state = 0 if invert else 1
            else:
                self.agent_in_sight = None
                self.current_state = 1 if invert else 0
                self.detection_id = 0

    def step(self, world, only_check_goals=False):
        super(BinaryFOVSensor, self).step(world=world)
        goal_detected = self.check_goals(world=world)
        if not goal_detected and not only_check_goals:
            self.checkForLOSCollisions(world=world)
        if self.store_history:
            if self.agent.agent_in_sight:
                self.history.append(int(self.agent.agent_in_sight.name))
            else:
                self.history.append(-1)

    def draw(self, screen, offset=((0, 0), 1.0)):
        super(BinaryFOVSensor, self).draw(screen, offset)
        pan, zoom = np.asarray(offset[0]), np.asarray(offset[1])
        zoom: float
        if self.show:
            # Draw Sensory Vector (Vision Vector)
            sight_color = (255, 0, 0)
            if self.current_state == 1:
                sight_color = (0, 255, 0)
            if self.current_state == 2:
                sight_color = (255, 255, 0)

            magnitude = self.r if self.agent.is_highlighted else self.agent.radius * 5

            head = np.asarray(self.agent.getPosition()) * zoom + pan
            e_left, e_right = self.getSectorVectors()
            e_left, e_right = np.asarray(e_left[:2]), np.asarray(e_right[:2])

            tail_l = head + magnitude * e_left * zoom
            tail_r = head + magnitude * e_right * zoom

            pygame.draw.line(screen, sight_color, head, tail_l)
            pygame.draw.line(screen, sight_color, head, tail_r)
            if self.agent.is_highlighted:
                width = max(1, round(0.01 * zoom))
                pygame.draw.circle(screen, sight_color + (50,), head, self.r * zoom, width)
                if self.wall_sensing_range:
                    pygame.draw.circle(screen, (150, 150, 150, 50), head, self.wall_sensing_range * zoom, width)
                AAR = self.getAARectContainingCone()
                AARtl = np.array(AAR[:2]) * zoom + pan
                AARbr = np.array(AAR[2:]) * zoom + pan
                pygame.draw.rect(screen, sight_color + (50,), pygame.Rect(*AARtl, *(AARbr - AARtl)), width)
                detected = [point.data for point in self.agent.world.quad.within_bb(quads.BoundingBox(*self.getAARectContainingCone()))]
                for agent in detected:
                    pygame.draw.circle(screen, pygame.colordict.THECOLORS["blue"], agent.pos * zoom + pan, agent.radius * zoom, width*3)

    def circle_interesect_sensing_cone(self, u, r):
        e_left, e_right = self.getSectorVectors()
        directional = np.dot(u, self.getBiasedSightAngle())
        if directional > 0:
            u = np.append(u, [0])
            cross_l = np.cross(e_left, u)
            cross_r = np.cross(u, e_right)
            sign_l = np.sign(cross_l)
            sign_r = np.sign(cross_r)
            added_signs = sign_l - sign_r
            sector_boundaries = np.all(added_signs == 0)
            if sector_boundaries:
                d_to_inter = np.linalg.norm(u)
                return d_to_inter

            # It may also be the case that the center of the agent is not within the FOV, but that some part of the
            # circle is visible and on the edges of the left and right viewing vectors.
            # LinAlg Calculations obtained from https://www.bluebill.net/circle_ray_intersection.html

            # u, defined earlier is the vector from the point of interest to the center of the circle
            # Project u onto e_left and e_right
            u_l = np.dot(u, e_left) * e_left
            u_r = np.dot(u, e_right) * e_right

            # Determine the minimum distance between the agent's center (center of circle) and the projected vector
            dist_l = np.linalg.norm(u - u_l)
            dist_r = np.linalg.norm(u - u_r)

            radius = r  # Note: Assumes homogenous radius
            if dist_l < radius:
                d_to_inter = np.linalg.norm(u)
                return d_to_inter
            if dist_r < radius:
                d_to_inter = np.linalg.norm(u)
                return d_to_inter
        return None

    def getDistance(self, a, b):
        return np.linalg.norm(b - a)

    def getLOSVector(self) -> List:
        if self.angle is None:
            return self.agent.orientation_uvec()

        return [
            math.cos(self.angle + self.agent.angle),
            math.sin(self.angle + self.agent.angle)
        ]

    def getBiasedSightAngle(self):
        angle: float = self.agent.angle + self.bias
        return vectorize(angle)

    def getSectorVectors(self):
        angle: float = self.agent.angle + self.bias
        span: float = self.theta

        leftBorder = vectorize(angle + span)
        rightBorder = vectorize(angle - span)
        return np.append(leftBorder, 0), np.append(rightBorder, 0)

    def as_config_dict(self):
        return {
            "type": "BinaryFOVSensor",
            "theta": self.theta,
            "bias": self.bias,
            "fp": self.fp,
            "fn": self.fn,
            "time_step_between_sensing": self.time_step_between_sensing,
            "store_history": self.store_history,
            "use_goal_state": self.use_goal_state,
            "wall_sensing_range": self.wall_sensing_range,
            "goal_sensing_range": self.goal_sensing_range,
            "agent_sensing_range": self.r,
            "seed": self.seed,
        }

    @staticmethod
    def from_dict(d):
        return BinaryFOVSensor(
            parent=None,
            theta=d["theta"],
            distance=d["agent_sensing_range"],
            bias=d["bias"],
            false_positive=d.get("fp", 0.0),
            false_negative=d.get("fn", 0.0),
            store_history=d["store_history"],
            detect_goal_with_added_state=d["use_goal_state"],
            wall_sensing_range=d["wall_sensing_range"],
            goal_sensing_range=d["goal_sensing_range"],
            seed=d["seed"] if "seed" in d else None,
        )
