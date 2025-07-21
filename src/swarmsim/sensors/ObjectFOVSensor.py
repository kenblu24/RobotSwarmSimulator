from swarmsim.world.objects.StaticObject import StaticObject
from swarmsim.world.RectangularWorld import RectangularWorld
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

# convert an angle to a representative unit vector
def vectorize(angle):
    return np.array((np.cos(angle), np.sin(angle)))

# compute the vector turn value from origin→p1 to origin→p2
# this value is positive if a left turn is the fastest way to go from p1 to p2, zero if p1 and p2 are colinear, and negative otherwise
def turn(p1, p2):
    return p1[0] * p2[1] - p2[0] * p1[1]

# project vector a onto vector b
def project(a, b):
    return b * (np.dot(a, b) / np.dot(b, b))

# determine if the line in the direction of the first arugment intersects the circle defined by the second and third arguments
def lineCircleIntersect(line, center, radius):
    clDiffVec = center - project(center, line)
    return np.dot(clDiffVec, clDiffVec) <= radius**2

# determine if the sector of an infinite circle defined by the first three arguments intersects the fourth argument point
def sectorPointIntersect(center, radius, angleLeft, angleRight, point):
    u = point - center # vector to agent
    leftTurn = turn(u, vectorize(angleLeft))
    rightTurn = turn(u, vectorize(angleRight))
    
    l180 = (angleLeft - angleRight) % (np.pi * 2) < np.pi

    # if fov < 180 use between minor arc, otherwise use not between minor arc
    return rightTurn <= 0 and 0 <= leftTurn if l180 else not (leftTurn < 0 and 0 < rightTurn)

def lineCircleIntersectionPoints(line: np.ndarray, center: np.ndarray, radius):
    unitLine = line / np.linalg.norm(line)
    projectCenterToLine = project(center, line)
    clDiffVec = center - projectCenterToLine
    clDiffVecMagsq = np.dot(clDiffVec, clDiffVec)
    if radius**2 < clDiffVecMagsq:
        return []
    midDist = np.sqrt(radius**2 - clDiffVecMagsq)
    return [projectCenterToLine + midDist * unitLine, projectCenterToLine - midDist * unitLine]
    

class ObjectFOVSensor(AbstractSensor):
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
        self.time_since_last_sensing += 1
        if self.time_since_last_sensing % self.time_step_between_sensing != 0:
            # Our sensing rate occurs less frequently than our dt physics update, so we need to
            #   only check for LOS collisions every n timesteps.
            return

        self.time_since_last_sensing = 0
        sensor_origin = self.agent.getPosition()

        # get left and right whiskers
        e_left, e_right = self.getSectorVectors()

        
        # true if the sensor fov is less than 180°
        l180 = self.theta * 2 < np.pi

        for obj in world.objects:
            pass
            

        # if an agent was in the fov then this function would have returned, so determine the sensing state to be false
        self.determineState(False, None, world)
        return

    def check_goals(self, world):
        # Add this to its own class later -- need to separate the binary from the trinary sensors
        if self.use_goal_state:
            pass
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
        super(ObjectFOVSensor, self).step(world=world)
        goal_detected = self.check_goals(world=world)
        if not goal_detected and not only_check_goals:
            self.checkForLOSCollisions(world=world)
        if self.store_history:
            if self.agent.agent_in_sight:
                self.history.append(int(self.agent.agent_in_sight.name))
            else:
                self.history.append(-1)

    def draw(self, screen, offset=((0, 0), 1.0)):
        super(ObjectFOVSensor, self).draw(screen, offset)
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
                
                #test code for lineCircleIntersectionPoints
                for obj in self.agent.world.objects:
                    for i in range(-1, len(obj.points) - 1):
                        o = obj.points[i]
                        l = obj.points[i + 1]
                        ips = lineCircleIntersectionPoints(l - o, self.agent.getPosition() - o, self.r)
                        for p in ips:
                            gp = p + o
                            pygame.draw.circle(screen, sight_color + (50,), gp * zoom + pan, 0.05 * zoom, width)

    def withinRadiusExclusiveFast(self, origin, other, radius):
        diff = origin - other
        return np.dot(diff, diff) < radius**2

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
            "type": "ObjectFOVSensor",
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
        return ObjectFOVSensor(
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
