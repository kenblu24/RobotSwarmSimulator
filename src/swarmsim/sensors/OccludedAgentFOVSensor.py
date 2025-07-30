from swarmsim.sensors.ObjectFOVSensor import ObjectFOVSensor
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

def colinearPointSegmentIntersect(seg: np.ndarray, point: np.ndarray, segsq = None):
    if not segsq:
        segsq = np.dot(seg, seg)
    sdp = np.dot(seg, point)
    return 0 <= sdp and sdp < segsq

def segSegIntersect(seg1: np.ndarray, seg2: np.ndarray):
    s1l = seg1[1] - seg1[0]
    s2l = seg2[1] - seg2[0]
    t12_0 = turn(s1l, seg2[0] - seg1[0])
    t12_1 = turn(s1l, seg2[1] - seg1[0])
    t21_0 = turn(s2l, seg1[0] - seg2[0])
    t21_1 = turn(s2l, seg1[1] - seg2[0])
    return ((((t12_0 < 0 and 0 < t12_1) or (t12_1 < 0 and 0 < t12_0)) and
        ((t21_0 < 0 and 0 < t21_1) or (t21_1 < 0 and 0 < t21_0))) or
        (t12_0 == 0 and colinearPointSegmentIntersect(s1l, seg2[0] - seg1[0])) or 
        (t12_1 == 0 and colinearPointSegmentIntersect(s1l, seg2[1] - seg1[0])) or 
        (t21_0 == 0 and colinearPointSegmentIntersect(s2l, seg1[0] - seg2[0])) or 
        (t21_1 == 0 and colinearPointSegmentIntersect(s2l, seg1[1] - seg2[0])))

# determine if the sector of an infinite circle defined by the first three arguments intersects the fourth argument point
def sectorPointIntersect(center, angleLeft, angleRight, point):

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
    
def segmentCircleIntersectionPoints(segPs: np.ndarray, center: np.ndarray, radius):
    origin = segPs[0]
    line = segPs[1] - origin
    intersectionPoints = lineCircleIntersectionPoints(line, center - origin, radius)
    lineSq = np.dot(line, line)
    onSegmentIntersectionPoints = [p for p in intersectionPoints if colinearPointSegmentIntersect(line, p, lineSq)]
    globalIntersectionPoints = [p + origin for p in onSegmentIntersectionPoints]
    return globalIntersectionPoints

                        
class OccludedAgentFOVSensor(AbstractSensor):
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

        self.objectSensor = ObjectFOVSensor(bias=self.bias, theta=self.theta, distance=self.r, agent=self.agent)
        self.objectSensor.binaryMode = False

        self.seed = seed
        if self.seed is not None:
            np.random.seed(self.seed)

    def checkOccludedVision(self, agent, world):
        viewSeg = np.array([self.agent.getPosition(), agent.getPosition()])
        for obj in self.objectSensor.sensedObjects:
            for i in range(len(obj.points)):
                if segSegIntersect(viewSeg, np.array([obj.points[i - 1], obj.points[i]])):
                    return False
        return True


    def senseAgent(self, agent, world) -> bool:
        if self.checkOccludedVision(agent, world):
            self.determineState(True, None, world)
            return True
        else:
            return False

    def checkForLOSCollisions(self, world: World) -> None:
        self.time_since_last_sensing += 1
        if self.time_since_last_sensing % self.time_step_between_sensing != 0:
            # Our sensing rate occurs less frequently than our dt physics update, so we need to
            #   only check for LOS collisions every n timesteps.
            return

        self.time_since_last_sensing = 0
        sensor_origin = self.agent.getPosition()

        self.objectSensor.checkForLOSCollisions(world)

        # use world.quad that tracks agent positions to retrieve the agents within the minimal rectangle that contains the FOV sector
        quadpoints = [point.data for point in world.quad.within_bb(quads.BoundingBox(*self.getAARectContainingSector(world)))]
        # filter agents to those within the sensing radius
        bag = [agent for agent in quadpoints if self.withinRadiusExclusiveFast(sensor_origin, agent.getPosition(), self.r)]

        # get left and right whiskers
        e_left, e_right = self.getSectorVectors()

        # true if the sensor fov is less than 180°
        l180 = self.theta * 2 < np.pi

        for agent in bag:
            if agent is self.agent:  # skip the agent the sensor is attached to
                continue

            if self.target_team and not agent.team == self.target_team:
                continue

            u = agent.getPosition() - sensor_origin  # vector to agent
            leftTurn = turn(u, e_left)
            rightTurn = turn(u, e_right)

            # if fov < 180 use between minor arc, otherwise use not between minor arc
            if rightTurn <= 0 and 0 <= leftTurn if l180 else not (leftTurn < 0 and 0 < rightTurn):
                if self.senseAgent(agent, world):
                    return
            elif not self.detect_only_origins:
                # circle whisker intercept correction
                # for left and right, check that vector u to the agent is in the correct direction and if the line of the whisker intersects the agent circle
                leftWhisker = (0 < np.dot(u, e_left[:2]) and lineCircleIntersect(e_left[:2], u, agent.radius))
                rightWhisker = (0 < np.dot(u, e_right[:2]) and lineCircleIntersect(e_right[:2], u, agent.radius))
                if leftWhisker or rightWhisker:
                    if self.senseAgent(agent, world):
                        return

        # if an agent was in the fov then this function would have returned, so determine the sensing state to be false
        self.determineState(False, None, world)
        return

    # get the smallest rectangle that contains the sensor fov sector
    def getAARectContainingSector(self, world: RectangularWorld):
        angle: float = self.agent.angle + self.bias  # global sensor angle
        span: float = self.theta  # angle fov sweeps to either side
        radius: float = self.r  # view radius
        position: list[float] = self.agent.pos.tolist()  # agent global position

        over180 = np.pi <= span * 2  # true if 180 <= FOV

        center = vectorize(angle)  # vector representing absolute look direction
        leftWhisker = vectorize(angle + span)  # vector representing left whisker
        rightWhisker = vectorize(angle - span)  # vector representing right whisker
        xaxis = (1, 0)  # vector representing positive x axis
        yaxis = (0, 1)  # vector representing positive y axis

        xts = np.sign(turn(xaxis, center))  # sign of turn from x axis to look direction
        fovOverXAxis = np.sign(turn(xaxis, leftWhisker)) != np.sign(turn(xaxis, rightWhisker))  # true if turns from x axis to whiskers have different signs

        yts = np.sign(turn(yaxis, center))  # sign of turn from y axis to look direction
        fovOverYAxis = np.sign(turn(yaxis, leftWhisker)) != np.sign(turn(yaxis, rightWhisker))  # true if turns from y axis to whiskers have different signs

        xmin = 0
        xmax = 0
        ymin = 0
        ymax = 0

        def xadd(val):  # extend either xmin or xmax if outside current range
            nonlocal xmin
            if val < xmin:
                xmin = val
            nonlocal xmax
            if xmax < val:
                xmax = val

        def yadd(val):  # extend either ymin or ymax if outside current range
            nonlocal ymin
            if val < ymin:
                ymin = val
            nonlocal ymax
            if ymax < val:
                ymax = val

        # consider the x coordinates of the whisker ends
        xadd(leftWhisker[0] * radius)
        xadd(rightWhisker[0] * radius)
        if fovOverXAxis:  # if over x axis, x range is maximized to radius either left or right
            xadd(radius * -yts)  # left or right is determined by the negated sign of the turn from the positive y axis to the look direction
            if over180:  # if also over 180, then y range is maximized to radius in the direction closer to the look direction
                yadd(radius * xts)
                if not fovOverYAxis:  # if over x axis, over 180, and not over y axis, y range is maximized in both directions
                    yadd(radius * -xts)

        # consider the y coordinates of the whisker ends
        yadd(leftWhisker[1] * radius)
        yadd(rightWhisker[1] * radius)
        if fovOverYAxis:  # if over y axis, y range is maximized to radius either up or down
            yadd(radius * xts)  # up or down is determined by the sign of the turn from the positive x axis to the look direction
            if over180:  # if also over 180, then x range is maximized to radius in the direction closer to the look direction
                xadd(radius * -yts)
                if not fovOverXAxis:  # if over y axis, over 180, and not over x axis, x range is maximized in both directions
                    xadd(radius * yts)

        # this padding of the rectangle is to account for and detect agents that would only be seen by the whisker circle intercept correction
        padding = 0 if self.detect_only_origins else world.maxAgentRadius

        # positions are relative until now, make them absolute for the return
        return [position[0] + xmin - padding, position[1] + ymin - padding, position[0] + xmax + padding, position[1] + ymax + padding]
        # xmin, ymin, xmax, ymax

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
                
                #test code for self.sensedObjects
                for obj in self.sensedObjects:
                    for i in range(-1, len(obj.points) - 1):
                        pygame.draw.line(screen, (255, 0, 255, 50), obj.points[i] * zoom + pan, obj.points[i + 1] * zoom + pan, width=3)
                        

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
