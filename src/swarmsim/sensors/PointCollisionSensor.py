from swarmsim.world.RectangularWorld import RectangularWorld
import pygame
import numpy as np
import math
from .AbstractSensor import AbstractSensor
from typing import List
from ..world.goals.Goal import CylinderGoal
from ..util.collider.AABB import AABB
from shapely.geometry import Polygon

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
# this value is positive if a left turn is the fastest way to go from p1 to p2,
#   zero if p1 and p2 are colinear, and negative otherwise
def turn(p1, p2):
    return p1[0] * p2[1] - p2[0] * p1[1]


# project vector a onto vector b
def project(a, b):
    return b * (np.dot(a, b) / np.dot(b, b))


# determine if the line in the direction of the first arugment intersects the circle defined by the second and third arguments
def lineCircleIntersect(line, center, radius):
    clDiffVec = center - project(center, line)
    return np.dot(clDiffVec, clDiffVec) <= radius**2


class PointCollisionSensor(AbstractSensor):
    config_vars = AbstractSensor.config_vars + [
        'distance', 'bias', 'fp', 'fn',
        'time_step_between_sensing', 'invert',
        'store_history', 'show', 'target_team'
    ]

    DEBUG = False

    def __init__(
        self,
        agent=None,
        parent=None,
        distance=100.0,
        bias=0.0,
        false_positive=0.0,
        false_negative=0.0,
        time_step_between_sensing=1,
        invert=False,
        store_history=False,
        show=True,
        seed=None,
        target_team=None,
        **kwargs
    ):
        super().__init__(agent=agent, parent=parent, seed=seed, **kwargs)
        # self.angle = 0.0
        self.bias = bias
        self.fp = false_positive
        self.fn = false_negative
        self.time_step_between_sensing = time_step_between_sensing
        self.time_since_last_sensing = 0
        self.history = []
        self.store_history = store_history
        self.show = show
        self.invert = invert
        self.goal_detected = False
        self.detection_id = 0
        self.target_team = target_team

        # self.detect_only_origins = False
        self.r = distance

    def checkForLOSCollisions(self, world: RectangularWorld) -> None:
        # Mathematics obtained from Sundaram Ramaswamy
        # https://legends2k.github.io/2d-fov/design.html
        # See section 3.1.1.2
        self.time_since_last_sensing += 1
        if self.time_since_last_sensing % self.time_step_between_sensing != 0:
            # Our sensing rate occurs less frequently than our dt physics update, so we need to
            #   only check for LOS collisions every n timesteps.
            return

        self.time_since_last_sensing = 0

        pos = self.parent.pos
        u = vectorize(self.parent.angle + self.bias) * self.r
        point = pos + u

        if world.quad:
            # use world.quad that tracks agent positions to retrieve the agents' BB containing the point
            quadpoints = [node.data for ret in world.quad._root.find_node(point)
                          if ret is not None for node, _searched in ret]
            bag = np.array([agent for data in quadpoints for agent in data], dtype=object)
        else:
            bag = np.array(world.population, dtype=object)
        if not bag.size:
            bag = []

        self.__candidates = bag

        bag = [a for a in bag if
               a is not self.agent
               and self.target_team is None
               or a.team == self.target_team]

        for agent in bag:
            if agent.is_poly:
                intersects = Polygon(agent.points).contains(point)
            else:
                intersects = agent.radius <= np.linalg.norm(point - agent.pos)
            if intersects:
                break
        else:  # if no agents are found to intersect
            agent = None

        self.determineState(intersects, agent, world)

    def determineState(self, real_value, agent, world=None):
        invert = self.invert
        if real_value:
            # Consider Reporting False Negative
            if self.rng.random() < self.fn:
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
            if self.rng.random() < self.fp:
                self.agent_in_sight = None
                self.detection_id = 0
                self.current_state = 0 if invert else 1
            else:
                self.agent_in_sight = None
                self.current_state = 1 if invert else 0
                self.detection_id = 0

    def step(self, world, only_check_goals=False):
        super(PointCollisionSensor, self).step(world=world)
        self.checkForLOSCollisions(world=world)
        if self.store_history:
            if self.agent.agent_in_sight:
                self.history.append(int(self.agent.agent_in_sight.name))
            else:
                self.history.append(-1)

    def draw(self, screen, offset=((0, 0), 1.0)):
        super(PointCollisionSensor, self).draw(screen, offset)
        pan, zoom = np.asarray(offset[0]), np.asarray(offset[1])
        zoom: float
        if self.show:
            # Draw Sensory Vector (Vision Vector)
            sight_color = (255, 0, 0)
            if self.current_state == 1:
                sight_color = (0, 255, 0)
            if self.current_state == 2:
                sight_color = (255, 255, 0)

            # draw the whiskers
            # length = actual sensor range if agent is selected/highlighted, otherwise draw relative to agent radius
            magnitude = self.r  # if self.agent.is_highlighted else self.agent.radius * 5

            head = np.asarray(self.parent.getPosition()) * zoom + pan
            tail = head + magnitude * vectorize(self.parent.angle + self.bias) * zoom

            pygame.draw.line(screen, sight_color, head, tail)
            if self.agent.is_highlighted:
                width = max(1, round(0.01 * zoom))

                if not self.DEBUG:
                    return
                # DEBUG DRAWINGS:
                for agent in self.__candidates:
                    pygame.draw.circle(screen, pygame.colordict.THECOLORS["blue"],
                                       agent.pos * zoom + pan, agent.radius * zoom, width * 3)

    @staticmethod
    def from_dict(d):
        aliases = {
            'fp': 'false_positive',
            'fn': 'false_negative',
        }
        return PointCollisionSensor(**d)
