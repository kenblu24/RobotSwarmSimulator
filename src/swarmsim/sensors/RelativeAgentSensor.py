from swarmsim.world.RectangularWorld import RectangularWorld
import pygame
import numpy as np
import math
from .AbstractSensor import AbstractSensor
from typing import List
from ..world.goals.Goal import CylinderGoal
from ..util.collider.AABB import AABB

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..world.World import World
else:
    World = None

import warnings
import quads


class BinaryFOVSensor(AbstractSensor):
    config_vars = AbstractSensor.config_vars + [
        'theta', 'distance', 'bias', 'false_positive', 'false_negative',
        'walls', 'wall_sensing_range', 'time_step_between_sensing', 'invert',
        'store_history', 'detect_goal_with_added_state', 'show', 'target_team'
    ]

    DEBUG = False

    def __init__(
        self,
        agent=None,
        parent=None,
        distance=100.0,
        time_step_between_sensing=1,
        store_history=False,
        show=True,
        seed=None,
        target_team=None,
        **kwargs
    ):
        super().__init__(agent=agent, parent=parent, seed=seed, **kwargs)
        self.time_step_between_sensing = time_step_between_sensing
        self.time_since_last_sensing = 0
        self.history = []
        self.store_history = store_history
        self.show = show
        self.target_team = target_team

        self.detect_only_origins = False

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

        sensor_origin = self.agent.pos

        if world.quad:
            # use world.quad that tracks agent positions to retrieve the agents within the minimal rectangle that contains the FOV sector
            quadpoints = [point.data for point in world.quad.within_bb(
                quads.BoundingBox(*self.getAARectContainingSector(world)))]
            agents = np.array([agent for data in quadpoints for agent in data], dtype=object)
        else:
            agents = np.array(world.population, dtype=object)
        # filter agents to those within the sensing radius
        if agents.size:  # if any agents found
            positions = np.array([agent.pos for agent in agents])
            distances = np.linalg.norm(positions - sensor_origin, axis=1)
            is_close = distances < self.r
            bag = agents[is_close]
        else:
            bag = []

        detected = []

        for other in bag:
            if other is self.agent:  # skip the agent the sensor is attached to
                continue

            if self.target_team and not other.team == self.target_team:
                continue

            u = other.pos - sensor_origin  # vector to agent


        # if an agent was in the fov then this function would have returned, so determine the sensing state to be false
        self.determineState(False, None, world)
        return

    def step(self, world, only_check_goals=False):
        super(BinaryFOVSensor, self).step(world=world)
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

            # draw the whiskers
            # length = actual sensor range if agent is selected/highlighted, otherwise draw relative to agent radius
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
                # pygame.draw.circle(screen, sight_color + (50,), head, self.r * zoom, width)
                # draw the arc of the sensor cone
                range_bbox = AABB.from_center_wh(head, self.r * 2 * zoom)
                langle = self.agent.angle + self.angle + self.theta
                rangle = self.agent.angle + self.angle - self.theta
                pygame.draw.arc(screen, sight_color + (50,), range_bbox.to_rect(), -langle, -rangle, width)

                if not self.DEBUG:
                    return
                # DEBUG DRAWINGS:
                if self.wall_sensing_range:
                    pygame.draw.circle(screen, (150, 150, 150, 50), head, self.wall_sensing_range * zoom, width)
                AAR = self.getAARectContainingSector(self.agent.world)
                AARtl = np.array(AAR[:2]) * zoom + pan
                AARbr = np.array(AAR[2:]) * zoom + pan
                pygame.draw.rect(screen, sight_color + (50,), pygame.Rect(*AARtl, *(AARbr - AARtl)), width)
                detected = [point.data for point in self.agent.world.quad.within_bb(quads.BoundingBox(*AAR))]
                for agent in detected:
                    pygame.draw.circle(screen, pygame.colordict.THECOLORS["blue"], agent.pos * zoom + pan, agent.radius * zoom, width * 3)

    # this function has been replaced by a more efficient procedure in checkForLOSCollisions and is no longer called there
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
