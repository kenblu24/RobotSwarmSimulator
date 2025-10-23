from swarmsim.world.RectangularWorld import RectangularWorld
import pygame
import numpy as np
import math
from .AbstractSensor import AbstractSensor
from typing import List
from ..util.collider.AABB import AABB

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..world.World import World
else:
    World = None

import warnings
import quads


class RangedSensor(AbstractSensor):
    config_vars = AbstractSensor.config_vars + [
        'distance', 'time_step_between_sensing', 'detect_only_origins',
        'show', 'target_team'
    ]

    DEBUG = False

    def __init__(
        self,
        agent=None,
        parent=None,

        distance=100.0,
        time_step_between_sensing=1,
        detect_only_origins=False,
        target_team=None,
        **kwargs
    ):
        super().__init__(agent=agent, parent=parent, **kwargs)
        self.time_step_between_sensing = time_step_between_sensing
        self.time_since_last_sensing = 0

        self.detect_only_origins = detect_only_origins
        self.target_team = target_team
        self.agents_within_range = []
        self.r = distance

    def ready_to_sense(self):
        # Our sensing rate occurs less frequently than our dt physics update, so we need to
        #   only check for LOS collisions every n timesteps.
        self.time_since_last_sensing += 1
        if self.time_since_last_sensing % self.time_step_between_sensing == 0:
            self.time_since_last_sensing = 0
            return True
        return False

    def make_bounding_box(self):
        mins = self.agent.pos - self.r
        maxs = self.agent.pos + self.r
        return *mins, *maxs

    def get_agents_within_range(self, world: RectangularWorld) -> None:

        infinite_range = self.r == float('inf')

        if world.quad and not infinite_range:
            # use world.quad that tracks agent positions to retrieve the agents within the minimal rectangle that contains the FOV sector
            quadpoints = [point.data for point in world.quad.within_bb(
                quads.BoundingBox(*self.make_bounding_box()))]
            agents = np.array([agent for data in quadpoints for agent in data], dtype=object)
        else:
            agents = np.array(world.population, dtype=object)
        # filter agents to those within the sensing radius
        if infinite_range:
            bag = agents
        elif agents.size:  # if any agents found
            positions = np.array([agent.pos for agent in agents])
            distances = np.linalg.norm(positions - self.position, axis=1)
            is_close = distances < self.r
            bag = agents[is_close]
        else:
            bag = []

        if self.target_team:
            bag = [agent for agent in bag if agent.team == self.target_team]
        else:
            bag = [agent for agent in bag]

        self.agents_within_range = bag
        return bag

    def draw(self, screen, offset=((0, 0), 1.0)):
        super().draw(screen, offset)
        pan, zoom = np.asarray(offset[0]), np.asarray(offset[1])
        zoom: float
        if self.show:
            # Draw Sensory Vector (Vision Vector)
            color = (150, 150, 150)
            spos = self.position * zoom + pan

            # draw the whiskers
            # length = actual sensor range if agent is selected/highlighted, otherwise draw relative to agent radius
            if self.agent.is_highlighted:
                width = max(1, round(0.01 * zoom))
                pygame.draw.circle(screen, color + (50,), spos, self.r * zoom, width)
                # draw the arc of the sensor cone

                if not self.DEBUG:
                    return
                # DEBUG DRAWINGS:
                AAR = self.make_bounding_box() * zoom
                AARtl = AAR[:2] + pan
                AARbr = AAR[2:] + pan
                pygame.draw.rect(screen, color + (50,), pygame.Rect(*AARtl, *(AARbr - AARtl)), width)
                detected = self.agents_within_range
                for agent in detected:
                    pygame.draw.circle(screen, pygame.colordict.THECOLORS["blue"], agent.pos * zoom + pan, agent.radius * zoom, width * 3)
