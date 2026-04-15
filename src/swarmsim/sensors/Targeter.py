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


class Targeter(AbstractSensor):
    config_vars = AbstractSensor.config_vars + [
        'distance', 'bias', 'fn', 'time_step_between_sensing',
        'target_mode', 'hold', 'attribute', 'store_history', 'show',
        'target_team', 'target_name', 'seed',
    ]

    DEBUG = False

    def __init__(
        self,
        agent=None,
        parent=None,
        distance=float('inf'),
        false_negative=0.0,
        time_step_between_sensing=1,
        target_mode=None,
        hold='retargetonloss',  # 'alwaysretarget', 'retargetonloss', 'holdlost', 'holdfirst'
        attribute=None,
        store_history=False,
        show=False,
        seed=None,
        target_team=None,
        target_name=None,
        **kwargs
    ):
        super().__init__(agent=agent, parent=parent, seed=seed, **kwargs)
        self.fn = false_negative
        self.time_step_between_sensing = time_step_between_sensing
        self.time_since_last_sensing = 0
        self.target_mode = target_mode
        self.hold = hold
        self.old_target = None
        self.current_target = None
        self._target = None
        self.attribute = attribute
        self.history = []
        self.store_history = store_history
        self.show = show
        self.goal_detected = False
        self.target_team = target_team
        self.target_name = target_name

        # self.detect_only_origins = False
        self.r = distance

    def look_for_agents(self, world: RectangularWorld):
        # Mathematics obtained from Sundaram Ramaswamy
        # https://legends2k.github.io/2d-fov/design.html
        # See section 3.1.1.2
        self.time_since_last_sensing += 1
        if self.time_since_last_sensing % self.time_step_between_sensing != 0:
            # Our sensing rate occurs less frequently than our dt physics update, so we need to
            #   only check for LOS collisions every n timesteps.
            return

        self.time_since_last_sensing = 0

        bag = []
        for a in world.population:
            if (a is self.agent
               or (self.target_name is not None and a.name != self.target_name)
               or (self.target_team is not None and a.team != self.target_team)):
                continue
            bag.append(a)
            if self.target_mode is None:
                break

        return bag

    def select_target(self, bag):
        if not bag:
            return

        def distance(other):
            return np.linalg.norm(self.parent.pos - np.asarray(other.pos))

        if self.target_mode is None:
            return bag[0]
        else:
            if self.target_mode == 'closest':
                return min(bag, key=distance)
            elif self.target_mode == 'furthest':
                return max(bag, key=distance)

    def fuzz(self, agent):
        if agent:
            # Consider Reporting False Negative
            if self.rng.random() < self.fn:
                return None
            else:
                return agent

    def determineState(self, agent):
        self.current_target = agent
        if self.attribute is None:
            self.current_state = agent
        else:
            self.current_state = getattr(agent, self.attribute)

        if self.store_history:
            if self.attribute is None:
                self.history.append(self.current_state.name)
            else:
                self.history.append(self.current_state)

    def step(self, world, only_check_goals=False):
        super(Targeter, self).step(world=world)
        if self.current_target is not None:
            self.old_target = self.current_target

        if self.hold == 'alwaysretarget':
            self._target = self.select_target(self.look_for_agents(world=world))
            self.determineState(self.fuzz(self._target))
        elif self.hold == 'retargetonloss' or self.hold == 'holdlost':
            # always check which agents are within range
            bag = self.look_for_agents(world=world)
            # if no longer in range, choose new target
            if self.current_target is None or self.current_target not in bag:
                self._target = self.select_target(bag)
            # otherwise, maintain previous target
            # always fuzz
            target = self.fuzz(self._target)
            if self.hold == 'holdlost' and target is None:
                # if still no target, revert to previous target
                target = self.old_target
            self.determineState(target)
        elif self.hold == 'holdfirst':
            # pick a target and stick to it
            if self.current_target is None or self._target not in world.population:
                self._target = self.select_target(self.look_for_agents(world=world))
            self.determineState(self._target)

    def draw(self, screen, offset=((0, 0), 1.0)):
        super(Targeter, self).draw(screen, offset)
        pan, zoom = np.asarray(offset[0]), np.asarray(offset[1])
        zoom: float
        # if self.show:
        #     # Draw Sensory Vector (Vision Vector)
        #     sight_color = (255, 0, 0)
        #     if self.current_state == 1:
        #         sight_color = (0, 255, 0)
        #     if self.current_state == 2:
        #         sight_color = (255, 255, 0)

        #     # draw the whiskers
        #     # length = actual sensor range if agent is selected/highlighted, otherwise draw relative to agent radius
        #     magnitude = self.r  # if self.agent.is_highlighted else self.agent.radius * 5

        #     head = np.asarray(self.parent.getPosition()) * zoom + pan
        #     tail = head + magnitude * vectorize(self.parent.angle + self.bias) * zoom

        #     pygame.draw.line(screen, sight_color, head, tail)
        #     if self.agent.is_highlighted:
        #         width = max(1, round(0.01 * zoom))

        #         if not self.DEBUG:
        #             return
        #         # DEBUG DRAWINGS:
        #         for agent in self.__candidates:
        #             pygame.draw.circle(screen, pygame.colordict.THECOLORS["blue"],
        #                                agent.pos * zoom + pan, agent.radius * zoom, width * 3)

    @staticmethod
    def from_dict(d):
        return Targeter(
            **d
        )
