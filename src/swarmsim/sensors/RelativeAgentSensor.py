from swarmsim.world.RectangularWorld import RectangularWorld
import pygame
import numpy as np
import math
from .RangedSensor import RangedSensor
from typing import List
from ..util.collider.AABB import AABB

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..world.World import World
else:
    World = None

import warnings
import quads


class RelativeAgentSensor(RangedSensor):
    DEBUG = False

    def __init__(
        self,
        agent=None,
        parent=None,
        distance=100.0,
        time_step_between_sensing=1,
        show=True,
        target_team=None,

        **kwargs
    ):
        super().__init__(
            agent=agent,
            parent=parent,
            distance=distance,
            time_step_between_sensing=time_step_between_sensing,
            detect_only_origins=True,
            target_team=target_team,
            **kwargs
        )
        self.time_step_between_sensing = time_step_between_sensing
        self.time_since_last_sensing = 0
        self.history = []
        self.store_history = False
        self.show = show
        self.target_team = target_team

        self.detect_only_origins = False

        self.r = distance

    def get_positions(self, world: RectangularWorld) -> None:
        bag = self.get_agents_within_range(world)
        positions = np.array([agent.pos for agent in bag])
        return positions

    def step(self, world, **kwargs):
        self.current_state = self.get_positions(world)

    def as_config_dict(self):
        return {
            "type": type(self).__name__,
            "time_step_between_sensing": self.time_step_between_sensing,
            "store_history": self.store_history,
            "agent_sensing_range": self.r,
            "target_team": self.target_team,
        }

    @classmethod
    def from_dict(cls, d):
        return cls(
            parent=None,
            distance=d["agent_sensing_range"],
            time_step_between_sensing=d.get("time_step_between_sensing", 1),
            target_team=d.get("target_team", None),
        )
