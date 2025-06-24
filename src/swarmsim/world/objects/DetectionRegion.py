from dataclasses import dataclass, field

import numpy as np

from ...config import filter_unexpected_fields, associated_type
from ...agent.StaticAgent import StaticAgent, StaticAgentConfig

# typing
from typing import Any, override


@associated_type("DetectionRegion")
@filter_unexpected_fields
@dataclass
class DetectionRegionConfig(StaticAgentConfig):
    collides: bool | int = True
    grounded: bool = True


class DetectionRegion(StaticAgent):
    def __init__(self, config, world, name=None, initialize=True):
        super().__init__(config, world, name, initialize)

    @override
    def step(self, check_for_world_boundaries=None, world=None, check_for_agent_collisions=None) -> None:
        world = world or self.world
        self.check_collisions(world, self.rng, refresh=False)

        self.body_color = (0, 250, 0) if self.collision_flag else (150, 150, 150)

    @override
    def draw_direction(self, screen, offset=((0, 0), 1.0)):
        pass

    def check_collisions(self, world, rng=None, refresh=False):
        if rng is None:
            rng = self.rng
        self.collision_flag = False
        if refresh:
            self.aabb = self.make_aabb()
        candidates = [other for other in world.population if self != other
                            and self.aabb.intersects_bb(other.make_aabb() if refresh else other.aabb)]
        self.collided = []
        if not candidates:
            return
        collider = self.build_collider()
        for other in candidates:
            other_collider = other.build_collider()
            correction = collider.correction(other_collider, rng=rng)
            if np.isnan(correction).any():
                continue
            self.collided.append(other)
            self.collision_flag = True
