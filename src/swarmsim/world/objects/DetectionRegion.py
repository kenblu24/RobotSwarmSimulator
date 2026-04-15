from dataclasses import dataclass, field

import numpy as np
import quads

from ...config import filter_unexpected_fields, associated_type
from ...agent.StaticAgent import StaticAgent, StaticAgentConfig
from ...util.collider.CollisionMode import CollisionMode, collision_mode

# typing
from typing import Any, override, Iterable


@associated_type("DetectionRegion")
@filter_unexpected_fields
@dataclass
class DetectionRegionConfig(StaticAgentConfig):
    collides: int | str | CollisionMode = CollisionMode.Detect
    grounded: bool = True
    target_teams: Iterable | None = None
    target_name: str | Any | None = None


class DetectionRegion(StaticAgent):
    def __init__(self, config: DetectionRegionConfig, world, name=None, initialize=True):
        super().__init__(config, world, name, initialize)
        self.target_teams = None if config.target_teams is None else set(config.target_teams)
        self.target_name = config.target_name

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
        if CollisionMode.Detect not in self.collides:
            return
        if refresh:
            self.aabb = self.make_aabb()
        bag = [other for other in world.population
                            if self != other
                            and self.aabb.intersects_bb(other.make_aabb() if refresh else other.aabb)]
        self.collided = []
        if not bag:
            return
        collider = self.build_collider()
        for other in bag:
            if (self == other
                or (self.target_name is not None and other.name != self.target_name)
                or (self.target_teams is not None and other.team not in self.target_teams)
                or CollisionMode.Detect not in collision_mode(other.collides)):
                continue
            other_collider = other.build_collider()
            correction = collider.correction(other_collider, rng=rng)
            if np.isnan(correction).any():
                continue
            self.collided.append(other)
            self.collision_flag = True
