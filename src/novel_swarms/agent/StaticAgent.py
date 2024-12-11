from typing import Tuple
import pygame
import random
import math
import numpy as np
from dataclasses import dataclass
from ..config import filter_unexpected_fields, associated_type
from copy import deepcopy
from .Agent import Agent, BaseAgentConfig
from ..sensors.GenomeDependentSensor import GenomeBinarySensor
from ..util.collider.AABB import AABB
from ..util.collider.CircularCollider import CircularCollider
from ..util.timer import Timer

# typing
from typing import override
from ..world.RectangularWorld import RectangularWorldConfig


@associated_type("StaticAgent")
@filter_unexpected_fields
@dataclass
class StaticAgentConfig(BaseAgentConfig):
    seed: int | None = None
    world_config: RectangularWorldConfig | None = None
    agent_radius: float = 5
    dt: float = 1.0
    body_color: tuple[int, int, int] = (255, 255, 255)
    body_filled: bool = False

    def attach_world_config(self, world_config):
        self.world = world_config


class StaticAgent(Agent):
    DEBUG = True

    def __init__(self, config: StaticAgentConfig, name=None, initialize=True) -> None:
        super().__init__(config, name, initialize=False)

        if config.seed is not None:
            self.seed = config.seed

        self.radius = config.agent_radius
        self.dt = config.dt
        self.is_highlighted = False
        self.agent_in_sight = None
        self.body_filled = config.body_filled
        self.body_color = config.body_color

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    @override
    def step(self, check_for_world_boundaries=None, world=None, check_for_agent_collisions=None) -> None:
        super().step()

    @override
    def draw(self, screen, offset=((0, 0), 1.0)) -> None:
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen)

        # Draw Cell Membrane
        filled = 0 if (self.is_highlighted or self.stopped_duration or self.body_filled) else 1
        color = self.body_color if not self.stopped_duration else (255, 255, 51)
        pos = np.asarray(self.getPosition()) * zoom + pan
        pygame.draw.circle(screen, color, (*pos,), self.radius * zoom, width=filled)  # pyright: ignore[reportArgumentType]

        # "Front" direction vector
        head = np.asarray(self.getFrontalPoint()) * zoom + pan
        tail = pos
        vec = head - tail
        mag = self.radius * 2
        vec_with_magnitude = tail + vec * mag
        pygame.draw.line(screen, (255, 255, 255), tail, vec_with_magnitude)
        if self.DEBUG:
            self.debug_draw(screen)

    def build_collider(self):
        return CircularCollider(self.x_pos, self.y_pos, self.radius)

    def debug_draw(self, screen):
        self.get_aabb().draw(screen)

    def get_aabb(self):
        """
        Return the Bounding Box of the agent
        """
        x, y = self.pos
        top_left = (x - self.radius, y - self.radius)
        bottom_right = (x + self.radius, y + self.radius)
        return AABB(top_left, bottom_right)

    def __str__(self) -> str:
        x, y = self.pos
        return f"(x: {x}, y: {y}, r: {self.radius}, θ: {self.angle})"
