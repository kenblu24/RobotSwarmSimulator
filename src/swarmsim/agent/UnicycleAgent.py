from dataclasses import dataclass, field

import pygame
import numpy as np

from ..config import filter_unexpected_fields, associated_type
from .MazeAgent import MazeAgent, MazeAgentConfig
from ..util.collider.Collider import CircularCollider, PolyCollider
from ..util.collider.AngleSensitiveCC import AngleSensitiveCC

# typing
from typing import override


@associated_type("UnicycleAgent")
@filter_unexpected_fields
@dataclass
class UnicycleAgentConfig(MazeAgentConfig):
    pass


class UnicycleAgent(MazeAgent):
    SEED = -1
    DEBUG = False

    def __init__(self, config: UnicycleAgentConfig, world, name=None, initialize=True) -> None:
        config: UnicycleAgentConfig
        super().__init__(config, world, name=name, initialize=False)

        self.c_now = (0, 0)

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    @override
    def step(self, check_for_world_boundaries=None, world=None, check_for_agent_collisions=None) -> None:

        super().step(world=world, check_for_world_boundaries=check_for_world_boundaries,
                     check_for_agent_collisions=check_for_agent_collisions)

        self.add_to_trace(self.pos)

    @override
    def build_collider(self):
        if self.is_poly:
            self.collider = PolyCollider(self.poly_rotated + self.pos)
        elif self.stop_on_collision:
            self.collider = AngleSensitiveCC(*self.pos, self.radius, self.angle, sensitivity=45)
        else:
            self.collider = CircularCollider(*self.pos, self.radius)
        return self.collider

    def draw_trace(self, screen):
        for pos in self.trace_path:
            pygame.draw.circle(screen, self.trace_color, pos, 2)

    def add_to_trace(self, pos):
        self.trace_path.append(pos)

    def get_icc(self):
        v, w = self.c_now
        r = v / w
        x, y = self.pos
        return x - (r * np.sin(self.angle)), y + (r * np.cos(self.angle))

    @override
    def __str__(self) -> str:
        x, y = self.pos
        return f"(x: {x}, y: {y}, r: {self.radius}, θ: {self.angle})"
