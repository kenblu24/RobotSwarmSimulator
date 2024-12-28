from functools import lru_cache, cached_property

import pygame
import numpy as np
from shapely.geometry import Polygon
from shapely import transform, get_coordinates

from dataclasses import dataclass, field
from ..config import filter_unexpected_fields, associated_type
from copy import deepcopy
from .Agent import Agent, BaseAgentConfig
from ..util.collider.AABB import AABB
from ..util.collider.Collider import CircularCollider, PolyCollider

# typing
from typing import override
from ..world.RectangularWorld import RectangularWorldConfig


@associated_type("StaticAgent")
@filter_unexpected_fields
@dataclass
class StaticAgentConfig(BaseAgentConfig):
    seed: int | None = None
    world_config: RectangularWorldConfig | None = None
    agent_radius: float = 0.
    body_color: tuple[int, int, int] = (255, 255, 255)
    body_filled: bool = False
    collides: bool | int = True
    points: list[tuple[float, float]] | np.ndarray | str = field(default_factory=list)
    anchor_point: None | tuple[float, float] | str = None

    def attach_world_config(self, world_config):
        self.world = world_config


class StaticAgent(Agent):
    DEBUG = True

    def __init__(self, config: StaticAgentConfig, world, name=None, initialize=True) -> None:
        super().__init__(config, world, name, initialize=False)

        if config.seed is not None:
            self.seed = config.seed
            self.rng = np.random.default_rng(self.seed)
        else:
            self.seed = np.random.randint(0, 90000)
            self.rng = np.random.default_rng(self.seed)

        # set hull shape -> self.points from config (if any)
        if isinstance(config.points, str):
            # assume string is from an SVG file
            # attempt to extract a single polygon to use as agent hull
            from ..util.geometry.svg_extraction import SVG
            polys = SVG(config.points).get_polygons()
            if not polys:
                raise Exception("No polygons found in SVG.")
            elif len(polys) == 1:
                points, _classes = polys[0]
                self.points = np.asarray(points, dtype=np.float64)
            else:
                raise Exception("Multiple polygons found in SVG.")
        else:
            self.points = np.asarray(config.points, dtype=np.float64)  # is empty array if unspecified

        # handle points_shift option
        self.shift = np.zeros(2)
        if self.points.any():
            if isinstance(config.anchor_point, str):
                if 'center' in config.anchor_point:
                    aabb = self.make_aabb()
                    wh = np.array([aabb.width, aabb.height], dtype=float)
                    self.shift = -wh / 2 - aabb._min
                    self.points += self.shift
                elif 'centroid' in config.anchor_point:
                    poly = Polygon(self.points)
                    self.shift = -get_coordinates(poly.centroid).reshape(2)
                    self.points += self.shift
                if 'inplace' in config.anchor_point:
                    self.pos -= self.shift
            elif isinstance(config.anchor_point, (tuple, list, np.ndarray)):
                self.shift = np.asarray(config.anchor_point, dtype=np.float64)
                self.points += self.shift
            elif config.anchor_point is not None:
                raise ValueError(f"Unknown points_shift type: {config.anchor_point}")

        self.radius = self.get_simple_poly_radius() or config.agent_radius or 0.5
        self.dt = world.dt
        self.is_highlighted = False
        self.agent_in_sight = None
        self.body_filled = config.body_filled
        self.body_color = config.body_color
        self.rotmat = self.rotmat2d()
        self.aabb = self.make_aabb()

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    @override
    def step(self, check_for_world_boundaries=None, world=None, check_for_agent_collisions=None) -> None:
        super().step()

        self.rotmat = self.rotmat2d()
        self.aabb = self.make_aabb()

    @property
    def is_poly(self):
        return self.points.any()

    def rotmat2d(self, offset=0):
        angle = self.angle + offset
        return np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]], dtype=np.float64)

    def rotmat3d(self, offset=None):
        angle = self.angle if offset is None else (self.angle + offset)
        return np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ], dtype=np.float64)

    @property
    def poly_rotated(self):
        return self.points @ self.rotmat2d()

    @override
    def draw(self, screen, offset=((0, 0), 1.0)) -> None:
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen)

        # Draw Cell Membrane
        filled = 0 if (self.is_highlighted or self.stopped_duration or self.body_filled) else 1
        color = self.body_color if not self.stopped_duration else (255, 255, 51)
        pos = np.asarray(self.getPosition()) * zoom + pan

        if self.is_poly:
            pygame.draw.polygon(screen, color, self.poly_rotated * zoom + pos, width=filled)
        else:
            pygame.draw.circle(screen, color, (*pos,), self.radius * zoom, width=filled)  # pyright: ignore[reportArgumentType]

        self.draw_direction(screen, offset)

        if self.DEBUG:
            self.debug_draw(screen, offset)

    def draw_direction(self, screen, offset=((0, 0), 1.0)):
        pan, zoom = np.asarray(offset[0]), offset[1]
        # "Front" direction vector
        pos = np.asarray(self.getPosition()) * zoom + pan
        head = np.asarray(self.getFrontalPoint()) * zoom + pan
        tail = pos
        vec = head - tail
        mag = self.radius * 2
        vec_with_magnitude = tail + vec * mag
        pygame.draw.line(screen, (255, 255, 255), tail, vec_with_magnitude)

    def build_collider(self):
        if self.is_poly:
            return PolyCollider(self.poly_rotated + self.pos)
        return CircularCollider(*self.pos, self.radius)

    def debug_draw(self, screen, offset):
        self.make_aabb().draw(screen, offset)

    def make_aabb(self) -> AABB:
        """
        Return the Bounding Box of the agent
        """
        if self.is_poly:
            return AABB(self.poly_rotated + self.pos)
        x, y = self.pos
        top_left = (x - self.radius, y - self.radius)
        bottom_right = (x + self.radius, y + self.radius)
        return AABB((top_left, bottom_right))

    def get_simple_poly_radius(self):
        if self.is_poly:
            return max(np.linalg.norm(p) for p in self.points)

    def __str__(self) -> str:
        x, y = self.pos
        return f"(x: {x}, y: {y}, r: {self.radius}, θ: {self.angle})"
