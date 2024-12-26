from dataclasses import dataclass, field
from ...config import filter_unexpected_fields, associated_type

from ...agent.StaticAgent import StaticAgent, StaticAgentConfig

# typing
from typing import Any, override


@associated_type("StaticObject")
@filter_unexpected_fields
@dataclass
class StaticObjectConfig(StaticAgentConfig):
    collides: bool | int = True
    grounded: bool = True


class StaticObject(StaticAgent):
    @override
    def step(self, check_for_world_boundaries=None, world=None, check_for_agent_collisions=None) -> None:
        pass

    @override
    def draw_direction(self, screen, offset=((0, 0), 1.0)):
        pass