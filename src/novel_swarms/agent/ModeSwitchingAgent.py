from dataclasses import dataclass, field

import pygame
import numpy as np

from ..agent.Agent import Agent, BaseAgentConfig
from ..config import filter_unexpected_fields, associated_type
from ..config import get_agent_class

# typing
from typing import Any, override


@associated_type("SwitchingAgent")
@filter_unexpected_fields
@dataclass
class SwitchingAgentConfig(BaseAgentConfig):
    position: tuple[float, ...] | np.ndarray | None = None
    agents: list[Any] = field(default_factory=list)
    switch_on_key: str | None = "m"


class SwitchingAgent(Agent):
    def __init__(self, config: SwitchingAgentConfig, name=None):
        super().__init__(config, name)

        # Create agents from config
        self.agents = []
        for c in config.agents:
            raise NotImplementedError
            agent_type, agent_config = get_agent_class(c)
            self.agents.append(agent_type(agent_config))
            print(f"CONFIG: {c.as_dict()}")

        self.active_index = 0
        self.active = self.agents[self.active_index]
        self.switch_on_key = config.switch_on_key

    @override
    def step(self, world=None, check_for_world_boundaries=None, check_for_agent_collisions=None) -> None:
        self.active.step(check_for_world_boundaries=check_for_world_boundaries,
                         check_for_agent_collisions=check_for_agent_collisions, world=world)

    @override
    def on_key_press(self, event):
        if self.switch_on_key and event.type == pygame.KEYDOWN:
            if event.key == getattr(pygame, f'K_{self.switch_on_key}'):
                self.switch()

    def build_collider(self):
        return self.active.build_collider()

    def switch(self):
        old_agent = self.active
        new_agent = self.agents[self.active_index % len(self.agents)]

        # Copy Position, Orientation from old_agent to new_agent
        new_agent.set_x_pos(old_agent.get_x_pos())
        new_agent.set_y_pos(old_agent.get_y_pos())
        new_agent.set_heading(old_agent.get_heading())
        new_agent.name = old_agent.name

        self.active = new_agent
        self.active_index += 1

    @override
    def draw(self, screen) -> None:
        super().draw(screen)
        self.active.draw(screen)

    @override
    def get_aabb(self):
        super().get_aabb()
        return self.active.get_aabb()

    @override
    def get_x_pos(self):
        return self.active.get_x_pos()

    @override
    def get_y_pos(self):
        return self.active.get_y_pos()

    @override
    def set_x_pos(self, new_x):
        return self.active.set_x_pos(new_x)

    @override
    def set_y_pos(self, new_y):
        return self.active.set_y_pos(new_y)

    @override
    def get_heading(self):
        return self.active.get_heading()

    @override
    def set_heading(self, new_heading):
        self.active.set_heading(new_heading)

    @override
    def get_sensors(self):
        return self.active.get_sensors()

    @override
    def set_name(self, new_name, set_all=False):
        self.name = new_name
        if set_all:
            for a in self.agents:
                a.set_name(new_name)
