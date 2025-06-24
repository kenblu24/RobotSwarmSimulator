from dataclasses import dataclass, field
from swarmsim.config import filter_unexpected_fields, associated_type

from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig

# typing
from typing import Any, override


@associated_type("RunnerAgent")
@filter_unexpected_fields
@dataclass
class RunnerAgentConfig(MazeAgentConfig):
    pass


class RunnerAgent(MazeAgent):
    # @override
    # def step(self, check_for_world_boundaries=None, world=None, check_for_agent_collisions=None) -> None:
    #     pass
    pass