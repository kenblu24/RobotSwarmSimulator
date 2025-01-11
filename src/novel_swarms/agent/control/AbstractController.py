from enum import Enum
from typing import override

ControllerType = Enum("ControllerType", ["method_based", "list_based", "inherit_agent"])


class AbstractController:
    """
    Given agent observations, return agent actions
    """

    def __init__(self, agent, parent=None):  # type:ignore[reportMissingSuperCall]
        self.agent = agent
        self.parent = self.agent if parent is None else parent

    def get_actions(self, agent):
        pass

    def as_config_dict(self):
        return {}
