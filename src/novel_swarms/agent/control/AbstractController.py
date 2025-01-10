from enum import Enum
from typing import override

ControllerType = Enum("ControllerType", ["method_based", "list_based", "inherit_agent"])


class AbstractController:
    """
    Given agent observations, return agent actions
    """

    def __init__(self, parent, agent=None):  # type:ignore[reportMissingSuperCall]
        self.parent = parent
        self.agent = self.parent if agent is None else agent

    def get_actions(self, agent):
        pass

    def as_config_dict(self):
        return {}
