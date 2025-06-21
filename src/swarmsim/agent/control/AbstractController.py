"""Abstract Controller class.

.. autoclass:: AbstractController
    :members:
    :undoc-members:

.. autodata:: ControllerType

"""

from enum import Enum
from typing import override

ControllerType = Enum("ControllerType", ["method_based", "list_based", "inherit_agent"])


class AbstractController:
    """Given agent observations, return agent actions

    Controllers can optionally take in sensor data and
    then return an action for the agent to take, usually in
    the form of a requested movement vector.
    """

    def __init__(self, agent, parent=None):  # type:ignore[reportMissingSuperCall]
        self.parent = None
        self.set_agent(agent, parent)

    def set_parent(self, parent=None):
        self.parent = self.agent if parent is None else parent

    def set_agent(self, agent, parent=None):
        self.agent = agent
        if parent is not None:  # if user specifies a parent, use that
            self.parent = parent
        elif self.parent is None or parent is ...:
            self.parent = agent

    def get_actions(self, agent):
        pass

    def as_config_dict(self):
        return {}
