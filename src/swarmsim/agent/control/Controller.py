from enum import Enum
from typing import override

from .AbstractController import AbstractController

ControllerType = Enum("ControllerType", ["method_based", "list_based", "inherit_agent"])


class Controller(AbstractController):
    """
    Given agent observations, return agent actions
    """

    def __init__(self, agent=None, parent=None, controller="self"):  # type:ignore[reportMissingSuperCall]
        """
        Controllers can take three forms
        First, a list of values where n states are mapped to n * k outputs, where k is the number of output values per state
        Second, a function, that takes an agent as an argument, and returns the appropriate k values based on the agent info
        Third, 'self', which redirects the request to the get_actions method of the agent (if available)
        """

        super().__init__(agent=agent, parent=parent)

        self.type = None
        self._config_controller = controller

        # Case 1: Controller is a Python List
        if isinstance(controller, list):
            self.type = ControllerType.list_based
            self.controller_as_list = controller
        # Case 2: Controller is a Python Function
        elif callable(controller):
            self.type = ControllerType.method_based
            self.controller_as_method = controller
        # case 3: Agent has or inherits a "get_action" method with no additional arguments
        elif controller == "self":
            self.type = ControllerType.inherit_agent
        elif controller is None:
            self.type = None
        # Neither --> Error
        else:
            raise Exception("The input value of controller to class Controller must be a callable, list, or 'self'!")

    @override
    def get_actions(self, agent):
        if self.type == ControllerType.list_based:
            sensor_state = agent.sensors[0].current_state
            # e1, e2 = self.controller_as_list[slice(2, 3) if sensor_state else slice(0, 1)]
            e1 = self.controller_as_list[sensor_state * 2]
            e2 = self.controller_as_list[(sensor_state * 2) + 1]
            return e1, e2
        elif self.type == ControllerType.inherit_agent:
            return agent.get_actions()
        elif self.type == ControllerType.method_based:
            return self.controller_as_method(agent)
        else:
            return

    @override
    def __str__(self):
        if self.type == ControllerType.list_based:
            return ""
        elif self.type == ControllerType.inherit_agent:
            return "get_actions() on Agent"
        elif self.type == ControllerType.method_based:
            return repr(self.controller_as_method)
        else:
            return repr(self.type)

    @override
    def as_config_dict(self):
        return {'controller': self._config_controller}
