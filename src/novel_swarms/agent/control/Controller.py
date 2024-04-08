from enum import Enum
from typing import override

ControllerType = Enum("ControllerType", ["method_based", "list_based", "inherit_agent"])


class Controller:
    """
    Given agent observations, return agent actions
    """

    def __init__(self, controller="self"):  # type:ignore[reportMissingSuperCall]
        """
        Controllers can take three forms
        First, a list of values where n states are mapped to n * k outputs, where k is the number of output values per state
        Second, a function, that takes an agent as an argument, and returns the appropriate k values based on the agent info
        Third, 'self', which redirects the request to the get_actions method of the agent (if available)
        """

        self.type = None

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
        # Neither --> Error
        else:
            raise Exception("The input value of controller to class Controller must be a callable, list, or 'self'!")

    def get_actions(self, agent):
        if self.type == ControllerType.list_based:
            sensor_state = agent.get_sensors().getState()
            # e1, e2 = self.controller_as_list[slice(2, 3) if sensor_state else slice(0, 1)]
            e1 = self.controller_as_list[sensor_state * 2]
            e2 = self.controller_as_list[(sensor_state * 2) + 1]
            return e1, e2
        elif self.type == ControllerType.inherit_agent:
            return agent.get_actions()
        else:
            return self.controller_as_method(agent)

    @override
    def __str__(self):
        if self.type == ControllerType.list_based:
            return ""
        elif self.type == ControllerType.inherit_agent:
            return "get_actions() on Agent"
        else:
            return repr(self.controller_as_method)

    @staticmethod
    def homogeneous_from_genome(genome):
        def custom_controller(agent):
            """
            An example of a "from scratch" controller that you can code with any information contained within the agent class
            """
            sigma = agent.goal_seen  # Whether the goal has been detected previously by this agent
            gamma = agent.agent_in_sight is not None  # Whether the agent detects another agent

            u_1, u_2 = 0.0, 0.0  # Set these by default
            if not sigma:
                if not gamma:
                    u_1, u_2 = genome[0], genome[1]  # u_1 in pixels/second (see b2p func), u_2 in rad/s
                else:
                    u_1, u_2 = genome[2], genome[3]  # u_1 in pixels/second (see b2p func), u_2 in rad/s
            else:
                u_1, u_2 = 0.0, 0.0  # u_1 in pixels/second (see b2p func), u_2 in rad/s
            return u_1, u_2
        return custom_controller
