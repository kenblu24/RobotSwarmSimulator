from functools import cached_property

import numpy as np

from swarmsim.agent.control.AbstractController import AbstractController

from swarmsim.util.pid import PID


class RunnerController(AbstractController):
    """
    Given agent observations, return agent actions
    """

    def __init__(self, agent=None, parent=None, target_name="goal"):  # type:ignore[reportMissingSuperCall]
        super().__init__(agent=agent, parent=parent)

        self.target_name = target_name.lower() if isinstance(target_name, str) else target_name

        self.tracking_pid = PID(p=1.0, i=0.01, d=0.0)

    def get_actions(self, agent):
        v, omega = 0, 0
        goal_position = np.asarray(self.goal_object.pos)

        dist_to_goal = np.linalg.norm(agent.pos - goal_position)
        radians_to_goal = np.arctan2(goal_position[1] - agent.pos[1], goal_position[0] - agent.pos[0]) - agent.angle
        if dist_to_goal > agent.radius:
            v = 0.03 # m/s
            omega = self.tracking_pid(np.clip(radians_to_goal, -2, 2))


        return v, omega  # return the agent's desired velocity and omega here

    @cached_property  # find the goal object and cache the result
    def goal_object(self):
        for obj in self.agent.world.objects:
            if isinstance(obj.name, str) and obj.name.lower() == self.target_name:
                return obj
        raise Exception("Controller could not find an object named 'goal' in world.objects")

    # @override
    # def __str__(self):
    #     return "RunnerController"

    # def as_config_dict(self):
    #     return {'controller': self._config_controller}
