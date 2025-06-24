from .Controller import Controller


class HomogeneousController(Controller):
    def __init__(self, agent=None, parent=None, genome=None, sensor_idx=0):
        super().__init__(agent=agent, parent=parent, controller=self.control_method)
        self.list_based = False
        self.controller_as_method = self.control_method
        self.genome = [.0, .0, .0, .0] if genome is None else genome
        self.sensor_idx = sensor_idx

    def control_method(self, agent):
        """
        An example of a "from scratch" controller that you can code with any information contained within the agent class
        """
        sigma = agent.goal_seen  # Whether the goal has been detected previously by this agent
        gamma = agent.agent_in_sight is not None  # Whether the agent detects another agent
        if hasattr(agent, "sensing_avg"):
            gamma = agent.sensing_avg(gamma)
        wall_detected = not gamma and agent.sensors[self.sensor_idx].current_state == 1

        u_1, u_2 = 0.0, 0.0  # Set these by default
        if not sigma:
            if not gamma and not wall_detected:
                u_1, u_2 = self.genome[0], self.genome[1]  # u_1 in pixels/second (see b2p func), u_2 in rad/s
            else:
                u_1, u_2 = self.genome[2], self.genome[3]  # u_1 in pixels/second (see b2p func), u_2 in rad/s
        else:
            u_1, u_2 = 0.0, 0.0  # u_1 in pixels/second (see b2p func), u_2 in rad/s
        return u_1, u_2

    def as_config_dict(self):
        return {'genome': self.genome}
