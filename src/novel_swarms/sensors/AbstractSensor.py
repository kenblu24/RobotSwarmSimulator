from ..agent.Agent import Agent
import numpy as np


class AbstractSensor:
    config_vars = ['static_position', 'n_possible_states', 'show']

    def __init__(self, agent, parent=None, static_position=None, n_possible_states=0, draw=True):
        """
        Initialize the abstract class.
            Sensors should typically have a parent that is assigned to them that must be of subclass 'Agent'
            If a parent is not included, a static position is accepted in the form (x, y)
        """
        if agent is not None and not issubclass(type(agent), Agent):
            raise Exception("The parent must be of type Agent")

        self.agent = agent
        self.parent = self.agent if parent is None else parent

        self.show = draw
        self.static_position = np.asarray(static_position) if static_position is not None else None
        self.n_possible_states = n_possible_states
        self.current_state = 0
        self.detection_id = 0
        self.goal_detected = False

    def step(self, world):
        if self.agent is None and self.static_position is None:
            raise Exception("Either a parent of type 'Agent' must be provided or a static position in the form (x, y)")
        pass

    @property
    def position(self):
        return self.agent.position

    def draw(self, screen, offset=((0, 0), 1.0)):
        pass

    def get_state(self):
        return self.current_state

    def state_pretty(self):
        return str(self.get_state())

    def as_config_dict(self):
        return {k: self.__dict__[k] for k in self.config_vars}
