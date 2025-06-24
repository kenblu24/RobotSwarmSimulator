import numpy as np

from .AbstractController import AbstractController

ConstantOutputValues = tuple[float, ...] | np.ndarray
TwoConstantOutputValues = tuple[ConstantOutputValues, ConstantOutputValues] | np.ndarray


class BinaryController(AbstractController):
    def __init__(self, a: TwoConstantOutputValues | ConstantOutputValues, b: ConstantOutputValues | None = None,
                 agent=None, parent=None, sensor_id=0, **kwargs):
        super().__init__(agent=agent, parent=parent, **kwargs)
        self.sensor_id = sensor_id  # use this to determine which sensor on the agent to use

        # set self.a and self.b, the two sets of constant output values.
        a = np.asarray(a, dtype='float64')
        if len(a.shape) > 2:
            raise ValueError("Expected first argument to be a 1D or 2D array")
        if a.shape[0] == 2 and b is None:
            self.a, self.b = a
        elif len(a.shape) == 1:
            if b is None and len(a) % 2 == 0:
                self.a = a[len(a) // 2:]
                self.b = a[:len(a) // 2]
            else:
                b = np.asarray(b, dtype='float64')
                if len(b.shape) != 1 or b.shape[0] != a.shape[0]:
                    raise ValueError("Expected constant output values to be 1D arrays of same size")
                self.a, self.b = a, b
        else:
            raise ValueError("Expected argument(s) to be 1D arrays of same length or single 2D array")

        # print(self.a, self.b)

    def get_actions(self, agent):
        """
        An example of a "from scratch" controller that you can code with any information contained within the agent class
        """
        other_agent_detected = bool(agent.agent_in_sight)  # Whether the agent detects another agent
        if hasattr(agent, "sensing_avg") and other_agent_detected is not None:
            other_agent_detected = agent.sensing_avg(other_agent_detected)
        wall_detected = not other_agent_detected and agent.sensors[self.sensor_id].current_state == 1

        if agent.goal_seen:
            return np.zeros(self.a.shape)

        if other_agent_detected or wall_detected:
            return self.b
        else:
            return self.a

    def as_config_dict(self):
        return {'a': self.a, 'b': self.b, 'sensor_id': self.sensor_id}
