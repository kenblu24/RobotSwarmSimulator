from typing import Literal

import numpy as np

from .AbstractController import AbstractController
from ...sensors.BinaryFOVSensor import BinaryFOVSensor


ConstantOutputValues = tuple[float, ...] | np.ndarray
LinearAngularVel = tuple[float, float]

# TODO:
# - [ ] Have multiple sensor ids
#   - `BinaryController` maintains an id it will use in `get_actions()`

# PERSONAL NOTE:
#   - Sensor id is just an index into list of sensors found in the agent's sensor

# `NaryController` NOTE:
#   - As of right now, only a BinaryFOVSensor is supported

class NaryController(AbstractController):
    def __init__(self, on_nothing: LinearAngularVel, on_detect: list[LinearAngularVel],
        agent=None, parent=None, **kwargs
    ) -> None:
        super().__init__(agent=agent, parent=parent, **kwargs)
        # use this to determine which sensor on the agent to use
        self.sensor_ids: list[int] = [i for i in range(len(on_detect))]
        self.on_nothing: LinearAngularVel = on_nothing
        self.on_detect: list[LinearAngularVel] = on_detect

    def get_actions(self, agent) -> LinearAngularVel:
        for index in self.sensor_ids:
            sensor = agent.sensors[index]
            if sensor.current_state == 1:
                return self.on_detect[index]

        return self.on_nothing

    def as_config_dict(self):
        return {
            "on_nothing": self.on_nothing,
            "on_detect": self.on_detect,
            "sensor_ids": self.sensor_ids
        }
