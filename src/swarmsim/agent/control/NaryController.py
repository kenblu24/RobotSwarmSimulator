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
        mode: Literal["first", "last", "average"] = "first",
        agent=None, parent=None, **kwargs
    ) -> None:
        super().__init__(agent=agent, parent=parent, **kwargs)
        # use this to determine which sensor on the agent to use
        self.n_sensors = len(on_detect)
        self.sensor_ids: list[int] = [i for i in range(self.n_sensors)]
        self.mode: Literal["first", "last", "average"] = mode
        self.on_nothing: LinearAngularVel = on_nothing
        self.on_detect: list[LinearAngularVel] = on_detect

    def get_actions(self, agent) -> LinearAngularVel:
        match self.mode:
            case "first":
                for index in self.sensor_ids:
                    sensor = agent.sensors[index]
                    if sensor.current_state == 1:
                        return self.on_detect[index]

                return self.on_nothing
            # end of case "first"

            case "last":
                for index in reversed(self.sensor_ids):
                    sensor = agent.sensors[index]
                    if sensor.current_state == 1:
                        return self.on_detect[index]

                return self.on_nothing
            # end of case "last"

            case "average":
                sum: LinearAngularVel = (0.0, 0.0)
                count: int = 0
                for index in reversed(self.sensor_ids):
                    sensor = agent.sensors[index]
                    if sensor.current_state == 1:
                        v, omega = self.on_detect[index]
                        sum = (sum[0] + v, sum[1] + omega)
                        count += 1;

                return (sum[0] / count, sum[1] / count) if count > 0 else self.on_nothing
            # end of case "average"

            case _:
                raise ValueError(f"Unknown n-ary controller mode: {self.mode}")

    def as_config_dict(self):
        return {
            "on_nothing": self.on_nothing,
            "on_detect": self.on_detect,
            "sensor_ids": self.sensor_ids
        }
