import math
from dataclasses import dataclass, field
import numpy as np

from ..config import store, filter_unexpected_fields
from ..agent.control.StaticController import zero_controller

# typing
from typing import Any


@filter_unexpected_fields
@dataclass
class BaseAgentConfig:
    position: tuple[float, ...] | np.ndarray = (0, 0),
    angle: float | Any = 0.,
    name: str | Any = None,
    controller: Any = None,
    sensors: list[Any] = field(default_factory=list),

    # def __post_init__(self):
    #     if self.stop_at_goal is not False:
    #         raise NotImplementedError  # not tested

    def as_dict(self):
        return self.asdict()

    def as_config_dict(self):
        return self.asdict()

    def asdict(self):
        return dict(self.as_generator())

    @classmethod
    def from_dict(cls, env):
        return cls(**env)

    def __badvars__(self):
        return []

    def as_generator(self):
        for key, value in self.__dict__.items():
            if any(key == bad for bad in self.__badvars__()):
                continue
            if hasattr(value, "asdict"):
                yield key, value.asdict()
            elif hasattr(value, "as_dict"):
                yield key, value.as_dict()
            elif hasattr(value, "as_config_dict"):
                yield key, value.as_config_dict()
            else:
                yield key, value


class Agent:

    def __init__(self, config, name=None, group=0, initialize=True) -> None:
        self.config = config
        self.pos = np.asarray(config.position)
        self.name = name or config.name
        self.dpos = np.zeros(len(self.pos))
        self.angle = config.angle
        self.dtheta = 0
        self.sensors = []
        self.controller = zero_controller(2)
        self.collision_flag = False
        self.stop_on_collision = False
        self.stopped_duration = 0
        self.detection_id = 0
        self.aabb = None
        self.group = group

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    def setup_controller_from_config(self):
        if not (isinstance(self.config.controller, dict) and "type" in self.config.controller):
            return
        controller_config = self.config.controller.copy()
        controller_type = controller_config.pop("type")
        if controller_type not in store.controllers:
            raise Exception(f"Unknown controller type: {controller_type}")
        self.controller = store.controllers[controller_type](**controller_config)

    def setup_sensors_from_config(self):
        for sensor_config in self.config.sensors:
            if not (isinstance(self.config.sensor, dict) and "type" in self.config.sensor):
                raise TypeError(f"Expected a sensor config dict, got {sensor_config}")
            sensor_config = self.config.sensor.copy()
            sensor_type = sensor_config.pop("type")
            if sensor_type not in store.sensor_types:
                raise Exception(f"Unknown sensor type: {sensor_type}")
            self.sensors.append(store.sensors[sensor_type](**sensor_config))

    def step(self, check_for_world_boundaries=None) -> None:
        self.pos = np.asarray(self.pos, dtype='float64')

    def draw(self, screen, offset=((0, 0), 1.0)) -> None:
        # TODO: Implement offset/zoom
        pass

    def get_sensors(self):
        return self.sensors

    def getPosition(self):
        return np.asarray(self.pos, dtype='float64')

    def getVelocity(self):
        return np.asarray(self.dpos, dtype='float64')

    def getFrontalPoint(self) -> tuple:
        """
        Returns the location on the circumference that represents the "front" of the robot
        """
        return self.get_x_pos() + math.cos(self.get_heading()), self.get_y_pos() + math.sin(self.get_heading())

    def attach_agent_to_sensors(self):
        for sensor in self.sensors:
            sensor.parent = self

    def get_aabb(self):
        pass

    def get_x_pos(self):
        return self.pos[0]

    def get_y_pos(self):
        return self.pos[1]

    def set_x_pos(self, new_x):
        self.pos[0] = new_x

    def set_y_pos(self, new_y):
        self.pos[1] = new_y

    def get_heading(self):
        return self.angle

    def set_heading(self, new_heading):
        self.angle = new_heading

    def on_key_press(self, event):
        pass

    def get_name(self):
        return self.name

    def set_name(self, new_name):
        self.name = new_name

    def set_pos_vec(self, vec):
        """
        Set the x, y, and angle of the agent.

        @params
        vec: An iterable of len 3, where vec[0] is the x_position, vec[1] is the y_position, and vec[2] is the angle heading

        @return None
        """
        self.set_x_pos(vec[0])
        self.set_y_pos(vec[1])
        self.set_heading(vec[2])

    @classmethod
    def from_config(cls, config):
        return cls(config)
