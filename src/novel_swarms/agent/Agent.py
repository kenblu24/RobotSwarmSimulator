import math
import copy
from dataclasses import dataclass, field

import numpy as np

from ..agent.control.StaticController import zero_controller
from ..config import get_class_from_dict, filter_unexpected_fields

# typing
from typing import Any
from ..util.collider.AABB import AABB


@filter_unexpected_fields
@dataclass
class BaseAgentConfig:
    position: tuple[float, ...] | np.ndarray = (0, 0)
    angle: float | Any = 0.
    name: str | Any = None
    controller: Any = None
    grounded: bool = False
    collides: bool | int = False
    sensors: list = field(default_factory=list)
    team: str | None = None

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

    def __init__(self, config, world, name=None, group=0, initialize=True) -> None:
        self.marked_for_deletion = False
        self.config = config
        self.pos = np.asarray(config.position, dtype=np.float64)
        self.name = name or config.name
        self.dpos = np.zeros(len(self.pos))
        self.angle = config.angle
        self.dtheta = 0
        self.sensors = []
        self.controller = zero_controller(2)
        self.collision_flag = False
        self.collides = config.collides
        self.stop_on_collision = False
        self.stopped_duration = 0
        self.detection_id = 0
        self.aabb = None
        self.group = group
        self.world = world
        self.grounded = config.grounded
        self.team = config.team

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    def setup_controller_from_config(self):
        if not self.config.controller:
            return
        # if it's already a controller, just add it
        from ..agent.control.AbstractController import AbstractController
        if isinstance(self.config.controller, AbstractController):
            self.controller = copy.copy(self.config.controller)
            return
        # otherwise, it's a config dict. find the class specified and create the controller
        if not isinstance(self.config.controller, dict):
            msg = f'Tried to setup controller, but {repr(self.config.controller)} is not a dict or subclass of AbstractController'
            raise Exception(msg)
        res = get_class_from_dict('controller', self.config.controller)
        if not res:
            return
        controller_cls, controller_config = res
        self.controller = controller_cls(agent=self, **controller_config)

    def setup_sensors_from_config(self):
        from ..sensors.AbstractSensor import AbstractSensor
        for sensor_config in self.config.sensors:
            # if it's already a sensor, just add it
            if isinstance(sensor_config, AbstractSensor):
                self.sensors.append(copy.copy(sensor_config))
                continue
            # otherwise, it's a config dict. find the class specified and create the sensor
            sensor_cls, sensor_config = get_class_from_dict('sensors', sensor_config)
            self.sensors.append(sensor_cls(agent=self, **sensor_config))

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

    def orientation_uvec(self, offset=0):
        return np.array([
            math.cos(self.angle + offset),
            math.sin(self.angle + offset)
        ], dtype=np.float64)

    def getFrontalPoint(self, offset=0) -> tuple:
        """
        Returns the location on the circumference that represents the "front" of the robot
        """
        return self.pos + self.orientation_uvec(offset)

    def attach_agent_to_sensors(self):
        for sensor in self.sensors:
            sensor.parent = self

    def get_aabb(self) -> AABB:
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
    def from_config(cls, config, world):
        return cls(config, world)
