import random
import os
from pygame import math
from dataclasses import dataclass, fields

from .WorldConfig import RectangularWorldConfig
from ..sensors.SensorSet import SensorSet
from ..sensors.SensorFactory import SensorFactory
from ..agent.MazeAgent import MazeAgent

# typing
from typing import Any, override

from ..world.World import World
from ..agent.control.Controller import Controller


def filter_unexpected_fields(cls):
    original_init = cls.__init__

    def new_init(self, *args, **kwargs):
        expected_fields = {field.name for field in fields(cls)}
        cleaned_kwargs = {key: value for key, value in kwargs.items() if key in expected_fields}
        unexpected_kwargs = {key: value for key, value in kwargs.items() if key not in expected_fields}
        for key, value in unexpected_kwargs.items():
            setattr(self, key, value)
        original_init(self, *args, **cleaned_kwargs)

    cls.__init__ = new_init
    return cls


@dataclass
class BaseAgentConfig:

    def __post_init__(self):
        if self.stop_at_goal is not False:
            raise NotImplementedError  # not tested

    def as_dict(self):
        return self.asdict()

    def as_config_dict(self):
        return self.asdict()

    def asdict(self):
        return dict(self.as_generator())

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


class DiffDriveAgentConfig:
    def __init__(self,
                 x=None,
                 y=None,
                 controller=None,
                 angle=None,
                 world_config: RectangularWorldConfig = None,
                 seed=None,
                 agent_radius=5,
                 wheel_radius=2.0,
                 dt=1.0,
                 sensors: SensorSet = None,
                 trace_length=None,
                 trace_color=None,
                 body_color=(255, 255, 255),
                 body_filled=False
                 ):
        self.x = x
        self.y = y
        self.angle = angle
        self.world = world_config
        self.seed = seed
        self.dt = dt
        self.agent_radius = agent_radius
        self.wheel_radius = wheel_radius
        self.controller = controller
        self.sensors = sensors
        self.trace_length = trace_length
        self.trace_color = trace_color
        self.body_color = body_color
        self.body_filled = body_filled

    def attach_world_config(self, world_config):
        self.world = world_config

    def as_dict(self):
        return {
            "type": "DiffDriveAgentConfig",
            "x": self.x,
            "y": self.y,
            "angle": self.angle,
            "seed": self.seed,
            "dt": self.dt,
            "agent_radius": self.agent_radius,
            "wheel_radius": self.wheel_radius,
            "body_color": list(self.body_color),
            "body_filled": self.body_filled,
            "controller": self.controller,
            "sensors": self.sensors.as_config_dict(),
            "trace_length": self.trace_length,
            "trace_color": self.trace_color,
        }

    @staticmethod
    def from_dict(d):
        ret = DiffDriveAgentConfig()
        for k, v in d.items():
            if k != "type":
                setattr(ret, k, v)
        ret.sensors = SensorFactory.create(ret.sensors)
        return ret


class DroneAgentConfig:
    def __init__(self,
                 x=None,
                 y=None,
                 controller=None,
                 angle=None,
                 world_config: RectangularWorldConfig = None,
                 seed=None,
                 agent_radius=5,
                 dt=1.0,
                 sensors: SensorSet = None,
                 trace_length=None,
                 trace_color=None,
                 body_color=(255, 255, 255),
                 body_filled=False
                 ):
        self.x = x
        self.y = y
        self.angle = angle
        self.world = world_config
        self.seed = seed
        self.dt = dt
        self.agent_radius = agent_radius
        self.controller = controller
        self.sensors = sensors
        self.trace_length = trace_length
        self.trace_color = trace_color
        self.body_color = body_color
        self.body_filled = body_filled

    def attach_world_config(self, world_config):
        self.world = world_config

    def as_dict(self):
        return {
            "type": "DroneAgentConfig",
            "x": self.x,
            "y": self.y,
            "angle": self.angle,
            "seed": self.seed,
            "dt": self.dt,
            "agent_radius": self.agent_radius,
            "body_color": list(self.body_color),
            "body_filled": self.body_filled,
            "controller": self.controller,
            "sensors": self.sensors.as_config_dict(),
            "trace_length": self.trace_length,
            "trace_color": self.trace_color,
        }

    @staticmethod
    def from_dict(d):
        ret = DroneAgentConfig()
        for k, v in d.items():
            if k != "type":
                setattr(ret, k, v)
        ret.sensors = SensorFactory.create(ret.sensors)
        return ret


class UnicycleAgentConfig:
    def __init__(self,
                 x=None,
                 y=None,
                 controller=None,
                 angle=None,
                 world_config: RectangularWorldConfig = None,
                 seed=None,
                 agent_radius=5,
                 wheel_radius=2.0,
                 dt=1.0,
                 sensors: SensorSet = None,
                 idiosyncrasies=False,
                 stop_on_collide=False,
                 body_color=(255, 255, 255),
                 body_filled=False,
                 trace_length=None,
                 trace_color=None,
                 ):
        self.x = x
        self.y = y
        self.angle = angle
        self.world = world_config
        self.seed = seed
        self.dt = dt
        self.agent_radius = agent_radius
        self.controller = controller
        self.sensors = sensors
        self.idiosyncrasies = idiosyncrasies
        self.stop_on_collision = stop_on_collide
        self.body_color = body_color
        self.body_filled = body_filled
        self.trace_length = trace_length
        self.trace_color = trace_color

    def attach_world_config(self, world_config):
        self.world = world_config

    def as_dict(self):
        return {
            "type": "UnicycleAgentConfig",
            "x": self.x,
            "y": self.y,
            "angle": self.angle,
            "seed": self.seed,
            "dt": self.dt,
            "agent_radius": self.agent_radius,
            "body_color": list(self.body_color),
            "body_filled": self.body_filled,
            "controller": self.controller,
            "sensors": self.sensors.as_config_dict(),
            "idiosyncrasies": self.idiosyncrasies,
            "stop_on_collision": self.stop_on_collision,
            "trace_length": self.trace_length,
            "trace_color": self.trace_color,
        }

    @staticmethod
    def from_dict(d):
        ret = UnicycleAgentConfig()
        for k, v in d.items():
            if k != "type":
                setattr(ret, k, v)
        ret.sensors = SensorFactory.create(ret.sensors)
        return ret


class LevyAgentConfig:
    def __init__(self, **kwargs):
        self.x = kwargs["config"].x
        self.y = kwargs["config"].y
        self.agent_radius = kwargs["config"].agent_radius
        self.unicycle_config = kwargs["config"]
        self.world = kwargs.get("world_config", None)
        self.levy_constant = kwargs.get("levy_constant", 1.0)
        self.forward_rate = kwargs["forward_rate"]
        self.turning_rate = kwargs["turning_rate"]
        self.step_scale = kwargs.get("step_scale", 1.0)
        self.mode_max_time = kwargs.get("mode_max_time", 500)

        self.seed = kwargs.get("seed", None)
        self.body_color = kwargs.get("body_color", (255, 255, 255))
        self.body_filled = kwargs.get("body_filled", True)
        self.stop_at_goal = kwargs.get("stop_at_goal", False)
        self.stop_on_goal_detect = kwargs.get("stop_on_goal_detect", False)
        self.curve_based = kwargs.get("curve_based", True)

    def attach_world_config(self, world_config):
        self.world = world_config
        self.unicycle_config.attach_world_config(world_config)

    def as_dict(self):
        return {
            "type": "LevyAgentConfig",
            "x": self.x,
            "y": self.y,
            "seed": self.seed,
            "agent_radius": self.agent_radius,
            "config": self.unicycle_config.as_dict(),
            "levy_constant": self.levy_constant,
            "forward_rate": self.forward_rate,
            "turning_rate": self.turning_rate,
            "step_scale": self.step_scale,
            "body_color": list(self.body_color),
            "body_filled": self.body_filled,
            "stop_on_goal" : self.stop_at_goal,
            "stop_on_goal_detect":self.stop_on_goal_detect
        }

    @staticmethod
    def from_dict(d):
        if isinstance(d["config"], dict):
            d["config"] = AgentConfigFactory.create(d["config"])
        return LevyAgentConfig(**d)

    def rescale(self, zoom):
        self.forward_rate *= zoom
        self.unicycle_config.rescale(zoom)


@filter_unexpected_fields
@dataclass
class MazeAgentConfig(BaseAgentConfig):
    x: float | None = None
    y: float | None = None
    angle: float | None = None
    world: World | None = None
    world_config: RectangularWorldConfig | None = None
    seed: Any = None
    agent_radius: float = 5
    dt: float = 1.0
    sensors: SensorSet | None = None
    idiosyncrasies: Any = False
    delay: str | int | float = 0
    sensing_avg: int = 1
    stop_on_collision: bool = False
    stop_at_goal: bool = False
    body_color: tuple[int, int, int] = (255, 255, 255)
    body_filled: bool = False
    catastrophic_collisions: bool = False
    trace_length: tuple[int, int, int] | None = None
    trace_color: tuple[int, int, int] | None = None
    controller: Controller | None = None
    track_io: bool = False
    type: str = ""

    def __post_init__(self):
        super().__post_init__()
        if self.stop_at_goal is not False:
            raise NotImplementedError  # not tested

    @override
    def __badvars__(self):
        return super().__badvars__() + ["world", "world_config"]

    def attach_world_config(self, world_config):
        self.world = world_config

    @staticmethod
    def from_dict(d):
        if isinstance(d["sensors"], dict):
            d["sensors"] = SensorFactory.create(d["sensors"])
        return MazeAgentConfig(**d)

    def rescale(self, zoom):
        self.agent_radius *= zoom
        if self.sensors is not None:
            for s in self.sensors:
                s.r *= zoom
                s.goal_sensing_range *= zoom
                s.wall_sensing_range *= zoom

    def create(self, **kwargs):
        return MazeAgent(config=self, **kwargs)


class ModeSwitchingAgentConfig():
    def __init__(self,
                 configs=None,
                 switch_mode="Keyboard"  # Elem in ["Keyboard", "Time", "Distribution"]
                 ):

        self.world = configs[0].world
        self.configs = configs
        self.switch_mode = switch_mode

    def attach_world_config(self, world_config):
        self.world = world_config
        for config in self.configs:
            config.attach_world_config(world_config)

    def as_dict(self):
        raise NotImplementedError

    @staticmethod
    def from_dict(d):
        raise NotImplementedError


class StaticAgentConfig:
    def __init__(self,
                 x=None,
                 y=None,
                 angle=None,
                 world_config: RectangularWorldConfig = None,
                 seed=None,
                 agent_radius=5,
                 dt=1.0,
                 body_color=(255, 255, 255),
                 body_filled=False
                 ):
        self.x = x
        self.y = y
        self.angle = angle
        self.world = world_config
        self.seed = seed
        self.dt = dt
        self.agent_radius = agent_radius
        self.body_color = body_color
        self.body_filled = body_filled

    def attach_world_config(self, world_config):
        self.world = world_config

    def as_dict(self):
        return {
            "type": "StaticAgentConfig",
            "x": self.x,
            "y": self.y,
            "angle": self.angle,
            "seed": self.seed,
            "dt": self.dt,
            "agent_radius": self.agent_radius,
            "body_color": list(self.body_color),
            "body_filled": self.body_filled
        }

    @staticmethod
    def from_dict(d):
        ret = StaticAgentConfig()
        for k, v in d.items():
            if k != "type":
                setattr(ret, k, v)
        return ret


class AgentConfigFactory:
    @staticmethod
    def create(d):
        if d["type"] == "StaticAgentConfig":
            return StaticAgentConfig.from_dict(d)
        elif d["type"] == "ModeSwitchingAgentConfig":
            return ModeSwitchingAgentConfig.from_dict(d)
        elif d["type"] == "MazeAgentConfig":
            return MazeAgentConfig.from_dict(d)
        elif d["type"] == "LevyAgentConfig":
            return LevyAgentConfig.from_dict(d)
        elif d["type"] == "UnicycleAgentConfig":
            return UnicycleAgentConfig.from_dict(d)
        elif d["type"] == "DroneAgentConfig":
            return DroneAgentConfig.from_dict(d)
        elif d["type"] == "DiffDriveAgentConfig":
            return DiffDriveAgentConfig.from_dict(d)
        elif d["type"] == "HeterogeneousSwarmConfig":
            from ..config.HeterogenSwarmConfig import HeterogeneousSwarmConfig
            return HeterogeneousSwarmConfig.from_dict(d)
        else:
            raise Exception(f"Cannot instantiate a config of type: {d['type']}")

class AgentYAMLFactory:
    @staticmethod
    def from_yaml(file_name):
        import yaml
        config = None
        with open(file_name, "r") as stream:
            config = yaml.safe_load(stream)

        a_type = config["type"]
        if a_type == "Levy":
            base = AgentYAMLFactory.from_yaml(os.path.join(os.path.dirname(file_name), config["base"]))
            config["config"] = base
            return LevyAgentConfig(**config)
        elif a_type == "Goalbot":
            sensors = SensorFactory.create(config["sensors"])
            config["sensors"] = sensors
            return MazeAgentConfig(**config)
        else:
            raise Exception(f"Unknown Agent type: {config['type']}")


