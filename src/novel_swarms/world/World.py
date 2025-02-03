import os

import pygame
import numpy as np

from ..gui.abstractGUI import AbstractGUI
from ..config.OutputTensorConfig import OutputTensorConfig
from ..config import store, filter_unexpected_fields, get_class_from_dict, get_agent_class, _ERRMSG_MISSING_ASSOCIATED_TYPE

import inspect
from dataclasses import dataclass, field, replace
from ..util.asdict import asdict
from collections.abc import Callable

from ..agent.Agent import Agent
from .spawners.Spawner import Spawner
from ..metrics.AbstractMetric import AbstractMetric


@filter_unexpected_fields
@dataclass
class AbstractWorldConfig:
    size: tuple[float, ...] | np.ndarray = (0, 0)
    metrics: list = field(default_factory=list)
    agents: list = field(default_factory=list)
    spawners: list = field(default_factory=list)
    objects: list = field(default_factory=list)
    goals: list = field(default_factory=list)
    stop_at: int | Callable | None = None
    background_color: tuple[int, int, int] = (0, 0, 0)
    seed: int | None = None
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.agents is None:
            self.agents = []
        if self.spawners is None:
            self.spawners = []

    @property
    def radius(self):
        self.size = np.asarray(self.size)
        return np.linalg.norm(self.size / 2)

    def as_dict(self):
        return self.asdict()

    def asdict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, env):
        return cls(**env)

    @classmethod
    def from_yaml(cls, path):
        from .. import yaml

        with open(path, "r") as f:
            return cls.from_dict(yaml.load(f))

    def save_yaml(self, path):
        import yaml

        with open(path, "w") as f:
            yaml.dump(self.as_dict(), f)

    # def addAgentConfig(self, agent_config):
    #     self.agentConfig = agent_config
    #     if self.agentConfig:
    #         self.agentConfig.attach_world_config(self.shallow_copy())

    # def shallow_copy(self):
    #     return RectangularWorldConfig(
    #         size=self.size,
    #         n_agents=self.population_size,
    #         seed=self.seed,
    #         init_type=self.init_type.getShallowCopy(),
    #         padding=self.padding,
    #         goals=self.goals,
    #         objects=self.objects
    #     )

    # def getDeepCopy(self):
    #     return self.from_dict(self.as_dict())

    # def set_attributes(self, dictionary):
    #     for key in dictionary:
    #         setattr(self, key, dictionary[key])

    # def factor_zoom(self, zoom):
    #     self.size = np.asarray(self.size) * zoom
    #     self.size *= zoom
    #     for goal in self.goals:
    #         goal.center[0] *= zoom
    #         goal.center[1] *= zoom
    #         goal.r *= zoom
    #         goal.range *= zoom
    #     # self.init_type.rescale(zoom)


class World:
    def __init__(self, config):
        self.config = config
        config = replace(config)
        self.population = []
        self.spawners = []
        self.metrics = []
        self.objects = []
        self.goals = config.goals
        self.meta = config.metadata
        self.gui = None
        self.total_steps = 0
        self.initialized = False
        self._screen_cache = None
        self.seed = config.seed
        self.set_seed(self.seed)
        self.rng: np.random.Generator

    def set_seed(self, seed):
        self.seed = np.random.randint(0, 2**31) if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        return self.seed

    def setup(self):
        # create agents, spawners, behaviors, objects, goals
        if self.initialized:
            return
        self.initialized = True

        # create agents
        for agent_config in self.config.agents:
            if isinstance(agent_config, Agent):  # if it's already an agent, just add it
                self.population.append(agent_config)
            else:  # otherwise, it's a config dict. find the class specified and create the agent
                agent_class, agent_config = get_agent_class(agent_config)
                self.population.append(agent_class.from_config(agent_config, self))

        for spawner_config in self.config.spawners:
            if isinstance(spawner_config, Spawner):  # if it's already a spawner, just add it
                self.spawners.append(spawner_config)
            else:  # otherwise, it's a config dict. find the class specified and create the spawner
                spawner_class, spawner_config = get_class_from_dict('spawners', spawner_config)
                self.spawners.append(spawner_class(self, **spawner_config))

        for metric_config in self.config.metrics:
            if isinstance(metric_config, AbstractMetric):  # if it's already a metric, just add it
                self.metrics.append(metric_config)
            else:  # otherwise, it's a config dict. find the class specified and create the metric
                metric_class, metric_config = get_class_from_dict('metrics', metric_config)
                self.metrics.append(metric_class(self, **metric_config))

        for b in self.metrics:
            b.reset()
            b.attach_world(self)

        # self.metrics = config.metrics
        # self.objects = config.objects
        # self.goals = config.goals
        # self.seed = config.seed

    def step(self):
        self.total_steps += 1

    def draw(self, screen, offset=None):
        pass

    def handle_key_press(self, event):
        pass

    def attach_gui(self, gui: AbstractGUI):
        self.gui = gui

    def as_dict(self):
        pass

    def asdict(self):
        return self.as_dict()

    def as_config_dict(self):
        pass

    @classmethod
    def from_config(cls, c):
        return cls(c)

    def evaluate(self, steps: int, output_capture: OutputTensorConfig = None, screen=None):
        frame_markers = []
        output = None
        screen = None
        if output_capture is not None:
            if output_capture.total_frames * output_capture.step > steps:
                raise Exception(
                    "Error: You have indicated an output capture that is larger than the lifespan of the simulation."
                )
            start = steps - (output_capture.total_frames * output_capture.step)

            if output_capture.timeless:
                frame_markers = [start]
            else:
                frame_markers = [(start + (output_capture.step * i)) - 1 for i in range(output_capture.total_frames)]

            screen = output_capture.screen
        for step in range(steps):
            self.step()

            if output_capture and output_capture.screen:
                # If start of recording, clear screen
                if frame_markers and step == frame_markers[0]:
                    screen.fill(output_capture.background_color)
                    pygame.display.flip()

                if not output_capture or not output_capture.timeless:
                    screen.fill(output_capture.background_color)

                if frame_markers and step > frame_markers[0]:
                    self.draw(screen)
                    pygame.display.flip()

            if output_capture:
                if not output_capture.timeless and step in frame_markers:
                    if output_capture.colored:
                        screen_capture = pygame.surfarray.array3d(screen)
                    else:
                        screen_capture = pygame.surfarray.array2d(screen)
                    if output is None:
                        output = np.array([screen_capture])
                    else:
                        output = np.concatenate((output, [screen_capture]))

        if output_capture and output_capture.timeless:
            if output_capture.colored:
                output = pygame.surfarray.array3d(screen)
            else:
                output = pygame.surfarray.array2d(screen)

        return output


def World_from_config(config: dict):
    world_types = store.world_types

    if isinstance(config, dict):
        if not config.get("associated_type", None):
            raise Exception(_ERRMSG_MISSING_ASSOCIATED_TYPE)
        if config["associated_type"] in world_types:
            world_cls = world_types[config["type"]]
        else:
            msg = f"Unknown world type: {config['associated_type']}"
            raise Exception(msg)
    else:
        if not hasattr(config, "associated_type"):
            raise Exception(_ERRMSG_MISSING_ASSOCIATED_TYPE)
        if config.associated_type not in world_types:
            msg = f"Unknown world type: {config.associated_type}"
            raise Exception(msg)
        world_cls = world_types[config.associated_type]
    if isinstance(world_cls, (list, tuple)):
        world_cls, _config_cls = world_cls
    if hasattr(world_cls, "from_config"):
        return world_cls.from_config(config)
    else:
        if not isinstance(config, dict):
            config = config.as_dict()
        return world_cls.from_dict(config)
