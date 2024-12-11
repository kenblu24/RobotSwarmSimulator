import pygame

from ..gui.abstractGUI import AbstractGUI
from ..config.OutputTensorConfig import OutputTensorConfig
import numpy as np

import inspect
from dataclasses import dataclass, field, asdict, replace
from collections.abc import Callable

@dataclass
class AbstractWorldConfig:
    size: tuple[float, ...] | np.ndarray = (0, 0)
    behavior: list | dict = field(default_factory=list)
    agents: list | dict = field(default_factory=list)
    spawners: list | dict = field(default_factory=list)
    objects: list | dict = field(default_factory=list)
    goals: list | dict = field(default_factory=list)
    stop_at: int | Callable | None = None
    background_color: tuple[int, int, int] = (0, 0, 0)
    seed: int | None = None
    metadata: dict = field(default_factory=dict)

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
        return cls(**{
            k: v for k, v in env.items()
            if k in inspect.signature(cls).parameters
        })

    @classmethod
    def from_yaml(cls, path):
        import yaml
        with open(path, "r") as f:
            return cls.from_dict(yaml.safe_load(f))

    def save_yaml(self, path):
        import yaml
        with open(path, "w") as f:
            yaml.dump(self.as_dict(), f)

    def addAgentConfig(self, agent_config):
        self.agentConfig = agent_config
        if self.agentConfig:
            self.agentConfig.attach_world_config(self.shallow_copy())

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
        self.population = config.agents
        self.spawners = config.spawners
        self.behavior = config.behavior
        self.objects = config.objects
        self.goals = config.goals
        self.seed = config.seed
        self.meta = config.metadata
        self.gui = None
        self.total_steps = 0

    def setup(self):
        pass

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

    def evaluate(self, steps: int, output_capture: OutputTensorConfig = None, screen=None):
        frame_markers = []
        output = None
        screen = None
        if output_capture is not None:
            if output_capture.total_frames * output_capture.step > steps:
                raise Exception("Error: You have indicated an output capture that is larger than the lifespan of the simulation.")
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

