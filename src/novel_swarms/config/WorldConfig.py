import inspect
from dataclasses import dataclass, field
from collections.abc import Callable

import numpy as np

# from novel_swarms.world.initialization.FixedInit import FixedInitialization
# from novel_swarms.world.initialization.AbstractInit import AbstractInitialization
# from novel_swarms.world.initialization.RandomInit import RectRandomInitialization


@dataclass
class RectangularWorldConfig:
    size: tuple[float, float] | np.ndarray = None
    # n_agents: int = 10
    # init_type = None
    # defined_start = False
    # population_size = None
    # padding: float = 0
    # agentConfig = None
    behavior: list | dict = field(default_factory=list)
    agents: list | dict = field(default_factory=list)
    spawners: list | dict = field(default_factory=list)
    objects: list | dict = field(default_factory=list)
    goals: list | dict = field(default_factory=list)
    show_walls: bool = True
    collide_walls: bool = True
    detectable_walls: bool = False
    stop_at: int | Callable | None = None
    background_color: tuple[int, int, int] = (0, 0, 0)
    metadata: dict = field(default_factory=dict)

    @property
    def radius(self):
        self.size = np.asarray(self.size)
        return np.linalg.norm(self.size / 2)

    @classmethod
    def from_dict(cls, env):
        return cls(**{k: v for k, v in env.items() if k in inspect.signature(cls).parameters})

    # def __init__(self, **kwargs):

    #     # # TODO: Deprecate Agent Initialization
    #     # if "agent_initialization" in kwargs:
    #     #     warnings.warn("The agent_initialization parameter (WorldConfig Class) will be deprecated in an upcoming version. Use Initialization Typing Instead (param init_type).")
    #     #     self.defined_start = True
    #     #     self.agent_init = kwargs.get("agent_initialization")
    #     #     if len(self.agent_init) != n_agents:
    #     #         raise Exception(f"Length of Predefined Starting Locations ({len(self.agent_init)}) must equal number of agents ({n_agents})")

    #     # elif init_type is None:
    #     #     init_type = RectRandomInitialization(num_agents=n_agents, bb=((0, 0), size))

    #     if self.agentConfig:
    #         self.agentConfig.attach_world_config(self.shallow_copy())

    def addAgentConfig(self, agent_config):
        self.agentConfig = agent_config
        if self.agentConfig:
            self.agentConfig.attach_world_config(self.shallow_copy())

    def shallow_copy(self):
        return RectangularWorldConfig(
            size=self.size,
            n_agents=self.population_size,
            seed=self.seed,
            init_type=self.init_type.getShallowCopy(),
            padding=self.padding,
            goals=self.goals,
            objects=self.objects,
        )

    def getDeepCopy(self):
        return self.from_dict(self.as_dict())

    def set_attributes(self, dictionary):
        for key in dictionary:
            setattr(self, key, dictionary[key])

    def factor_zoom(self, zoom):
        self.size = np.asarray(self.size) * zoom
        self.size *= zoom
        for goal in self.goals:
            goal.center[0] *= zoom
            goal.center[1] *= zoom
            goal.r *= zoom
            goal.range *= zoom
        # self.init_type.rescale(zoom)

    # def as_dict(self):
    #     return {
    #         "behavior": [b.as_config_dict() for b in self.behavior],
    #         "size": tuple(self.size),
    #         "population_size": self.population_size,
    #         "seed": self.seed,
    #         "padding": self.padding,
    #         # "agent_config": self.agentConfig.as_dict(),
    #         "show_walls": self.show_walls,
    #         "collide_walls": self.collide_walls,
    #         "detectable_walls" : self.detectable_walls,
    #         "stop_at": self.stop_at,
    #         "objects": [o.as_config_dict() for o in self.objects],
    #         "goals": [g.as_config_dict() for g in self.goals],
    #         "background_color": list(self.background_color),
    #         "metadata": self.metadata,
    #         "agent_init": self.agent_init if hasattr(self, "agent_init") else None,
    #         "init_type": self.init_type.as_dict()
    #     }

    def save_yaml(self, path):
        import yaml

        with open(path, "w") as f:
            yaml.dump(self.as_dict, f)

    # @staticmethod
    # def from_dict(d):
    #     from ..world.objects.ObjectFactory import ObjectFactory
    #     from ..world.goals.GoalFactory import GoalFactory
    #     from ..behavior.BehaviorFactory import BehaviorFactory
    #     from ..config.AgentConfig import AgentConfigFactory

    #     objects, a_config, behavior = None, None, []
    #     if "objects" in d:
    #         d["objects"] = [ObjectFactory.create(o) for o in d["objects"]]
    #     if "goals" in d:
    #         d["goals"] = [GoalFactory.create(g) for g in d["goals"]]
    #     if "behavior" in d:
    #         d["behavior"] = [BehaviorFactory.create(b) for b in d["behavior"]]
    #     if "agent_config" in d:
    #         d["agentConfig"] = AgentConfigFactory.create(d["agent_config"])
    #     if "init_type" in d:
    #         t = d["init_type"]
    #         if isinstance(t, dict):
    #             if t["type"] == "RectRandomInit":
    #                 d["init_type"] = RectRandomInitialization.from_dict(t)
    #             elif t["type"] == "FixedInit":
    #                 d["init_type"] = FixedInitialization.from_dict(t)
    #     return RectangularWorldConfig(**d)


class WorldYAMLFactory:
    @staticmethod
    def from_yaml(file_name):
        from ..util.yaml import load

        config = None
        with open(file_name, "r") as f:
            config = load(f)

        w_type = config["type"]
        if w_type == "RectangularWorld":
            init_type = config["init_type"]["type"]
            if init_type == "RectRandomInit":
                from novel_swarms.world.initialization.RandomInit import RectRandomInitialization

                if "num_agents" not in config["init_type"]:
                    config["init_type"]["num_agents"] = int(config["n_agents"])
                config["init_type"] = RectRandomInitialization(**config["init_type"])

            return RectangularWorldConfig.from_dict(config)
        else:
            raise Exception(f"Unknown World type: {config['type']}")
