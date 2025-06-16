import numpy as np
import copy

from shapely import Polygon

from .Spawner import Spawner
from ...util.collider.AABB import AABB
from ...config import get_agent_class

# typing:
from ...agent.Agent import Agent, BaseAgentConfig
from ...agent.StaticAgent import StaticAgent, StaticAgentConfig
from ...agent.MazeAgent import MazeAgent

import warnings


def AgentSpawner(*args, **kwargs):
    # suss out the type of spawner
    raise NotImplementedError("AgentSpawner is intended to be a convenience function for choosing the right spawner based on the arguments. For now please use the spawner classes as the 'type'.")  # noqa


class BaseAgentSpawner(Spawner):
    def __init__(
        self,
        world,
        agent=None,
        n: int | None = 1,
        seed='unspecified',
        mode='oneshot',
        oneshot=False,
        delay=0,
        **kwargs
    ):
        super().__init__(world, seed, **kwargs)
        self.type = 'agent'
        self.mode: str = 'oneshot' if oneshot else str(mode).lower()
        if oneshot:
            warnings.warn("The 'oneshot' argument is deprecated. Use `mode='oneshot'` instead.", FutureWarning, stacklevel=1)
        self.n_objects = n
        self.delay = delay

        self.set_agent_config(agent)
        self.agent_class: type
        self.agent_config: Agent | BaseAgentConfig

    def set_agent_config(self, agent):
        if isinstance(agent, Agent):
            self.agent_class, self.agent_config = type(agent), agent  # this is a REFERENCE, not a COPY!
        elif agent:
            self.agent_class, self.agent_config = get_agent_class(agent)  # pyright: ignore[reportAttributeAccessIssue]
        else:
            self.agent_class, self.agent_config = None, None  # pyright: ignore[reportAttributeAccessIssue]

    def generate_config(self):
        if isinstance(self.agent_config, BaseAgentConfig):
            return copy.deepcopy(self.agent_config)
        elif isinstance(self.agent_config, Agent):
            return self.agent_config.copy()
            

    def make_agent(self, config):
        if isinstance(self.agent_config, Agent):
            return config
        else:
            return self.agent_class.from_config(config, self.world)


class PointAgentSpawner(BaseAgentSpawner):
    def __init__(
        self,
        world,
        facing=None,
        avoid_overlap=False,
        **kwargs
    ):
        super().__init__(world, **kwargs)
        self.avoid_overlap = avoid_overlap
        if isinstance(facing, str):
            if facing.lower() == 'random':
                self.facing = [0, np.pi]
            elif facing.lower() in ['towards', 'away']:
                self.facing = facing.lower()
            else:
                msg = f"Invalid option for key 'facing' in spawner config: {facing}"
                raise ValueError(msg)
        else:
            self.facing = facing

    def generate_config(self, name=None):
        config = super().generate_config()

        # modify agent config
        if isinstance(self.facing, (list, tuple, np.ndarray)):
            config.angle = self.rng.uniform(*self.facing)
        if name is not None:
            config.name = name

        return config

    def step(self):
        if self.mark_for_deletion:
            return
        super().step()
        if self.spawned < self.n_objects:
            if self.mode == 'oneshot':
                # oneshot mode should spawn everything in a single step
                for i in range(self.spawned, self.n_objects):
                    self.do_spawn(str(i))
            # TODO: implement non-oneshot modes i.e. spawn in a loop, spawn at a fixed interval, etc.
            self.mark_for_deletion = True
        # self.world.draw(self.world._screen_cache)

    @staticmethod
    def angle_between(a, b):
        b, a = np.asarray(b, dtype=np.float64).reshape(2), np.asarray(a, dtype=np.float64).reshape(2)
        vec = b - a
        return np.arctan2(*reversed(vec))

    def set_angle_post_spawn(self, agent):
        d = np.linalg.norm(agent.pos - self.agent_config.position)
        if d < 0.000_001:
            return
        if isinstance(self.facing, str):
            if self.facing == 'towards':
                agent.angle = self.angle_between(agent.pos, self.agent_config.position)
            elif self.facing == 'away':
                agent.angle = self.angle_between(self.agent_config.position, agent.pos)

    def do_spawn(self, name=None):
        config = self.generate_config(name)
        agent = self.make_agent(config)
        self.world.population.append(agent)  # make world aware of the new agent. necessary for collision handling
        if self.avoid_overlap and isinstance(agent, MazeAgent):
            agent.handle_collisions(self.world, max_attempts=5, nudge_amount=0.4, rng=self.rng, refresh=True)
            agent.handle_collisions(self.world, max_attempts=10, nudge_amount=1.0, rng=self.rng, refresh=True)

        self.set_angle_post_spawn(agent)
        self.spawned += 1
        return agent


class UniformAgentSpawner(PointAgentSpawner):
    def __init__(
        self,
        world,
        region=None,
        holes=None,
        **kwargs
    ):
        super().__init__(world, **kwargs)
        if region is None:
            raise ValueError("region must be specified for UniformAgentSpawner")
        shell = np.asarray(region, dtype=np.float64)
        if holes is not None:
            holes = np.asarray(holes, dtype=np.float64)
        try:
            self.poly = Polygon(shell, holes)
        except ValueError as err:
            raise ValueError("Invalid region specified for UniformAgentSpawner") from err
        self.aabb = AABB(shell)
        self.is_aabb = self.aabb.is_mungible(shell, tolerance=0.000_001)
        if self.is_aabb:
            self.poly = Polygon(self.aabb.corners)
        else:
            import pointpats.random  # preload pointpats (loads slowly)

    def generate_points_in_polygon(self, n: int):
        if self.is_aabb:
            return self.rng.uniform(*self.aabb._cs, size=(n, self.aabb._min.size))
        from pointpats.random import poisson
        np.random.seed(self.rng.integers(0, 90000))
        return poisson(self.poly, size=n)

    def set_angle_post_spawn(self, agent):
        if isinstance(self.facing, str):
            if self.facing == 'towards':
                agent.angle = self.angle_between(agent.pos, self.poly.centroid.xy)
            elif self.facing == 'away':
                agent.angle = self.angle_between(self.poly.centroid.xy, agent.pos)

    def generate_config(self, name=None):
        config = super().generate_config(name)
        config.position = self.generate_points_in_polygon(1).flatten()
        return config
