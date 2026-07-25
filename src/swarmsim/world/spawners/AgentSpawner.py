import numpy as np
import copy
from numbers import Real

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
        if isinstance(facing, str) and facing not in ['towards', 'away', 'random']:
            msg = f"Invalid option for key 'facing' in spawner config: {facing}"
            raise ValueError(msg)
        self.facing = facing

    def generate_config(self, name=None):
        config = super().generate_config()

        if name is not None and config.name is None:
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

    @property
    def center_point(self):
        return self.agent_config.position

    def set_angle_post_spawn(self, agent):
        def angle(a, b):
            """Get angle between two points. If points are too close, return random angle."""
            b, a = np.asarray(b, dtype=np.float64).reshape(2), np.asarray(a, dtype=np.float64).reshape(2)
            d = np.linalg.norm(agent.pos - self.agent_config.position)
            return self.rng.uniform(0, np.pi * 2) if d < 0.000_001 else self.angle_between(a, b)

        match self.facing:
            case None:
                pass
            case [Real() as theta] | (Real() as theta):  # either singleton sequence of Real or Real
                agent.angle = theta
            case (Real(), Real()) | np.ndarray(size=2):  # either ordered pair tuple or ndarray with two elements
                # nb: angle_between does reshaping, so it's okay to pass shapes like [[[]], [[]]]
                agent.angle = angle(agent.pos, self.facing)
            # ^ need to handle ndarray above. otherwise comparing ndarray to str will error
            case np.ndarray():
                raise ValueError("Invalid option for key 'facing' in spawner config: ndarray with more than 2 elements")
            case 'towards':
                agent.angle = angle(agent.pos, self.center_point)
            case 'away':
                agent.angle = angle(self.center_point, agent.pos)
            case 'random':
                agent.angle = self.rng.uniform(0, np.pi * 2)
            case _:
                msg = f"Invalid option for key 'facing' in spawner config: {self.facing}"
                raise ValueError(msg)

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
        if shell.size == 4:
            shell = AABB(shell.reshape(2, 2)).corners
        if holes is not None:
            holes = np.asarray(holes, dtype=np.float64)
        try:
            self.poly = Polygon(shell, holes)
        except ValueError as err:
            raise ValueError("Invalid region specified for UniformAgentSpawner") from err
        self.aabb = AABB(shell)  # HACK: may happen twice if shell.size == 4
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

    @property
    def center_point(self):
        return self.poly.centroid.xy

    def generate_config(self, name=None):
        config = super().generate_config(name)
        config.position = self.generate_points_in_polygon(1).flatten()
        return config


class UniformCircleAgentSpawner(PointAgentSpawner):
    def __init__(
        self,
        world,
        center=None,
        radius=None,
        **kwargs
    ):
        super().__init__(world, **kwargs)
        if center is None or radius is None:
            raise ValueError("center and radius must be specified for UniformCircleAgentSpawner")
        self.center = np.asarray(center)
        self.radius = radius

    def generate_points_in_circle(self, n: int):
        theta = self.rng.uniform(0, np.pi * 2, size=n)
        radii = np.sqrt(self.rng.random(size=(n, 1))) * self.radius
        points = np.array([np.cos(theta), np.sin(theta)]).reshape(-1, 2)
        return radii * points + self.center

    @property
    def center_point(self):
        return self.center

    def generate_config(self, name=None):
        config = super().generate_config(name)
        config.position = self.generate_points_in_circle(1).flatten()
        return config
