import numpy as np
import copy

from shapely import Polygon

from .Spawner import Spawner
from ...util.collider.AABB import AABB
from ...config import get_agent_class

# typing:
from ...agent.StaticAgent import StaticAgent, StaticAgentConfig


def AgentSpawner(*args, **kwargs):
    # suss out the type of spawner
    raise NotImplementedError("AgentSpawner is intended to be a convenience function for choosing the right spawner based on the arguments. For now please use the spawner classes as the 'type'.")  # noqa


class PointAgentSpawner(Spawner):
    def __init__(
        self,
        world,
        n=1,
        agent=None,
        facing=None,
        avoid_overlap=False,
        seed=None,
        oneshot=False,
        **kwargs
    ):
        super().__init__(world, **kwargs)
        self.type = 'agent'
        self.oneshot = oneshot
        self.n_objects = n
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

        if seed is None:
            self.seed = np.random.randint(0, 90000)
            self.rng = np.random.default_rng(self.seed)
        else:
            self.seed = seed
            self.rng = np.random.default_rng(self.seed)

        self.agent_class, self.agent_config = get_agent_class(agent)
        self.agent_class: StaticAgent
        self.agent_config: StaticAgentConfig

    def generate_config(self, name=None):
        config = copy.deepcopy(self.agent_config)

        # modify agent config
        if isinstance(self.facing, (list, tuple, np.ndarray)):
            config.angle = self.rng.uniform(*self.facing)
        if name is not None:
            config.name = name

        return config

    def make_agent(self, config):
        return self.agent_class.from_config(config, self.world)

    def step(self):
        if self.mark_for_deletion:
            return
        super().step()
        if self.spawned < self.n_objects:
            if self.oneshot:
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
        if self.avoid_overlap:
            agent.handle_collisions(self.world, max_attempts=5, nudge_amount=0.4, rng=self.rng, refresh=True)
            agent.handle_collisions(self.world, max_attempts=10, nudge_amount=1.0, rng=self.rng, refresh=True)

        self.set_angle_post_spawn(agent)


class UniformAgentSpawner(PointAgentSpawner):
    def __init__(
        self,
        world,
        n=1,
        agent=None,
        facing=None,
        avoid_overlap=False,
        seed=None,
        oneshot=False,
        region=None,
        holes=None,
        **kwargs
    ):
        super().__init__(world, n, agent, facing, avoid_overlap, seed, oneshot, **kwargs)
        if region is None:
            raise ValueError("region must be specified for UniformAgentSpawner")
        shell = np.asarray(region, dtype=np.float64)
        if holes is not None:
            holes = np.asarray(holes, dtype=np.float64)
        try:
            self.poly = Polygon(shell, holes)
        except ValueError:
            raise ValueError("Invalid region specified for UniformAgentSpawner")
        self.aabb = AABB(shell)
        self.is_aabb = self.aabb.is_mungible(shell, tolerance=0.000_001)
        if self.is_aabb:
            self.poly = Polygon(self.aabb.corners)
        else:
            import pointpats.random

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
