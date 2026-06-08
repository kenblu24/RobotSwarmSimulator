import numpy as np

from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner


class DonutAgentSpawner(PointAgentSpawner):
    def __init__(
        self,
        world,
        n=1,
        agent=None,
        facing=None,
        avoid_overlap=False,
        seed='unspecified',
        oneshot=False,
        center=(5.0, 5.0),
        inner_radius=4.0,
        outer_radius=6.0,
        **kwargs
    ):
        super().__init__(
            world=world,
            n=n,
            agent=agent,
            facing=facing,
            avoid_overlap=avoid_overlap,
            seed=seed,
            oneshot=oneshot,
            **kwargs
        )
        self.center = center
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius

    def generate_position(self):
        direction = self.rng.uniform(0, 2 * np.pi)
        radius = self.rng.uniform(self.inner_radius, self.outer_radius)
        vec = np.array([np.cos(direction), np.sin(direction)])
        return self.center + vec * radius

    def generate_config(self, name=None):
        config = super().generate_config(name)
        config.position = self.generate_position().flatten()
        return config
