"""
Donut Agent Spawner

.. autoclass:: DonutAgentSpawner
    :members:
    :undoc-members:
"""

import numpy as np

from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner


class DonutAgentSpawner(PointAgentSpawner):
    """Spawn agents uniformly within an annular region about a center point.

    .. inheritance-diagram:: swarmsim.world.spawners.DonutSpawner.DonutAgentSpawner
        :parts: 1

    Parameters
    ----------
    agent : Agent | BaseAgentConfig | dict
        The agent config to use for the spawned agents. Can be a config dataclass, dictionary, or instance of an agent.
    center : tuple[float, float] | np.ndarray
        The center of the circle.
    inner_radius : float
        The inner radius of the donut.
    outer_radius : float
        The outer radius of the donut.
    **kwargs
        Additional keyword arguments to pass to the base class.
        This spawner takes the same arguments as
        :class:`~swarmsim.world.spawners.AgentSpawner.PointAgentSpawner` and
        :class:`~swarmsim.world.spawners.AgentSpawner.BaseAgentSpawner`.
    """
    def __init__(
        self,
        center=(5.0, 5.0),
        inner_radius=4.0,
        outer_radius=6.0,
        **kwargs
    ):
        super().__init__(
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
