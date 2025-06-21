import pygame
import numpy as np

from swarmsim.sensors.AbstractSensor import AbstractSensor

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from swarmsim.world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None


def distance(a, b):
    return np.linalg.norm(b - a)


class RelativePositionSensor(AbstractSensor):
    # these are the variables that should be included in the config
    config_vars = AbstractSensor.config_vars + [
        'r', 'store_history',
    ]

    def __init__(
        self,
        agent=None,
        parent=None,
        distance=10,
        store_history=False,
    ):
        super().__init__(agent=agent, parent=parent)
        self.r = distance
        self.store_history = store_history

        self.history = []

    def step(self, world: RectangularWorld) -> None:
        super().step(world=world)

        agents = [agent for agent in world.population if agent != self.agent]

        self.current_state =  # TODO: implement

    def draw(self, screen, offset=((0, 0), 1.0)) -> None:
        super().draw(screen, offset)
        pan, zoom = np.asarray(offset[0]), offset[1]
        if self.show:
            # Draw Sensory Range
            p = np.asarray(self.position) * zoom + pan
            pygame.draw.circle(screen, (255, 0, 0), p, self.r * zoom, width=1)
