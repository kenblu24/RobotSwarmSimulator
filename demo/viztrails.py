import numpy as np
import pygame
from numpy.typing import NDArray

from swarmsim import config_from_yaml
from swarmsim.gui.agentGUI import DifferentialDriveGUI
from swarmsim.world import RectangularWorld
from swarmsim.world.simulate import main


class VizTrail:
    def __init__(self, interval: int = 2, window: int | None = None, opacity=0.5):
        assert 0. <= opacity <= 1., "Opacity should be in [0, 1]"
        
        self.agent_pos: dict[str, list[NDArray]] = {}
        self.agent_cfg: dict[str, tuple[float, pygame.Surface]] = {}

        self.timesteps = 0
        self.interval = interval
        self.window = window
        self.opacity = opacity

    def _update_positions(self, world: RectangularWorld, screen_size):
        for agent in world.population:
            if agent.name not in self.agent_pos:
                self.agent_pos[agent.name] = []
                self.agent_cfg[agent.name] = agent.radius, pygame.Surface(screen_size, pygame.SRCALPHA)

            self.agent_pos[agent.name].append((agent.getPosition().copy(), agent.angle))

    def draw(self, screen, world: RectangularWorld):
        def color_hsla(color: pygame.Color, angle_rad: float, s=0.5, l=0.5):
            color.hsla = np.rad2deg(angle_rad)%360., s*100., l*100., self.opacity*100.

        if self.timesteps % self.interval == 0:
            self._update_positions(world, screen.get_size())

        pan, zoom = world.pos, world.zoom
        surface = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        color = pygame.Color("white")
        for name, orientation in self.agent_pos.items():
            radius, surface = self.agent_cfg[name]
            surface.fill("#00000000")
            if self.window is not None:
                orientation = orientation[-self.window:]

            for (pos, heading) in orientation:
                color_hsla(color, heading)
                pygame.draw.circle(surface, color, pos * zoom, radius * zoom)

        self.timesteps += 1
        for _, surface in self.agent_cfg.values():
            screen.blit(surface, pan)


class VizTest(DifferentialDriveGUI):
    def __init__(self, x=0, y=0, w=0, h=0):
        super().__init__(x, y, w, h)
        self.tracker = VizTrail()

    def draw(self, screen):
        self.tracker.draw(screen, self.world)
        super().draw(screen)


world_cfg = config_from_yaml("demo/configs/turbopi-milling/world.yaml")
main(
    world_config=world_cfg,
    start_paused=True,
    gui=VizTest(),
    stop_detection=None,
    save_duration=1200,
    save_every_ith_frame=3,
    save_time_per_frame=50,
)
