import numpy as np
import pygame
from numpy.typing import NDArray

from swarmsim import config_from_yaml
from swarmsim.gui.agentGUI import DifferentialDriveGUI
from swarmsim.world import RectangularWorld
from swarmsim.world.simulate import main


class VizTrail:
    def __init__(self, interval=2, window=100):
        self.agent_pos: dict[str, list[NDArray]] = {}
        self.agent_cfg: dict = {}

        self.timesteps = 0
        self.interval = interval
        self.window = window
        self.alpha_falloff = lambda x: 0.85
        # self.alpha_falloff = lambda x: 0.9 * x/self.window

    def _update_positions(self, world: RectangularWorld):
        for agent in world.population:
            if agent.name not in self.agent_pos:
                self.agent_pos[agent.name] = []
                self.agent_cfg[agent.name] = agent.radius

            self.agent_pos[agent.name].append((agent.getPosition().copy(), agent.angle))


    def draw(self, screen, world: RectangularWorld):
        def color_hsla(color: pygame.Color, index: int, angle_rad: float, s=0.5, l=0.5):
            color.hsla = np.rad2deg(angle_rad)%360., s*100., l*100., self.alpha_falloff(index)*100.

        if self.timesteps % self.interval == 0:
            self._update_positions(world)

        pan, zoom = world.pos, world.zoom
        surface = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        color = pygame.Color("white")
        for name, orientation in self.agent_pos.items():
            radius = self.agent_cfg[name]
            for i, (pos, heading) in enumerate(orientation[-self.window:]):
                color_hsla(color, i, heading)
                pygame.draw.circle(surface, color, pos * zoom, radius * zoom)

        self.timesteps += 1
        screen.blit(surface, pan)


class VizTest(DifferentialDriveGUI):
    def __init__(self, x=0, y=0, w=0, h=0):
        super().__init__(x, y, w, h)
        self.tracker = VizTrail()

    def draw(self, screen):
        super().draw(screen)
        self.tracker.draw(screen, self.world)


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
