import pygame
from ..agent.DiffDriveAgent import DifferentialDriveAgent
from ..agent.MazeAgent import MazeAgent
from .abstractGUI import AbstractGUI
from ..world.World import World
from functools import cache
import numpy as np
from importlib import resources
from . import fonts


PI = np.pi
font_dir = resources.files(fonts)


@cache
def get_font(font_name, size):
    return pygame.font.Font(font_dir / font_name, size)


class DifferentialDriveGUI(AbstractGUI):
    # Pair the GUI to the World
    world = None
    title = None
    subtitle = None
    selected = None
    text_baseline = 10

    def __init__(self, x=0, y=0, w=0, h=0):
        super().__init__(x=x, y=y, w=w, h=h)
        self.time: int = 0
        self.fps = None
        self.speed = None
        self.position = "absolute"

    def set_selected(self, agent: DifferentialDriveAgent):
        super().set_selected(agent)
        self.selected = agent

    def set_title(self, title, subtitle=None):
        self.title = title
        self.subtitle = subtitle

    def set_world(self, world: World):
        self.world = world

    def set_time(self, time_steps):
        self.time = time_steps

    def draw(self, screen, zoom=1.0):
        # super().draw(screen)
        if self.position == "sidebar_right":
            self.x = screen.get_width() - self.w
            self.y = 0
            self.h = screen.get_height()
        self.text_baseline = 10
        if pygame.font:
            if self.title:
                self.appendTextToGUI(screen, self.title, size=20)
            if self.subtitle:
                self.appendTextToGUI(screen, self.subtitle, size=18)
            timing = f"Timesteps:{self.time: >6}"
            timing += f"  e/FPS:{int(self.fps[1]): >3}/{int(self.fps[0]):03}" if self.fps else ""
            timing += f"  {self.speed}" if self.speed else ""
            self.appendTextToGUI(screen, timing)  # type: ignore[reportAttributeAccessIssue]
            if self.selected:
                a: MazeAgent = self.selected
                heading = a.get_heading() % (2 * PI) / 2 / PI * 360
                self.appendTextToGUI(screen, f"Current Agent: {a.name}")
                self.appendTextToGUI(screen, f"")
                self.appendTextToGUI(screen, f"x: {round(a.get_x_pos(), 12)}")
                self.appendTextToGUI(screen, f"y: {round(a.get_y_pos(), 12)}")
                self.appendTextToGUI(screen, f"θ: {round(heading, 12): >16}")
                self.appendTextToGUI(screen, f"D: {round(a.iD, 12): >16}")
                self.appendTextToGUI(screen, f"dx: {round(a.dx, 12)}")
                self.appendTextToGUI(screen, f"dy: {round(a.dy, 12)}")
                self.appendTextToGUI(screen, f"dt: {round(a.dt, 12)}")
                self.appendTextToGUI(screen, f"|v| (m/s): {round(np.linalg.norm(a.getVelocity()) / a.dt, 12)}")
                self.appendTextToGUI(screen, f"ω (rad/s): {round(a.dtheta, 12)}")
                self.appendTextToGUI(screen, f"sense-state: {a.get_sensors().getState()}")
                if hasattr(a, "i_1") and hasattr(a, "i_2"):
                    self.appendTextToGUI(screen, f"Idio_1: {a.i_1}")
                    self.appendTextToGUI(screen, f"Idio_2: {a.i_2}")
                self.appendTextToGUI(screen, f"")
                if hasattr(a, "controller"):
                    self.appendTextToGUI(screen, f"controller: {a.controller}")
                    self.appendTextToGUI(screen, f"")
                if hasattr(a, "agent_in_sight"):
                    agent_seen = getattr(a.agent_in_sight, "name", '')
                    self.appendTextToGUI(screen, f"sees: {agent_seen}")
                try:
                    v, w = a.requested
                except AttributeError:
                    pass
                else:
                    self.appendTextToGUI(screen, f"ego v     (m/s): {round(v, 12)}")
                    self.appendTextToGUI(screen, f"ego v (bodylen): {round(v / (a.radius * 2), 12)}")
                    self.appendTextToGUI(screen, f"ego ω   (rad/s): {round(w, 12)}")
            else:
                self.appendTextToGUI(screen, "Current Agent: None")
                self.appendTextToGUI(screen, "")
                self.appendTextToGUI(screen, "Behavior", size=16)
                for b in self.world.behavior:
                    out = b.out_current()
                    b.draw(screen)
                    try:
                        self.appendTextToGUI(screen, "{} : {:0.3f}".format(out[0], out[1]))
                    except ValueError:
                        pass
                    except Exception:
                        self.appendTextToGUI(screen, "{} : {}".format(out[0], out[1]))
        else:
            print("NO FONT")

    def appendTextToGUI(self, screen, text, x=None, y=None, color=(255, 255, 255), aliasing=True, size=14):

        if x is None:
            x = self.x + size - 2
        if y is None:
            y = self.text_baseline

        font = get_font("JetBrainsMono-SemiBold.ttf", size)
        text = font.render(text, aliasing, color)
        textpos = (x, y)
        screen.blit(text, textpos)
        self.text_baseline += size + 1
