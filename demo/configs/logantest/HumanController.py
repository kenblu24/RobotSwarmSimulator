from functools import cached_property

import numpy as np
import pygame

from novel_swarms.agent.control.AbstractController import AbstractController
from math import copysign


class HumanController(AbstractController):
    def __init__(
        self, agent=None, parent=None,
        joystick=0,
        keys='wasd',

    ):
        super().__init__(agent=agent, parent=parent)

        pygame.joystick.init()

        self.joy_id = joystick
        self.keys = keys

        self.body_color = (255, 0, 255)
        self.v, self.omega = 0, 0

    @property
    def joy_id(self):
        return self._joy_id

    @joy_id.setter
    def joy_id(self, value):
        self._joy_id = value
        self.joystick = None

        if pygame.joystick.get_count() == 0 and value is not None:
            raise ValueError("No joysticks found")
        if self._joy_id is not None:
            self.joystick = pygame.joystick.Joystick(value)

    def get_actions(self, agent):  # uses arrow keys to move
        VEL = 0.3
        RAD_KEY, RAD_CON = 0.1, 1

        self.handle_controller(VEL, RAD_CON)
        self.handle_key_press(VEL, RAD_KEY)

        self.v, self.omega = np.clip(self.v, -0.3, 0.3), np.clip(self.omega, -1.5, 1.5)

        return self.v, self.omega

    def handle_key_press(self, vel, rad):
        # if self.joystick.get_axis(1) < 0.01 and abs(self.joystick.get_axis(2)) < 0.01 and self.joystick.get_hat(0)[0] == (0, 0)):
        if not self.keys:
            return

        keys = pygame.key.get_pressed()

        if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            self.omega += rad
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            self.omega -= rad
        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            self.v = -vel
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            self.v = vel

        if not (keys[pygame.K_RIGHT] or keys[pygame.K_d]) and not (keys[pygame.K_LEFT] or keys[pygame.K_a]):
            self.omega = 0

        if not (keys[pygame.K_DOWN] or keys[pygame.K_s]) and not (keys[pygame.K_UP] or keys[pygame.K_w]):
            self.v = 0

    def handle_controller(self, vel, rad):
        if pygame.joystick.get_count() > 0 and True not in pygame.key.get_pressed():
            self.v = -self.joystick.get_axis(1) * vel if abs(self.joystick.get_axis(1)) > 0.01 else self.joystick.get_hat(0)[1] * vel
            self.omega = self.joystick.get_axis(2) * rad if abs(self.joystick.get_axis(2)) > 0.01 else self.joystick.get_hat(0)[0] * rad
