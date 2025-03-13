from functools import cached_property

import numpy as np
import pygame

from novel_swarms.agent.control.AbstractController import AbstractController
from novel_swarms.agent.MazeAgent import MazeAgent
from math import copysign

class HumanController(AbstractController):
    def __init__(self, agent=None, parent=None, control="keys-holonomic"):  # type:ignore[reportMissingSuperCall]
        super().__init__(agent=agent, parent=parent)
        
        self.control = control
        
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)

        self.body_color = (255, 0, 255)
        self.v, self.omega = 0, 0


    def get_actions(self, agent): #uses arrow keys to move
        VEL = 0.3
        RAD_KEY, RAD_CON = 0.1, 1
        
        self.handle_controller(VEL, RAD_CON)
        self.handle_key_press(VEL, RAD_KEY)     
        
        return self.v, self.omega


    def handle_key_press(self, vel, rad):
        if self.control == "keys-holonomic" and (abs(self.joystick.get_axis(1)) < 0.01 and abs(self.joystick.get_axis(2)) < 0.01 and self.joystick.get_hat(0)[0] == (0, 0)):
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

        self.v, self.omega = np.clip(self.v, -0.3, 0.3), np.clip(self.omega, -1.5, 1.5)
            

    def handle_controller(self, vel, rad):
        if pygame.joystick.get_count() > 0 and not (True in pygame.key.get_pressed()):
            self.v = -self.joystick.get_axis(1) * vel if abs(self.joystick.get_axis(1)) > 0.01 else self.joystick.get_hat(0)[1] * vel
            self.omega = self.joystick.get_axis(2) * rad if abs(self.joystick.get_axis(2)) > 0.01 else self.joystick.get_hat(0)[0] * rad
            