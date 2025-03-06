from functools import cached_property

import numpy as np
import pygame

from novel_swarms.agent.control.AbstractController import AbstractController
from novel_swarms.agent.MazeAgent import MazeAgent

class HumanController(AbstractController):
    def __init__(self, agent=None, parent=None, control="keys-holonomic"):  # type:ignore[reportMissingSuperCall]
        super().__init__(agent=agent, parent=parent)
        
        self.control = control
        self.body_color = (255, 0, 255)


    def get_actions(self, agent): #uses arrow keys to move
        v, omega = 0, 0
        if self.control == "keys-holonomic":
            VEL = 0.3
            RAD = 0.5
            keys = pygame.key.get_pressed()
            if keys[pygame.K_RIGHT]:
                omega = RAD
            if keys[pygame.K_LEFT]:
                omega = -RAD
            if keys[pygame.K_DOWN]:
                v = -VEL
            if keys[pygame.K_UP]:
                v = VEL
        
        return v, omega




    # def handle_key_press(self, agent, keys):
    #     if self.control == "keys-holonomic":
    #         STEP_SIZE = 2
    #         if keys[pygame.K_RIGHT]:
    #             self.set_x_pos(self.get_x_pos() + STEP_SIZE)
    #         if keys[pygame.K_LEFT]:
    #             self.set_x_pos(self.get_x_pos() - STEP_SIZE)
    #         if keys[pygame.K_DOWN]:
    #             self.set_y_pos(self.get_y_pos() + STEP_SIZE)
    #         if keys[pygame.K_UP]:
    #             self.set_y_pos(self.get_y_pos() - STEP_SIZE)