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
        
        # pygame.joystick.init()
        # if pygame.joystick.get_count() > 0:
        #     self.joystick = pygame.joystick.Joystick(0)
        #     self.joystick.init()

        self.body_color = (255, 0, 255)
        self.v, self.omega = 0, 0


    def get_actions(self, agent): #uses arrow keys to move
        VEL = 0.3
        RAD = 0.05

        # if pygame.joystick.get_count() > 0:
        #     self.joystick = pygame.joystick.Joystick(0)
        #     self.joystick.init()

        # if self.joystick.get_axis(1) > 0 or self.joystick.get_axis(2) > 0:
        #     return self.joystick.get_axis(1) * VEL, self.joystick.get_axis(2) * RAD
        # if pygame.display.get_init():
        #     event = pygame.event.get()
        #     # if event.axis == 1:
        #     #     self.v = event.value * VEL
        #     # if event.axis == 2:
        #     #     self.omega = event.value * RAD

        if self.control == "keys-holonomic":
            keys = pygame.key.get_pressed()

            if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
                self.omega += RAD
            if keys[pygame.K_LEFT] or keys[pygame.K_a]:
                self.omega -= RAD
            if keys[pygame.K_DOWN] or keys[pygame.K_s]:
                self.v = -VEL
            if keys[pygame.K_UP] or keys[pygame.K_w]:
                self.v = VEL
            
            if not (keys[pygame.K_RIGHT] or keys[pygame.K_d]) and not (keys[pygame.K_LEFT] or keys[pygame.K_a]):
                if abs(self.omega) > RAD:
                    self.omega -= copysign(RAD, self.omega)
                else:
                    self.omega = 0

            if not (keys[pygame.K_DOWN] or keys[pygame.K_s]) and not (keys[pygame.K_UP] or keys[pygame.K_w]):
                # if abs(self.v) > VEL:
                #     self.v -= copysign(VEL, self.v)
                # else:
                #     self.v = 0
                self.v = 0

        
        return np.clip(self.v, -0.3, 0.3), np.clip(self.omega, -2, 2)





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