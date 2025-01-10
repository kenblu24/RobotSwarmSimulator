import pygame
import numpy as np
import math
from .AbstractSensor import AbstractSensor
from typing import List


class RegionalSensor(AbstractSensor):
    def __init__(self, world_object_index, parent=None, draw=True, history_length=50):
        super(RegionalSensor, self).__init__(parent=parent, draw=draw)
        self.current_state = 0
        self.history = []
        self.hist_len = history_length
        self.i = world_object_index

    def checkForLOSCollisions(self, world) -> None:
        sensor_position = self.agent.getPosition()

        # Check if agent is directly in region
        if world.objects[self.i].point_inside(sensor_position):
            self.current_state = 1
            self.add_to_history(self.current_state)
            return

        self.current_state = 0
        self.add_to_history(self.current_state)

    def step(self, world):
        super(RegionalSensor, self).step(world=world)
        self.checkForLOSCollisions(world=world)

    def draw(self, screen, zoom=1.0):
        if not self.show:
            return
        super(RegionalSensor, self).draw(screen, zoom)

        # Draw Sensory Vector (Vision Vector)
        sight_color = (255, 0, 0)
        if self.current_state == 1:
            sight_color = (0, 255, 0)

        width = max(1, round(zoom))
        r = self.agent.radius * zoom + width * 2
        pos = np.asarray(self.agent.getPosition()) * zoom
        pygame.draw.circle(screen, sight_color, (*pos,), r, width=width)

    def getLOSVector(self) -> List:
        head = self.agent.getPosition()
        tail = self.getFrontalPoint()
        return [tail[0] - head[0], tail[1] - head[1]]

    def getFrontalPoint(self):
        if self.angle is None:
            return self.agent.getFrontalPoint()
        return self.agent.x_pos + math.cos(self.angle + self.agent.angle), self.agent.y_pos + math.sin(self.angle + self.agent.angle)

    def add_to_history(self, value):
        if len(self.history) > self.hist_len:
            self.history = self.history[1:]
        self.history.append(value)