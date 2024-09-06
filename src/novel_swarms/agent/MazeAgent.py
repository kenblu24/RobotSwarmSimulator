from typing import NamedTuple, Tuple
from collections import namedtuple
import pygame
import random
import math
import numpy as np
from copy import deepcopy

from .Agent import Agent
from ..sensors.GenomeDependentSensor import GenomeBinarySensor
from ..util.collider.AABB import AABB
from ..util.collider.CircularCollider import CircularCollider
from ..util.timer import Timer
from .control.Controller import Controller

# # typing
# try:
#     from ..config.AgentConfig import MazeAgentConfig
# except ImportError:
#     pass

SPA = namedtuple("SPA", ['state', 'perception', 'action'])
State = NamedTuple("State", [('x', float), ('y', float), ('angle', float)])


class MazeAgent(Agent):
    SEED = -1

    def __init__(self, config=None, name=None) -> None:

        if hasattr(config.controller, 'get_actions'):
            self.controller = config.controller
        else:
            self.controller = Controller(config.controller)

        self.seed = config.seed
        if config.seed is not None:
            self.set_seed(config.seed)

        if config.x is None:
            self.x_pos = random.randint(round(0 + config.agent_radius), round(config.world.w - config.agent_radius))
        else:
            self.x_pos = config.x

        if config.y is None:
            self.y_pos = random.randint(round(0 + config.agent_radius), round(config.world.h - config.agent_radius))
        else:
            self.y_pos = config.y

        super().__init__(self.x_pos, self.y_pos, name=name)

        if config.angle is None:
            self.angle = random.random() * math.pi * 2
        else:
            self.angle = config.angle

        self.radius = config.agent_radius
        self.dt = config.dt
        self.is_highlighted = False
        self.agent_in_sight = None
        self.idiosyncrasies = config.idiosyncrasies
        I1_MEAN, I1_SD = 0.93, 0.08
        I2_MEAN, I2_SD = 0.95, 0.06
        self.i_1 = np.random.normal(I1_MEAN, I1_SD) if self.idiosyncrasies else 1.0
        self.i_2 = np.random.normal(I2_MEAN, I2_SD) if self.idiosyncrasies else 1.0
        self.stop_on_collision = config.stop_on_collision
        self.catastrophic_collisions = config.catastrophic_collisions
        self.dead = False
        self.goal_seen = False
        self.stop_at_goal = config.stop_at_goal
        self.config = config

        self.sensors = deepcopy(config.sensors)
        for sensor in self.sensors:
            if isinstance(sensor, GenomeBinarySensor):
                sensor.augment_from_genome(config.controller)

        self.attach_agent_to_sensors()

        self.body_filled = config.body_filled
        if config.body_color == "Random":
            self.body_color = self.get_random_color()
        else:
            self.body_color = config.body_color

        self.history = []
        self.track_io = getattr(config, "track_io", False)

    def set_seed(self, seed):
        random.seed(seed)

    def step(self, check_for_world_boundaries=None, world=None, check_for_agent_collisions=None) -> None:

        if world is None:
            raise Exception("Expected a Valid value for 'World' in step method call - Unicycle Agent")

        # timer = Timer("Calculations")
        super().step()

        if self.dead:
            return

        if world.goals and world.goals[0].agent_achieved_goal(self) or self.detection_id == 2:
            if self.stop_at_goal:
                v, omega = 0, 0
            else:
                v, omega = self.controller.get_actions(self)
            # self.detection_id = 2
            # self.set_color_by_id(3)
        else:
            v, omega = self.controller.get_actions(self)

        if self.track_io:
            sensor_state = self.sensors.getState()
            self.history.append(SPA(
                State(self.x_pos, self.y_pos, self.angle),
                sensor_state,
                (v, omega),
            ))

        # Define Idiosyncrasies that may occur in actuation/sensing
        idiosync_1 = self.i_1
        idiosync_2 = self.i_2

        self.dx = v * math.cos(self.angle) * idiosync_1
        self.dy = v * math.sin(self.angle) * idiosync_1
        self.dtheta = omega * idiosync_2

        old_x_pos = self.x_pos
        old_y_pos = self.y_pos

        if self.stopped_duration > 0:
            self.stopped_duration -= 1
            if self.catastrophic_collisions:
                self.dead = True
                self.body_color = (200, 200, 200)
                return
        else:
            self.x_pos += self.dx * self.dt
            self.y_pos += self.dy * self.dt

        self.angle += self.dtheta * self.dt

        self.collision_flag = False
        if check_for_world_boundaries is not None:
            check_for_world_boundaries(self)

        self.handle_collisions(world)

        # Calculate the 'real' dx, dy after collisions have been calculated.
        # This is what we use for velocity in our equations
        self.dx = self.x_pos - old_x_pos
        self.dy = self.y_pos - old_y_pos
        # timer = timer.check_watch()

        for sensor in self.sensors:
            sensor.step(world=world)
            if sensor.goal_detected:
                self.goal_seen = True

    def draw(self, screen, offset=((0, 0), 1.0)) -> None:
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen, offset)
        for sensor in self.sensors:
            sensor.draw(screen, offset)

        # Draw Cell Membrane
        filled = 0 if (self.is_highlighted or self.stopped_duration or self.body_filled) else 1
        color = self.body_color if not self.stopped_duration else (255, 255, 51)
        pos = np.asarray(self.getPosition()) * zoom + pan
        pygame.draw.circle(screen, color, (*pos,), self.radius * zoom, width=filled)

        # "Front" direction vector
        head = np.asarray(self.getFrontalPoint()) * zoom + pan
        tail = pos
        vec = head - tail
        mag = self.radius * 2
        vec_with_magnitude = tail + vec * mag
        pygame.draw.line(screen, (255, 255, 255), tail, vec_with_magnitude)


    # def interpretSensors(self) -> Tuple:
    #     """
    #     Deprecated: See Controller Class (novel_swarms.agent.control.Controller)
    #     """
    #     sensor_state = self.sensors.getState()
    #     sensor_detection_id = self.sensors.getDetectionId()
    #     # self.set_color_by_id(sensor_detection_id)
    #
    #     # if sensor_state == 2:
    #     #     return 12, 0
    #
    #     v = self.controller[sensor_state * 2]
    #     omega = self.controller[(sensor_state * 2) + 1]
    #     return v, omega

    def set_color_by_id(self, id):
        if id == 0:
            self.body_color = self.config.body_color
        elif id == 2:
            self.body_color = (255, 255, 0)
            self.body_filled = True
        elif id == 3:
            self.body_color = (15, 15, 255)
            self.body_filled = True

    def get_random_color(self):
        rand_color = [255, 255, 255]
        while sum(rand_color) > 245*3:
            rand_color = np.random.choice(256, 3)
        return rand_color

    def handle_collisions(self, world):
        collisions = True
        attempts = 0
        while collisions and attempts < 10:
            collisions = False
            attempts += 1
            collider = self.build_collider()
            agent_set = world.getAgentsMatchingYRange(self.get_aabb())
            for agent in agent_set:
                if agent.name == self.name:
                    continue
                if self.get_aabb().intersects(agent.get_aabb()):
                    self.collision_flag = True
                    self.get_aabb().toggle_intersection()
                    correction = collider.collision_then_correction(agent.build_collider())
                    if correction is not None:
                        self.x_pos += correction[0]
                        self.y_pos += correction[1]
                        if self.catastrophic_collisions:
                            self.dead = True
                            self.body_color = (200, 200, 200)
                            agent.dead = True
                            agent.body_color = (200, 200, 200)
                        collisions = True
                        break



    def build_collider(self):
        return CircularCollider(self.x_pos, self.y_pos, self.radius)

    def debug_draw(self, screen):
        self.get_aabb().draw(screen)

    def get_aabb(self):
        """
        Return the Bounding Box of the agent
        """
        top_left = (self.x_pos - self.radius, self.y_pos - self.radius)
        bottom_right = (self.x_pos + self.radius, self.y_pos + self.radius)
        return AABB(top_left, bottom_right)



    def __str__(self) -> str:
        return "(x: {}, y: {}, r: {}, θ: {})".format(self.x_pos, self.y_pos, self.radius, self.angle)
