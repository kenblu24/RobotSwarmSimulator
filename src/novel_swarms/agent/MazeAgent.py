import math
import random
from copy import deepcopy
from typing import NamedTuple
from dataclasses import dataclass, field
from collections import namedtuple

import pygame
import numpy as np

from ..config import filter_unexpected_fields, associated_type
from .StaticAgent import StaticAgent, StaticAgentConfig
from ..sensors.GenomeDependentSensor import GenomeBinarySensor
from ..util.collider.AABB import AABB
from ..util.collider.CircularCollider import CircularCollider
from ..util.timer import Timer
from ..util import statistics_tools as st
from .control.Controller import Controller
from ..world.RectangularWorld import RectangularWorldConfig

# # typing
from typing import Any, override
from ..world.World import World
# try:
#     from ..config.AgentConfig import MazeAgentConfig
# except ImportError:
#     pass

SPA = namedtuple("SPA", ['state', 'perception', 'action'])
State = NamedTuple("State", [('x', float), ('y', float), ('angle', float)])


@associated_type("MazeAgent")
class MazeAgentConfig(StaticAgentConfig):
    world: World | None = None
    world_config: RectangularWorldConfig | None = None
    seed: Any = None
    dt: float = 1.0
    sensors: list = field(default_factory=list)
    controller: Any = None
    # sensors: SensorSet | None = None
    idiosyncrasies: Any = False
    delay: str | int | float = 0
    sensing_avg: int = 1
    stop_on_collision: bool = False
    stop_at_goal: bool = False
    body_color: tuple[int, int, int] = (255, 255, 255)
    body_filled: bool = False
    catastrophic_collisions: bool = False
    trace_length: tuple[int, int, int] | None = None
    trace_color: tuple[int, int, int] | None = None
    controller: Controller | None = None
    track_io: bool = False
    type: str = ""

    def __post_init__(self):
        # super().__post_init__()
        if self.stop_at_goal is not False:
            raise NotImplementedError  # not tested

    @override
    def __badvars__(self):
        return super().__badvars__() + ["world", "world_config"]

    def attach_world_config(self, world_config):
        self.world = world_config

    @staticmethod
    def from_dict(d):
        # if isinstance(d["sensors"], dict):
        #     d["sensors"] = SensorFactory.create(d["sensors"])
        return MazeAgentConfig(**d)

    def rescale(self, zoom):
        self.agent_radius *= zoom
        if self.sensors is not None:
            for s in self.sensors:
                s.r *= zoom
                s.goal_sensing_range *= zoom
                s.wall_sensing_range *= zoom

    def create(self, **kwargs):
        return MazeAgent(config=self, **kwargs)


class MazeAgent(StaticAgent):
    SEED = -1

    def __init__(self, config: MazeAgentConfig, world, name=None, initialize=True) -> None:

        # if hasattr(config.controller, 'get_actions'):
        #     self.controller = config.controller
        # else:
        #     self.controller = Controller(config.controller)

        self.seed = config.seed
        if config.seed is not None:
            self.set_seed(config.seed)

        super().__init__(config, world, name=name, initialize=False)

        self.radius = config.agent_radius
        self.dt = config.dt
        self.is_highlighted = False
        self.agent_in_sight = None
        if config.idiosyncrasies:
            idiosync = config.idiosyncrasies
            self.idiosyncrasies = [np.random.normal(mean, sd) for mean, sd in zip(idiosync['mean'], idiosync['sd'])]
        else:
            self.idiosyncrasies = [1.0, 1.0]
        # I1_MEAN, I1_SD = 0.93, 0.08
        # I2_MEAN, I2_SD = 0.95, 0.06
        self.delay_1 = st.Delay(delay=config.delay)
        self.delay_2 = st.Delay(delay=config.delay)
        self.sensing_avg = st.Average(config.sensing_avg)
        self.stop_on_collision = config.stop_on_collision
        self.catastrophic_collisions = config.catastrophic_collisions
        self.iD = 0
        self.dead = False
        self.goal_seen = False
        self.stop_at_goal = config.stop_at_goal
        self.config = config

        # self.sensors = deepcopy(config.sensors)
        # for sensor in self.sensors:
        #     if isinstance(sensor, GenomeBinarySensor):
        #         sensor.augment_from_genome(config.controller)

        # self.attach_agent_to_sensors()

        self.body_filled = config.body_filled
        if config.body_color == "Random":
            self.body_color = self.get_random_color()
        else:
            self.body_color = config.body_color

        self.history = []
        self.track_io = getattr(config, "track_io", False)

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    def set_seed(self, seed):
        random.seed(seed)

    @override
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

        v = self.delay_1(v)
        omega = self.delay_2(omega)

        # Define Idiosyncrasies that may occur in actuation/sensing
        # using midpoint rule from https://books.google.com/books?id=iEYnnQeOaaIC&pg=PA29
        self.dtheta = omega * self.idiosyncrasies[-1] * self.dt
        dtheta2 = self.dtheta / 2
        self.iD = abs(v / omega) * 2 if abs(omega) > 1e-9 else float("inf")
        s = 2 * math.sin(dtheta2) * v / omega if abs(omega) > 1e-9 else v * self.dt
        delta = s * self.orientation_uvec(offset=dtheta2) * self.idiosyncrasies

        old_pos = self.pos.copy()

        if self.stopped_duration > 0:
            self.stopped_duration -= 1
            if self.catastrophic_collisions:
                self.dead = True
                self.body_color = (200, 200, 200)
                return
        else:
            self.pos += delta

        self.angle += self.dtheta

        self.collision_flag = False
        if check_for_world_boundaries is not None:
            check_for_world_boundaries(self)

        self.handle_collisions(world)

        # Calculate the 'real' dx, dy after collisions have been calculated.
        # This is what we use for velocity in our equations
        self.dpos = self.pos - old_pos
        # timer = timer.check_watch()

        for sensor in self.sensors:
            sensor.step(world=world)
            if sensor.goal_detected:
                self.goal_seen = True

    @override
    def draw(self, screen, offset=((0, 0), 1.0)) -> None:
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen, offset)
        for sensor in self.sensors:
            sensor.draw(screen, offset)

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
        while sum(rand_color) > 245 * 3:
            rand_color = np.random.choice(256, 3)
        return rand_color

    def handle_collisions(self, world):
        max_attempts = 10
        self.collision_flag = False
        for _ in range(max_attempts):
            candidates = [other for other in world.population if self != other
                               and self.get_aabb().intersects_bb(other.get_aabb())]
            collided = []
            if not candidates:
                break
            for other in candidates:
                collider = self.build_collider()
                other_collider = other.build_collider()
                correction = collider.correction(other_collider)
                if np.isnan(correction).any():
                    continue
                collided.append(other)
                self.collision_flag = True
                self.pos += correction
                if self.catastrophic_collisions:
                    self.dead = True
                    self.body_color = (200, 200, 200)
                    other.dead = True
                    other.body_color = (200, 200, 200)
            if not collided:
                break

            if self.DEBUG:
                world.draw(world._screen_cache)
                pygame.display.flip()
