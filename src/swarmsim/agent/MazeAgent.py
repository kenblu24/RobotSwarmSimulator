"""MazeAgent has Unicycle Dynamics and can move based on sensor info.

.. inheritance-diagram:: swarmsim.agent.MazeAgent.MazeAgent
    :parts: 1

.. autoclass:: swarmsim.agent.MazeAgent.MazeAgentConfig
    :members:
    :inherited-members:

.. autoclass:: swarmsim.agent.MazeAgent.MazeAgent
    :members:
    :inherited-members:
    :undoc-members:

.. py:class:: State

    A :py:class:`typing.NamedTuple` for recording the state of the agent.

    Args:
        x : float
            The agent's x position in the world.
        y : float
            The agent's y position in the world.
        angle : float
            The agent's heading. (radians)

.. py:class:: SPA

    A :py:func:`collections.namedtuple` for recording the **S**\\ tate, **P**\\ erception, **A**\\ ction of the agent.

    Args:
        state : State | tuple[float, ...] | np.ndarray
            The agent's position and heading.
        perception : tuple[Any, ...] | np.ndarray
            The sensor readings.
        action : tuple[float, ...] | np.ndarray
            The action to take.
"""

import math
import random
from copy import deepcopy
from typing import NamedTuple
from dataclasses import dataclass, field
from collections import namedtuple, deque

import pygame
import numpy as np

from ..config import filter_unexpected_fields, associated_type
from .StaticAgent import StaticAgent, StaticAgentConfig
from ..util import statistics_tools as st
from .control.Controller import Controller

# # typing
from typing import Any, override
# from ..world.World import World
# from ..world.RectangularWorld import RectangularWorldConfig

SPA = namedtuple("SPA", ['state', 'perception', 'action'])
State = NamedTuple("State", [('x', float), ('y', float), ('angle', float)])


@associated_type("MazeAgent")
@filter_unexpected_fields
@dataclass
class MazeAgentConfig(StaticAgentConfig):
    # world: World | None = None
    # world_config: RectangularWorldConfig | None = None
    #: list[Sensor]: The sensors used by the agent. Emtpy list by default.
    sensors: list = field(default_factory=list)
    #: AbstractController: The controller used by the agent. Zero controller by default.
    controller: Any = None

    sensing_avg: int = 1
    stop_on_collision: bool = False
    stop_at_goal: bool = False
    body_color: tuple[int, int, int] = (255, 255, 255)
    body_filled: bool = False
    catastrophic_collisions: bool = False
    trace_length: int | None = 0
    trace_color: tuple[int, int, int] = (255, 255, 255)
    controller: Controller | None = None
    track_io: bool = False

    idiosyncrasies: Any = False
    #: dict[str, float]: The idiosyncrasies of the agent. False by default.

    delay: str | int | float = 0

    def __post_init__(self):
        # super().__post_init__()
        if self.stop_at_goal is not False:
            raise NotImplementedError  # not tested

    @override
    def __badvars__(self):
        return super().__badvars__() + ["world", "world_config"]

    @override
    def attach_world_config(self, world_config):
        self.world = world_config

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
        """Agent w/ Unicycle Dynamics which can move based on sensor info.

        Parameters
        ----------
        config : MazeAgentConfig
            Agent will be initialized with this config.
        world : RectangularWorld
            Back-reference to the world instance.
        name : str, optional
            Name of the agent, by default None
        initialize : bool, optional
            If True, run post-init procedure (setup controller and sensors from config).

            Leave this as True if it's the last call to the constructor.

        .. dropdown:: Inheritance Tree
            :color: primary

            .. inheritance-diagram:: swarmsim.agent.MazeAgent.MazeAgent

        .. toggle:: Why is it called ``MazeAgent``?

            You might be wondering why it's called ``MazeAgent``, or rather, why
            this agent is so commonly used. It's also the subclass of a lot of other
            agent types, such as the nearly identical :py:class:`UnicycleAgent <swarmsim.agent.UnicycleAgent>`.
            So why not have ``UnicycleAgent`` be the base?

            The reason is largely historical. One of the research directions was to
            discover behaviors that could allow a swarm to navigate a maze faster.
            Research then shifted to multi-agent search-and-rendezvous, but this
            being so early in the sim's development, the name ``MazeAgent`` had
            been developed just enough to be used as a base for other agent types.
        """

        super().__init__(config, world, name=name, initialize=False)

        self.is_highlighted = False
        self.agent_in_sight = None
        if config.idiosyncrasies:
            idiosync = config.idiosyncrasies
            self.idiosyncrasies = [np.random.normal(mean, sd) for mean, sd in zip(idiosync['mean'], idiosync['sd'])]
        else:
            self.idiosyncrasies = [1.0, 1.0]
        # I1_MEAN, I1_SD = 0.93, 0.08
        # I2_MEAN, I2_SD = 0.95, 0.06
        self.delay_1 = st.Delay(delay=config.delay)  # type: ignore[reportArgumentType]
        self.delay_2 = st.Delay(delay=config.delay)  # type: ignore[reportArgumentType]
        self.sensing_avg = st.Average(config.sensing_avg)
        self.stop_on_collision = config.stop_on_collision
        self.catastrophic_collisions = config.catastrophic_collisions
        self.iD = 0
        self.dead = False
        self.goal_seen = False
        self.stop_at_goal = config.stop_at_goal
        self.config = config

        self.body_filled = config.body_filled
        if config.body_color == "Random":
            self.body_color = self.get_random_color()
        else:
            self.body_color = config.body_color

        # Set Trace Settings if a trace was assigned to this object.
        self.trace_color = config.trace_color
        self.trace_path = deque(maxlen=config.trace_length)

        self.history = []
        self.track_io = getattr(config, "track_io", False)

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    @override
    def step(self, world=None, check_for_world_boundaries=None, check_for_agent_collisions=None) -> None:
        world = world or self.world
        if world is None:
            raise Exception("Expected a Valid value for 'World' in step method call - Unicycle Agent")

        # timer = Timer("Calculations")
        super().step(world)

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
            sensor_state = self.sensors[0].current_state
            self.history.append(SPA(
                State(*self.pos, self.angle),
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
            # TODO: remove this
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
        pan, zoom = np.asarray(offset[0]), offset[1]  # type:ignore[reportUnusedVariable]
        super().draw(screen, offset)
        for sensor in self.sensors:
            sensor.draw(screen, offset)

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

    def handle_collisions(self, world, max_attempts=10, nudge_amount=1.0, rng=None, refresh=False):
        if rng is None:
            rng = self.rng
        self.collision_flag = False
        for _i in range(max_attempts):
            if refresh:
                self.aabb = self.make_aabb()
            candidates = [other for other in world.population if self != other
                               and self.aabb.intersects_bb(other.make_aabb() if refresh else other.aabb)]
            collided = []
            if not candidates:
                break
            collider = self.build_collider()
            for other in candidates:
                other_collider = other.build_collider()
                correction = collider.correction(other_collider, rng=rng) * nudge_amount
                if np.isnan(correction).any():
                    continue
                collided.append(other)
                self.collision_flag = True
                self.pos += correction
                collider = self.build_collider()  # refresh collider

                if self.catastrophic_collisions:
                    self.dead = True
                    self.body_color = (200, 200, 200)
                    other.dead = True
                    other.body_color = (200, 200, 200)

            if not collided:
                return
            if self.debug and world._screen_cache:
                world.draw(world._screen_cache)
                pygame.display.flip()

        else:
            return True
