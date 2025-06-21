import numpy as np

from dataclasses import dataclass, field
from ..config import filter_unexpected_fields, associated_type
from .MazeAgent import MazeAgent, MazeAgentConfig, SPA, State

# typing
from typing import Any, override


@associated_type("DroneAgent")
@filter_unexpected_fields
@dataclass
class DroneAgentConfig(MazeAgentConfig):
    wheel_radius: float = 0.01


class DroneAgent(MazeAgent):

    SEED = -1
    DEBUG = False

    def __init__(self, config: DroneAgentConfig, world,
                 name=None, initialize=True) -> None:
        super().__init__(config, world, name=name, initialize=False)

        self.wheel_radius = config.wheel_radius

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    @override
    def step(self, world=None, check_for_world_boundaries=None, check_for_agent_collisions=None) -> None:
        # This code is largely the same as that of MazeAgent, but we assume
        # the controller returns two wheelspeeds, vl, vr instead of v, omega
        # and convert said wheelspeeds to v, omega for the unicycle agent
        world = world or self.world
        if world is None:
            raise Exception("Expected a Valid value for 'World' in step method call - Unicycle Agent")

        # timer = Timer("Calculations")
        super().step(world, check_for_world_boundaries=check_for_world_boundaries,
                     check_for_agent_collisions=check_for_agent_collisions)

        if self.dead:
            return

        if world.goals and world.goals[0].agent_achieved_goal(self) or self.detection_id == 2:
            if self.stop_at_goal:
                delta_x, delta_y, da = 0, 0, 0
            else:
                delta_x, delta_y, da = self.controller.get_actions(self)
            # self.detection_id = 2
            # self.set_color_by_id(3)
        else:
            delta_x, delta_y, da = self.controller.get_actions(self)

        if self.track_io:
            sensor_state = self.sensors[0].current_state
            self.history.append(SPA(
                State(*self.pos, self.angle),
                sensor_state,
                (delta_x, delta_y, da),
            ))

        self.dx = delta_x * np.cos(self.angle) - delta_y * np.cos(self.angle)
        self.dy = delta_x * np.sin(self.angle) - delta_y * np.sin(self.angle)

        self.dtheta = da * self.idiosyncrasies[-1] * self.dt
        orient = self.orientation_uvec(offset=self.angle)
        delta = delta_x * orient - delta_y * orient
        delta *= self.idiosyncrasies

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

    def simulate_error(self, err_type="Death"):
        if err_type == "Death":
            self.controller = [0 for _ in self.controller]
            self.body_color = (255, 0, 0)
            self.body_filled = True
        elif err_type == "Divergence":
            self.controller = [1, 1, 1, 1]
            self.body_color = (255, 0, 0)
            self.body_filled = True
