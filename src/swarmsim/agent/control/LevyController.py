import math

import numpy as np
from scipy.stats import levy, gamma

from .AbstractController import AbstractController

# typing
from typing import override


class LevyController(AbstractController):
    def __init__(self,
        velocity: float,
        angular_velocity: float,
        agent=None,
        parent=None,
        levy_constant=None,
        curving=False,
        timeout=5.0,
        timeout_steps=None,
        step_scale=1.0,
    ):
        super().__init__(parent)
        if levy_constant is None:
            self.levy_dist_index = self.agent.rng.random() + 1
        else:
            self.levy_dist_index = levy_constant

        self.sigma_u = self._sigma(self.levy_dist_index)
        self.sigma_v = 1

        self.forward_rate = velocity
        self.turning_rate = angular_velocity

        self.curve_based = curving
        if timeout_steps is not None:
            self.mode_max_time = timeout_steps
        else:
            self.mode_max_time = timeout / self.agent.dt

        self.turning = True
        self.v = self.forward_rate
        self.omega = self.turning_rate
        self.X_from_levy = 0

        # Initialize the number of steps within the levy segment left to walk
        self.steps_left = 0
        self.step_scaling = step_scale

    @override
    def as_config_dict(self):
        return {
            # "velocity": self.velocity,
            # "angular_velocity": self.angular_velocity,
            # "levy_constant": self.levy_index,
            # "curving": self.curving,
            # "timeout": self.timeout,
            # "step_scale": self.step_scale,
        }

    @override
    def get_actions(self, agent):
        super().get_actions(agent)

        self.steps_left -= 1

        if self.agent.collision_flag:
            self.steps_left = 0

        if self.steps_left <= 0:
            if self.turning:
                self.new_foward_steps()
            else:
                self.levy_sample()
                self.new_heading()

        return self.v, self.omega

    def new_heading(self):
        if self.curve_based:
            self.steps_left = int(self.X_from_levy / 2)
            self.omega = self.turning_rate
            self.v = self.forward_rate
            self.turning = True
        else:
            d_theta = (self.agent.rng.random() * 2 * np.pi) - np.pi
            self.steps_left = abs(d_theta // self.turning_rate) + 1
            if d_theta < 0:
                self.omega = -self.turning_rate
            else:
                self.omega = self.turning_rate
            self.v = 0

    def levy_sample(self):
        # self.X_from_levy = min(int(levy.rvs(loc=0, scale=1.0)), 1000)
        l_sample = self.mode_max_time + 1
        while l_sample > self.mode_max_time:
            l_sample = round(100 * (1 / (gamma.rvs(a=0.5, scale=2))))
        self.X_from_levy = l_sample

    def new_foward_steps(self):
        if self.curve_based:
            self.steps_left = int(self.X_from_levy / 2)
            self.omega = 0
            self.v = self.forward_rate
            self.turning = False
        else:
            step = self.sample_step_size() * self.step_scaling
            self.steps_left = (step // self.forward_rate) + 1
            self.omega = 0
            self.v = self.forward_rate

    def sample_step_size(self):
        u = np.random.normal(0, np.power(self.sigma_u, 2))
        v = np.random.normal(0, np.power(self.sigma_v, 2))
        s = u / np.power(np.abs(v), 1 / self.levy_dist_index)
        return s

    def _sigma(self, beta):
        numer = (self._gamma(1 + beta) * np.sin(np.pi * (beta / 2)))
        denom = (self._gamma((1 + beta) / 2) * beta * np.power(2, ((beta - 1) / 2)))
        return np.power((numer / denom), 1 / beta)

    def _gamma(self, z):
        # from scipy.special import gamma as g
        return math.gamma(z)
