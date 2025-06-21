import numpy as np
import pygame

from swarmsim.agent.control.AbstractController import AbstractController
import swarmsim.util.statistics_tools as st
from math import copysign


trigger_remap = st.Remap([-1, 1], [0, 1])


def decay(x, decay=0.1):
    magnitude = np.clip(abs(x) - decay, 0., None)
    return copysign(magnitude, x)


class HumanController(AbstractController):
    def __init__(
        self, agent=None, parent=None,
        joystick=0,
        keys='wasd',

        speed_range=(-0.3, 0.3),
        turn_range=(-1.5, 1.5),
        key_speed_mult=0.01,
        key_turn_mult=0.1,
        joy_speed_map=None,
        joy_turn_map=None,
        joy_deadzone=0.1,
        trigger_deadzone=0.03,
    ):
        super().__init__(agent=agent, parent=parent)

        pygame.joystick.init()

        self.joy_id = joystick
        self.keys = keys

        self.body_color = (255, 0, 255)
        self.kv, self.kw = 0, 0

        self.key_speed_mult = key_speed_mult
        self.key_turn_mult = key_turn_mult
        self.joy_speed_map = joy_speed_map
        self.joy_turn_map = joy_turn_map
        self.joy_deadzone = st.Deadzone(joy_deadzone)
        self.trigger_deadzone = st.Deadzone(trigger_deadzone)

        self.speed_range = speed_range
        self.turn_range = turn_range

        keys = keys.lower()

        if keys == "wasd":
            self.keymap = {
                '+v': pygame.K_w,
                '-w': pygame.K_a,
                '-v': pygame.K_s,
                '+w': pygame.K_d,
            }
        elif keys == "arrowkeys":
            self.keymap = {
                '+v': pygame.K_UP,
                '-w': pygame.K_LEFT,
                '-v': pygame.K_DOWN,
                '+w': pygame.K_RIGHT,
            }
        elif keys == "ijkl":
            self.keymap = {
                '+v': pygame.K_i,
                '-w': pygame.K_j,
                '-v': pygame.K_k,
                '+w': pygame.K_l,
            }
        else:
            raise ValueError("Invalid keymap specified")

    @property
    def speed_range(self):
        return self._speed_range

    @speed_range.setter
    def speed_range(self, value):
        self._speed_range = value
        self.speed_denorm = st.Remap([-1, 1], value)

    @property
    def turn_range(self):
        return self._turn_range

    @turn_range.setter
    def turn_range(self, value):
        self._turn_range = value
        self.turn_denorm = st.Remap([-1, 1], value)

    @property
    def joy_speed_map(self):
        return self._joy_speed_map

    @joy_speed_map.setter
    def joy_speed_map(self, value):
        if value is None:
            value = [[-1, 1], [-1, 1]]
        self._joy_speed_map = value
        match value:
            case [[*in_points], [*out_points]]:
                self.joy_speed_remap = st.Remap(in_points, out_points)
            case _:
                raise ValueError("joy_speed_map must be a list of two lists")

    @property
    def joy_turn_map(self):
        return self._joy_turn_map

    @joy_turn_map.setter
    def joy_turn_map(self, value):
        if value is None:
            value = [[-1, 1], [-1, 1]]
        self._joy_turn_map = value
        match value:
            case [[*in_points], [*out_points]]:
                self.joy_turn_remap = st.Remap(in_points, out_points)
            case _:
                raise ValueError("joy_turn_map must be a list of two lists")

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
        jv, jw = self.handle_controller()
        kv, kw = self.handle_key_press()

        v = jv + kv
        w = jw + kw

        v = np.clip(v, *self.speed_range)
        w = np.clip(w, *self.turn_range)

        return v, w

    def handle_key_press(self):
        # if self.joystick.get_axis(1) < 0.01 and abs(self.joystick.get_axis(2)) < 0.01 and self.joystick.get_hat(0)[0] == (0, 0)):
        if not self.keys:
            return 0., 0.

        keys = pygame.key.get_pressed()
        m = self.keymap

        if keys[m['+v']]:
            self.kv += self.key_speed_mult
        elif keys[m['-v']]:
            self.kv -= self.key_speed_mult
        else:
            self.kv = decay(self.kv, self.key_speed_mult)

        if keys[m['+w']]:
            self.kw += self.key_turn_mult
        elif keys[m['-w']]:
            self.kw -= self.key_turn_mult
        else:
            self.kw = decay(self.kw, self.key_turn_mult)

        self.kv = np.clip(self.kv, *self.speed_range)
        self.kw = np.clip(self.kw, *self.turn_range)

        return self.kv, self.kw

    def handle_controller(self):
        if self.joystick is None:
            return 0., 0.
        x, y = self.joystick.get_axis(0), -self.joystick.get_axis(1)
        x2, _y2 = self.joystick.get_axis(2), -self.joystick.get_axis(3)
        lt = self.trigger_deadzone(trigger_remap(self.joystick.get_axis(4)))
        rt = self.trigger_deadzone(trigger_remap(self.joystick.get_axis(5)))
        y = self.joy_speed_remap(self.joy_deadzone(y)) + rt - lt
        x = self.joy_turn_remap(self.joy_deadzone(x) + self.joy_deadzone(x2))
        v = self.speed_denorm(y)
        w = self.turn_denorm(x)
        # self.v = -self.joystick.get_axis(1) * vel if abs(self.joystick.get_axis(1)) > 0.01 else [1] * vel
        # self.omega = self.joystick.get_axis(2) * rad if abs(self.joystick.get_axis(2)) > 0.01 else self.joystick.get_hat(0)[0] * rad
        return v, w
