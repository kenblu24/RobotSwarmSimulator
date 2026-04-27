import numpy as np

from .AbstractController import AbstractController
from ...util.pid import PID

DEFAULT_PID_ARGS = dict(p=0.0, i=0.0, d=0.0, imax=0.0, min=None, max=None)


class WaypointPIDController(AbstractController):
    def __init__(
        self,
        agent=None, parent=None,
        sensor_id=None,
        speed_pid=DEFAULT_PID_ARGS,
        steer_pid=DEFAULT_PID_ARGS,
        static_waypoint=None,
        **kwargs
    ):
        super().__init__(agent=agent, parent=parent, **kwargs)
        self.sensor_id = sensor_id  # use this to determine which sensor on the agent to use
        self.speed_pid = speed_pid if isinstance(speed_pid, PID) else PID(**speed_pid)
        self.steer_pid = steer_pid if isinstance(steer_pid, PID) else PID(**steer_pid)
        self.static_waypoint = static_waypoint

    def get_actions(self, agent):
        if self.sensor_id is not None:
            sensor = agent.sensors[self.sensor_id]
            waypoint = sensor.current_state
        elif self.static_waypoint is not None:
            waypoint = self.static_waypoint
        else:
            raise ValueError("WaypointPIDController must have a sensor_id or static_waypoint")

        if waypoint is None:
            return 0.0, 0.0

        vec = waypoint - agent.pos
        heading = np.arctan2(*vec[::-1]) % (2 * np.pi)
        omega = heading - (agent.angle % (2 * np.pi)) if heading != 0 else 0
        omega %= (2 * np.pi)
        if omega > np.pi:
            omega -= (2 * np.pi)
        v = self.speed_pid(np.linalg.norm(vec))  # * agent.world.dt
        w = self.steer_pid(omega)  # * agent.world.dt
        return v, w
