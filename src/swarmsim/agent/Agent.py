"""Base Agent Config and Class to be inherited by all agents.

.. rubric:

    Inheritance Diagram

.. inheritance-diagram:: swarmsim.agent.Agent.Agent
    :parts: 1

.. autoclass:: swarmsim.agent.Agent.BaseAgentConfig
    :members:
    :inherited-members:
    :undoc-members:

.. autoclass:: swarmsim.agent.Agent.Agent
    :members:
    :inherited-members:
    :undoc-members:
"""

import math
import copy
from dataclasses import dataclass, field

import numpy as np

from ..agent.control.StaticController import zero_controller
from ..config import get_class_from_dict, filter_unexpected_fields

# typing
from typing import Any
from ..util.collider.AABB import AABB


@filter_unexpected_fields
@dataclass
class BaseAgentConfig:
    position: tuple[float, ...] | np.ndarray = (0, 0)
    angle: float | Any = 0.
    name: str | Any = None
    controller: Any | None = None
    grounded: bool = False
    collides: bool | int = False
    sensors: list = field(default_factory=list)
    team: str | None = None

    # def __post_init__(self):
    #     if self.stop_at_goal is not False:
    #         raise NotImplementedError  # not tested

    def as_dict(self):
        return self.asdict()

    def as_config_dict(self):
        return self.asdict()

    def asdict(self):
        return dict(self.as_generator())

    @classmethod
    def from_dict(cls, env):
        return cls(**env)

    def __badvars__(self):
        return []

    def as_generator(self):
        for key, value in self.__dict__.items():
            if any(key == bad for bad in self.__badvars__()):
                continue
            if hasattr(value, "asdict"):
                yield key, value.asdict()
            elif hasattr(value, "as_dict"):
                yield key, value.as_dict()
            elif hasattr(value, "as_config_dict"):
                yield key, value.as_config_dict()
            else:
                yield key, value


class Agent:

    _always_shallow_copy = ["world"]

    def __init__(self, config, world, name=None, group=0, initialize=True) -> None:
        self.marked_for_deletion = False
        #: Agent config.
        self.config = config
        #: Agent's position in the world.
        self.pos: np.ndarray = np.asarray(config.position, dtype=np.float64)
        #: Agent's name.
        self.name: str | None = name or config.name
        #: Change in position since last step
        self.dpos: np.ndarray = np.zeros(len(self.pos))
        #: Agent's heading in radians. 0 is facing frame right.
        self.angle = config.angle
        #: Change in heading since last step.
        self.dtheta = 0
        #: List of this agent's sensors.
        self.sensors: list = []
        #: The :py:class:`Controller <swarmsim.agent.control.Controller>` for this agent.
        self.controller = zero_controller(2)
        #: Colliders should set to True if a collision was detected.
        self.collision_flag = False
        #: If True, the agent should be solid.
        self.collides = config.collides
        self.stop_on_collision = False
        self.stopped_duration = 0
        self.detection_id = 0
        #: The agent's :py:class:`~swarmsim.util.collider.AABB`.
        #: This gets built at least once on init.
        self.aabb = None
        self.group = group
        #: Back-reference to the world instance.
        self.world = world
        #: If True, the agent should never change position or be pushed during collision resolution.
        self.grounded = config.grounded
        #: The agent's team.
        self.team = config.team

        if initialize:
            self.setup_controller_from_config()
            self.setup_sensors_from_config()

    def setup_controller_from_config(self):
        """Creates and adds the :py:attr:`AgentConfig.controller` to the agent.

        If ``config.controller`` is a dict with a ``type`` key, then it is created
        through the :doc:`/guide/config` and set as the agent's :py:attr:`~swarmsim.agent.Agent.Agent.controller`.
        :py:class:`~swarmsim.agent.control.AbstractController` instances copied and added to the list.

        The controller is also given a back-reference to this agent via its
        :py:meth:`~swarmsim.agent.control.AbstractController.set_agent`.

        Raises
        ------
        TypeError
            if the ``element`` is not a dict or :py:class:`AbstractSensor` instance.
        """
        if not self.config.controller:
            return
        # if it's already a controller, just add it
        from ..agent.control.AbstractController import AbstractController
        if isinstance(self.config.controller, AbstractController):
            self.controller = copy.copy(self.config.controller)
            if self.controller.agent is None:
                self.controller.set_agent(self)
            return
        if isinstance(self.config.controller, type):
            raise TypeError("Expected a config dict or AbstractController instance but got a class instead.")
        # otherwise, it's a config dict. find the class specified and create the controller
        if not isinstance(self.config.controller, dict):
            msg = f'Tried to setup controller, but {repr(self.config.controller)} is not a dict or subclass of AbstractController'  # noqa: E501
            raise Exception(msg)
        res = get_class_from_dict('controller', self.config.controller)
        if not res:
            return
        controller_cls, controller_config = res
        self.controller = controller_cls(agent=self, **controller_config)
        self.controller: AbstractController

    def setup_sensors_from_config(self):
        """Creates and adds sensors from the :py:attr:`AgentConfig.sensors` to the agent.

        For each ``element`` of ``config.sensors``, if the ``element`` is a
        dict with a ``type`` key, then it is created through the :doc:`/guide/config`
        and then added to the agent's ``sensors`` list. :py:class:`AbstractSensor` instances
        copied and added to the list.

        The new sensor instances are also given a back-reference to this agent via their
        :py:meth:`~swarmsim.sensors.AbstractSensor.set_agent`.

        Raises
        ------
        TypeError
            if the ``element`` is not a dict or :py:class:`AbstractSensor` instance.
        """
        from ..sensors.AbstractSensor import AbstractSensor
        for sensor_config in self.config.sensors:
            # if it's already a sensor, just add it
            if isinstance(sensor_config, AbstractSensor):
                sensor = copy.copy(sensor_config)
                self.sensors.append(sensor)
                if sensor.agent is None:
                    sensor.set_agent(self)
                continue
            if isinstance(sensor_config, type):
                raise TypeError("Expected a config dict or AbstractSensor instance but got a class instead.")
            # otherwise, it's a config dict. find the class specified and create the sensor
            sensor_cls, sensor_config = get_class_from_dict('sensors', sensor_config)
            self.sensors.append(sensor_cls(agent=self, **sensor_config))

    def step(self, *args, **kwargs) -> None:
        self.pos = np.asarray(self.pos, dtype='float64')

    def draw(self, screen, offset=((0, 0), 1.0)) -> None:
        pass

    def get_sensors(self):
        """Alias for :py:attr:`~swarmsim.agent.Agent.Agent.sensors`."""
        return self.sensors

    def getPosition(self):
        """Alias for :py:attr:`~swarmsim.agent.Agent.Agent.pos`."""
        return np.asarray(self.pos, dtype='float64')

    def getVelocity(self):
        """Alias for :py:attr:`~swarmsim.agent.Agent.Agent.dpos`."""
        return np.asarray(self.dpos, dtype='float64')

    @property
    def position(self):
        """Alias for :py:attr:`~swarmsim.agent.Agent.Agent.pos`."""
        return self.pos

    @position.setter
    def position(self, new_pos: np.ndarray[(2,), np.dtype[float]] | tuple[float, float]):
        self.pos = np.asarray(new_pos, dtype='float64')

    def orientation_uvec(self, offset=0.):
        """Returns the agent's 2D orientation matrix.

        This is a unit vector pointing in the direction of the agent's
        heading plus an offset.

        Equivalent to

        .. code-block:: python

            np.array([
                math.cos(self.angle + offset),
                math.sin(self.angle + offset)
            ], dtype=np.float64)

        Parameters
        ----------
        offset : float, default=0
            The offset to add to the agent's heading (radians).

        Returns
        -------
        numpy.ndarray (2,)
            The 2D orientation matrix.
        """
        return np.array([
            math.cos(self.angle + offset),
            math.sin(self.angle + offset)
        ], dtype=np.float64)

    def getFrontalPoint(self, offset=0) -> tuple:
        # """Returns the location on the circumference that represents the 'front' of the robot."""
        return self.pos + self.orientation_uvec(offset)

    def attach_agent_to_sensors(self):
        for sensor in self.sensors:
            sensor.parent = self

    def get_aabb(self) -> AABB:
        """Get the agent's :py:class:`AABB <swarmsim.util.collider.AABB>`."""
        pass

    def get_x_pos(self) -> float:
        return self.pos[0]

    def get_y_pos(self) -> float:
        return self.pos[1]

    def set_x_pos(self, new_x: float):
        self.pos[0] = new_x

    def set_y_pos(self, new_y: float):
        self.pos[1] = new_y

    def get_heading(self) -> float:
        return self.angle

    def set_heading(self, new_heading: float):
        """Set the agent's heading (radians)."""
        self.angle = new_heading

    def on_key_press(self, event):
        pass

    def get_name(self):
        return self.name

    def set_name(self, new_name: str | None | Any):
        """Set the name of the agent.

        Parameters
        ----------
        new_name : str | None | Any
            The new name of the agent. Strongly recommended to use ``str``.
        """
        self.name = new_name

    def set_pos_vec(self, vec):
        """Set the ``x``, ``y``, and ``angle`` of the agent.

        Parameters
        ----------
        vec : tuple[float, float, float] | numpy.ndarray
            A vector with ``len()`` of ``3`` containing the ``x, y, angle``
        """
        self.set_x_pos(vec[0])
        self.set_y_pos(vec[1])
        self.set_heading(vec[2])

    @classmethod
    def from_config(cls, config, world):
        """Returns a new agent instance from the given config and world.

        Parameters
        ----------
        config : AgentConfig
            The config to create the agent from.
        world : World
            This provides a back-reference to the world instance for the new agent.

        Returns
        -------
        Agent
            The new agent has the same type as the class/instance it was called from.
        """
        return cls(config, world)

    def copy(self):
        """Create a copy of this agent.


        Returns
        -------
        Agent
            The new agent has the same type as the class/instance it was called from.


        Almost all attributes are deep-copied from this agent. However, some attributes
        do not get recursively deep-copied, such as the ``world`` attribute.

        The :py:attr:`Agent._always_shallow_copy` attribute is a list of agent attribute names
        which determines which attributes do not get deep-copied. By default, this is ``['world']``.

        On the new agent, Attributes of **this** agent not in this list will be a deep-copy of those in the original agent.

        Attributes of **this** agent in this list will share the same reference as the original agent.

        Examples
        --------
        >>> agent = Agent(config, world)
        >>> agent.copy().world is agent.world
        True
        >>> copy.deepcopy(agent).world is agent.world
        False

        """
        cls = self.__class__
        result = cls.__new__(cls)
        memo = {}
        memo[id(self)] = result
        for key, value in self.__dict__.items():
            if key in cls._always_shallow_copy:
                setattr(result, key, value)  # keep reference to same world, etc.
            else:
                setattr(result, key, copy.deepcopy(value, memo))
        return result
