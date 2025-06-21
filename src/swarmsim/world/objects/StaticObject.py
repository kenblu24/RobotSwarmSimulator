"""Module for StaticObject class.

StaticObject is a class for objects that do not move.

It is a subclass of :py:class:`~swarmsim.agent.StaticAgent.StaticAgent`
and is basically the same except it does not ``step()`` or
:py:meth:`~swarmsim.agent.StaticAgent.StaticAgent.draw_direction`\\ .

Object Config Class
-------------------

.. autoclass:: StaticObjectConfig
    :members:
    :undoc-members:
    :inherited-members:

Object Class
------------

.. autoclass:: StaticObject
    :members:
    :undoc-members:

"""

from dataclasses import dataclass, field
from ...config import filter_unexpected_fields, associated_type

from ...agent.StaticAgent import StaticAgent, StaticAgentConfig

# typing
from typing import Any, override


@associated_type("StaticObject")
@filter_unexpected_fields
@dataclass
class StaticObjectConfig(StaticAgentConfig):
    collides: bool | int = True
    grounded: bool = True


class StaticObject(StaticAgent):
    @override
    def step(self, check_for_world_boundaries=None, world=None, check_for_agent_collisions=None) -> None:
        pass

    @override
    def draw_direction(self, screen, offset=((0, 0), 1.0)):
        pass
