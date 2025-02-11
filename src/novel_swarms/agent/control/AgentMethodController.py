"""Controller which redirects to a method on the agent.

.. autoclass:: AgentMethodController
    :members:
    :undoc-members:

"""

from .AbstractController import AbstractController
from typing import override


class AgentMethodController(AbstractController):
    """Controller which redirects its ``get_actions()`` call to the agent's ``get_actions()`` method."""

    @override
    def get_actions(self, agent):
        """Redirects to the agent's ``get_actions()`` method."""
        return agent.get_actions()

    @override
    def __str__(self):
        return "get_actions() on Agent"
