from .AbstractController import AbstractController
from typing import override


class AgentMethodController(AbstractController):
    """
    Given agent observations, return agent actions
    """

    @override
    def get_actions(self, agent):
        return agent.get_actions()

    @override
    def __str__(self):
        return "get_actions() on Agent"
