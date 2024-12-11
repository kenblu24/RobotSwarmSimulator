world_types = {}
agent_types = {}










def add_native_agent_types():
    from ..agent.DiffDriveAgent import DiffDriveAgent, DiffDriveAgentConfig
    from ..agent.HumanAgent import HumanDrivenAgent, HumanDrivenAgentConfig
    from ..agent.MillingAgent import MillingAgent, MillingAgentConfig
    from ..agent.StaticAgent import StaticAgent, StaticAgentConfig


def add_native_world_types():
    from ..world.RectangularWorld import RectangularWorld, RectangularWorldConfig

    world_types.["RectangularWorld"] = (RectangularWorld, RectangularWorldConfig)