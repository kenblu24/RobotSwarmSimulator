"""This is the root swarmsim module.

See the :doc:`/api/index` for a description of the layout of this module.

Most code in the package is organized into submodules, but some commonly used
functions and classes are available directly in the root module.
This page lists those.


.. currentmodule:: swarmsim

Functions
=========

.. autosummary::

   ~config.associated_type
   ~config.filter_unexpected_fields
   ~config.register_agent_type
   ~config.register_world_type
   ~config.register_dictlike_namespace
   ~config.register_dictlike_type
   ~world.World.config_from_dict
   ~world.World.config_from_yaml
   ~world.World.config_from_yamls
   ~world.World.world_from_config
   run_sim

Classes
=======

.. autosummary::

   ~world.World.World
   ~world.World.BaseWorldConfig
   ~world.RectangularWorld.RectangularWorld
   ~world.RectangularWorld.RectangularWorldConfig
   ~agent.Agent.Agent
   ~agent.Agent.BaseAgentConfig
   ~agent.StaticAgent.StaticAgent
   ~agent.StaticAgent.StaticAgentConfig
   ~agent.MazeAgent.MazeAgent
   ~agent.MazeAgent.MazeAgentConfig
   ~sensors.Sensor.Sensor
   ~agent.control.Controller.Controller
   ~world.spawners.Spawner.Spawner
   ~world.spawners.AgentSpawner.PointAgentSpawner
   ~world.spawners.AgentSpawner.UniformAgentSpawner
   ~util.collider.CollisionMode.CollisionMode

Functions in this module
========================

.. autofunction:: swarmsim.print_debugversions


"""


from .world.World import config_from_dict, config_from_yaml, config_from_yamls, world_from_config
from .world.World import World, BaseWorldConfig
from .world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from .agent.Agent import Agent, BaseAgentConfig
from .agent.StaticAgent import StaticAgent, StaticAgentConfig
from .agent.MazeAgent import MazeAgent, MazeAgentConfig
from .agent.control.Controller import Controller
from .sensors.Sensor import Sensor
from .world.spawners.Spawner import Spawner
from .world.spawners.AgentSpawner import AgentSpawner, PointAgentSpawner, UniformAgentSpawner
from .config import associated_type, filter_unexpected_fields
from .config import register_agent_type, register_world_type, register_dictlike_namespace, register_dictlike_type
from .util.collider.CollisionMode import CollisionMode
from .world.simulate import main as run_sim

__all__ = [
    # ---
    'associated_type',
    'filter_unexpected_fields',
    'register_agent_type',
    'register_world_type',
    'register_dictlike_namespace',
    'register_dictlike_type',
    # ---
    'config_from_dict',
    'config_from_yaml',
    'config_from_yamls',
    'world_from_config',
    # ---
    'World',
    'BaseWorldConfig',
    'RectangularWorld',
    'RectangularWorldConfig',
    # ---
    'Agent',
    'BaseAgentConfig',
    'StaticAgent',
    'StaticAgentConfig',
    'MazeAgent',
    'MazeAgentConfig',
    # ---
    'Sensor',
    # ---
    'Controller',
    # ---
    'Spawner',
    'AgentSpawner',
    'PointAgentSpawner',
    'UniformAgentSpawner',
    # ---
    'CollisionMode',
    # ---
    'run_sim',
    # ---
    'print_debugversions',
    'version',
]


# get version from package metadata (pyproject.toml)
# will only work if package is installed
# NB: this number only changes when the package is pip installed/updated
try:
    import importlib.metadata
    __version__ = importlib.metadata.version(__package__ or "swarmsim")
except (ImportError, StopIteration):
    __version__ = "unknown"


version = __version__


def print_debugversions():
    """Prints the versions of the operating system and Python."""
    import platform
    import numpy
    import scipy
    print(f"RobotSwarmSimulator: {__version__}")
    print(f"OS: {platform.platform()}")
    print(f"Python: {platform.python_version()}")
    print(f"Numpy: {numpy.__version__}")
    print(f"Scipy: {scipy.__version__}")
