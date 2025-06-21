"""Lazy initialization of built-in types

Holds registry of classes for the initialization system.
prior to first use. This is to avoid circular imports.

These modules can then be accessed via the :py:data:`.store` object
and the getting functions in this module
for use with initialization from ``.yaml`` files or config objects.

It also provides the interface for registering third-party types/classes.

.. seealso::
    :doc:`/guide/config_store_api`

.. autoclass:: LazyKnownModules
    :members: world_types, agent_types, dictlike_types, initialized_natives

.. autodata:: swarmsim.config.store

Functions
=========

.. currentmodule:: swarmsim.config

.. autofunction:: get_agent_class
.. autofunction:: get_class_from_dict
.. autofunction:: register_agent_type
.. autofunction:: register_world_type
.. autofunction:: register_dictlike_namespace
.. autofunction:: register_dictlike_type
.. autofunction:: initialize_natives

Decorators
==========

.. autofunction:: associated_type
.. autofunction:: filter_unexpected_fields

.. seealso::
   There is no ``swarmsim.config.get_world_class`` function,
   world type lookup is handled inside :py:mod:`~swarmsim.world.World.World_from_config`.

"""

from dataclasses import fields


def associated_type(type_name: str):
    """Decorate a config dataclass to add an ``associated_type`` field

    Normally, dataclasses will raise an error if you try to create
    an instance with an argument that is not in the dataclass.

    The decorator cause the dataclass to detect unexpected arguments
    and set them as attributes on the config object.

    Examples
    --------

    .. code-block:: python
       :caption: MyAgentConfig.py

       from dataclasses import dataclass
       from swarmsim.config import associated_type, filter_unexpected_fields

       @associated_type("MyAgent")
       @filter_unexpected_fields  # optional
       @dataclass
       class MyAgentConfig:
           my_custom_field: int = 999

       config = MyAgentConfig()
       config.associated_type == "MyAgent"  # True

    """

    def decorator(cls):
        original_init = cls.__init__

        def new_init(self, *args, **kwargs):
            self.associated_type = type_name
            original_init(self, *args, **kwargs)

        cls.__init__ = new_init
        return cls

    return decorator


def filter_unexpected_fields(cls):
    """Decorate a dataclass to filter out unexpected fields

    Normally, dataclasses will raise an error if you try to create
    an instance with an argument that is not in the dataclass.

    The decorator cause the dataclass to detect unexpected arguments
    and set them as attributes on the config object.

    Examples
    --------

    .. code-block:: python

       @filter_unexpected_fields
       @dataclass
       class MyAgentConfig:
           my_custom_field: int = 999

       config = MyAgentConfig(unexpected_field="hello")
       config.unexpected_field == "hello"  # True

    """

    original_init = cls.__init__

    def new_init(self, *args, **kwargs):
        expected_fields = {field.name for field in fields(cls)}
        cleaned_kwargs = {key: value for key, value in kwargs.items() if key in expected_fields}
        unexpected_kwargs = {key: value for key, value in kwargs.items() if key not in expected_fields}
        original_init(self, *args, **cleaned_kwargs)
        for key, value in unexpected_kwargs.items():
            setattr(self, key, value)

    cls.__init__ = new_init
    return cls


class LazyKnownModules:
    """Holds the registry of known classes

    This class is used internally by the config system.

    .. caution:: Users should avoid adding new entries directly and instead use the functions in
        the :py:mod:`~swarmsim.config` module.

    The registry will be initialized with all the built-in classes
    just before the first access of any of its known types.
    This is to avoid circular imports.
    """
    def __init__(self):
        self._world_types = {}
        self._agent_types = {}
        self._dictlike_types = {}
        self._controllers = {}
        self._behaviors = {}
        #: True after the registry has been initialized.
        self.initialized_natives = False

    @property
    def world_types(self):
        self.initialize_natives()
        return self._world_types

    @property
    def agent_types(self):
        self.initialize_natives()
        return self._agent_types

    @property
    def dictlike_types(self):
        self.initialize_natives()
        return self._dictlike_types

    def add_dictlike_namespace(self, key: str):
        if key not in self._dictlike_types:
            self._dictlike_types[key] = {}

    def initialize_natives(self):
        if self.initialized_natives:
            return
        self.initialized_natives = True
        self.add_native_world_types()
        self.add_native_agent_types()
        self.add_native_sensors()
        self.add_native_controllers()
        self.add_native_metrics()
        self.add_native_spawners()

    def add_native_world_types(self):
        from ..world.RectangularWorld import RectangularWorld, RectangularWorldConfig

        self._world_types['RectangularWorld'] = (RectangularWorld, RectangularWorldConfig)

    def add_native_sensors(self):
        from ..sensors.BinaryFOVSensor import BinaryFOVSensor
        from ..sensors.BinaryLOSSensor import BinaryLOSSensor
        from ..sensors.GenomeDependentSensor import GenomeBinarySensor
        from ..sensors.RegionalSensor import RegionalSensor
        from ..sensors.StaticSensor import StaticSensor

        self.add_dictlike_namespace('sensors')

        self._dictlike_types['sensors']['BinaryFOVSensor'] = BinaryFOVSensor
        self._dictlike_types['sensors']['BinaryLOSSensor'] = BinaryLOSSensor
        self._dictlike_types['sensors']['GenomeBinarySensor'] = GenomeBinarySensor
        self._dictlike_types['sensors']['RegionalSensor'] = RegionalSensor
        self._dictlike_types['sensors']['StaticSensor'] = StaticSensor

    def add_native_controllers(self):
        from ..agent.control.Controller import Controller
        from ..agent.control.StaticController import StaticController
        from ..agent.control.BinaryController import BinaryController
        from ..agent.control.AgentMethodController import AgentMethodController
        from ..agent.control.HomogeneousController import HomogeneousController

        self.add_dictlike_namespace('controller')

        self._dictlike_types['controller']['Controller'] = Controller
        self._dictlike_types['controller']['StaticController'] = StaticController
        self._dictlike_types['controller']['BinaryController'] = BinaryController
        self._dictlike_types['controller']['AgentMethodController'] = AgentMethodController
        self._dictlike_types['controller']['HomogeneousController'] = HomogeneousController

    def add_native_metrics(self):
        from .. import metrics

        self.add_dictlike_namespace('metrics')

        native_metrics = {name: getattr(metrics, name) for name in metrics.__all__}
        self._dictlike_types['metrics'].update(native_metrics)

    def add_native_agent_types(self):

        # actual agents
        from ..agent.DiffDriveAgent import DifferentialDriveAgent, DifferentialDriveAgentConfig
        # from ..agent.HumanAgent import HumanDrivenAgent, HumanDrivenAgentConfig
        from ..agent.StaticAgent import StaticAgent, StaticAgentConfig
        from ..agent.MazeAgent import MazeAgent, MazeAgentConfig

        self._agent_types['MazeAgent'] = (MazeAgent, MazeAgentConfig)
        self._agent_types['DiffDriveAgent'] = (DifferentialDriveAgent, DifferentialDriveAgentConfig)
        # self._agent_types['HumanDrivenAgent'] = (HumanDrivenAgent, HumanDrivenAgentConfig)
        self._agent_types['StaticAgent'] = (StaticAgent, StaticAgentConfig)

        # world objects also use the agent creation system
        from ..world.objects.StaticObject import StaticObject, StaticObjectConfig
        from ..world.objects.DetectionRegion import DetectionRegion, DetectionRegionConfig
        # from ..world.objects.TriggerRegion import TriggerRegion, TriggerRegionConfig

        self._agent_types['StaticObject'] = (StaticObject, StaticObjectConfig)
        self._agent_types['DetectionRegion'] = (DetectionRegion, DetectionRegionConfig)
        # self._agent_types['TriggerRegion'] = (TriggerRegion, TriggerRegionConfig)

    def add_native_spawners(self):
        from ..world.spawners.AgentSpawner import AgentSpawner, UniformAgentSpawner, PointAgentSpawner
        from ..world.spawners.ExcelSpawner import ExcelSpawner

        self.add_dictlike_namespace('spawners')

        self._dictlike_types['spawners']['AgentSpawner'] = AgentSpawner
        self._dictlike_types['spawners']['ExcelSpawner'] = ExcelSpawner
        self._dictlike_types['spawners']['PointAgentSpawner'] = PointAgentSpawner
        self._dictlike_types['spawners']['PointAgentSpawner'] = PointAgentSpawner
        self._dictlike_types['spawners']['UniformAgentSpawner'] = UniformAgentSpawner


#: Holds the registry of known classes.
#: Instance of :py:class:`~swarmsim.config.LazyKnownModules`
store = LazyKnownModules()


def register_world_type(name: str, world_type, world_config=None):
    """Register a world class with the config system

    Parameters
    ----------
    name : str
        The name of the world type, i.e. 'RectangularWorld'

        We recommend choosing it as the name of the ``world`` class, i.e. ``world_type.__name__``
    world_type : _type_
        The ``world`` class to register
    world_config : _type_
        The world config dataclass type to register
    """
    store.world_types[name] = (world_type, world_config)


def register_agent_type(name: str, agent_type, agent_config=None):
    """Register an agent class with the config system

    Parameters
    ----------
    name : str
        The name of the agent type, i.e. 'MazeAgent'

        We recommend choosing it as the name of the ``agent`` class, i.e. ``agent_type.__name__``
    agent_type : _type_
        The ``agent`` class to register
    agent_config : _type_
        The agent config dataclass type to register
    """
    store.agent_types[name] = (agent_type, agent_config)


def register_dictlike_namespace(key: str):
    """Register a new namespace for classes configured using dicts

    Parameters
    ----------
    key : str
        The namespace to register, i.e. 'controller', 'spawner', etc.
    """
    store.add_dictlike_namespace(key)


def register_dictlike_type(key: str, name: str, cls):
    """Register a class for a given namespace

    Parameters
    ----------
    key : str
        The namespace to register, i.e. 'controller', 'spawner', etc.
    name : str
        The name of the class, i.e. 'StaticController'
    cls : _type_
        The class to register, i.e. ``StaticController`` itself
    """
    store.add_dictlike_namespace(key)
    store.dictlike_types[key][name] = cls


_ERRMSG_MISSING_ASSOCIATED_TYPE = """
Expected this config to have an associated_type field.
Use @swarmsim.config.associated_type(ClassNameHere) on the config dataclass.
"""


def get_agent_class(config):
    """Retrieve the :py:mod:`~swarmsim.agent` class from the agent type registry

    Parameters
    ----------
    config : dict or config dataclass
        A config dict with a ``type`` entry, or a config dataclass with a ``associated_type`` field.

    Returns
    -------
    type
        The agent class which will be used to create the agent
    AgentConfig
        The config

    Raises
    ------
    AttributeError
        ``associated_type`` field not found on config dataclass or ``type`` entry not found in config dict
    KeyError
        Agent type not found in registry
    TypeError
        Found an entry in the registry for the requested agent type,
        but it's not the correct format of ``(agent_class, agent_config_class)``

    Examples
    --------

    .. code-block:: python

       cls, config = get_agent_class(config)
       agent = cls(config, world)

    """

    # get the type name
    if isinstance(config, dict):  # if it's a config dict (i.e. from yaml) then the key is 'type'
        # agent_config = agent_config.copy()
        associated_type = config.pop("type", None)
        if associated_type is None:
            raise KeyError(_ERRMSG_MISSING_ASSOCIATED_TYPE)
    else:  # if it's a config object (i.e. from dataclasses) then the key is the associated_type field
        associated_type = config.associated_type

    # get the agent class and config class
    if associated_type not in store.agent_types:
        msg = f"Unknown agent type: {associated_type}"
        raise AttributeError(msg)
    type_entry = store.agent_types[associated_type]
    if not (isinstance(type_entry, (list, tuple)) and len(type_entry) == 2):
        msg = f"Registered agent type {associated_type} should be tuple: (AgentClass, AgentConfigClass)"
        raise TypeError(msg)
    agent_class, agent_config_class = type_entry

    # if it's a config dict (i.e. from yaml) then convert it to a config object
    if isinstance(config, dict):
        config = agent_config_class.from_dict(config)

    return agent_class, config


def get_class_from_dict(key: str, config: dict, copy=True, raise_errors=True) -> tuple[object, dict] | None:
    """Retrieve a class from the registry for a given config ``dict``

    Parameters
    ----------
    key : str
        The namespace to look up, i.e. 'controller', 'spawner', etc.
    config : dict
        The config dict containing a ``type`` key
    copy : bool, optional
        Will copy the config dict before returning it, by default True
    raise_errors : bool, optional
        If True, raise errors, by default True

    Returns
    -------
    tuple[object, dict] | None
        returns (type, modified_config_dict)
        * type: The class
        * modified_config_dict: The config dict with the ``type`` key removed

    Raises
    ------
    AttributeError
        The config dict is missing the ``type`` key
    KeyError
        The namespace or ``type`` was not found in the registry.
    TypeError
        Received a config that is not of ``type(dict)``

    Examples
    --------

    .. code-block:: python

       from swarmsim.config import get_class_from_dict
       from swarmsim.agent.StaticAgent import StaticAgent, StaticAgentConfig

       agent = StaticAgent(StaticAgentConfig(), world=None)

       config = {'type': 'StaticController', 'output': [0.1, 0]}

       cls = get_class_from_dict('controller', config)
       controller = cls(agent, config)

    """
    if key not in store.dictlike_types:
        msg = f"Object namespace is unknown to init system: {key}"
        raise KeyError(msg)
    try:
        if not isinstance(config, dict):
            msg = f"Expected config entry for namespace '{key}' to be a dict, not {type(config).__name__}"
            raise TypeError(msg)
        elif 'type' not in config:
            msg = f"Config dict in namespace '{key}' is missing the 'type' key"
            raise AttributeError(msg)
    except AttributeError:
        if raise_errors:
            raise
    if copy:
        config = config.copy()
    cls_name = config.pop('type')

    # if it's a string, then it's a class name
    if isinstance(cls_name, str):
        if cls_name not in store.dictlike_types[key]:
            msg = f"Class is unknown to init system: {cls_name}"
            raise KeyError(msg)
        return store.dictlike_types[key][cls_name], config

    # otherwise, assume it's a class
    elif isinstance(cls_name, type):
        return cls_name, config


def initialize_natives():
    """Makes the config system aware of all the built-in classes

    Calling this function after the config system has been initialized
    will have no effect.
    """
    store.initialize_natives()
