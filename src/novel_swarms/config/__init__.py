"""
this module enforces lazy initialization of the native modules
prior to first use. This is to avoid circular imports.
These modules can then be accessed via the store object
for use with initialization from yaml files or config objects.

It also provides the interface for registering external modules.
"""

from dataclasses import fields


def associated_type(type_name: str):
    def decorator(cls):
        original_init = cls.__init__

        def new_init(self, *args, **kwargs):
            self.associated_type = type_name
            original_init(self, *args, **kwargs)

        cls.__init__ = new_init
        return cls

    return decorator


def filter_unexpected_fields(cls):
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
    def __init__(self):
        self._world_types = {}
        self._agent_types = {}
        self._dictlike_types = {}
        self._controllers = {}
        self._behaviors = {}
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
        self.add_native_behaviors()
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

    def add_native_behaviors(self):
        from .. import behavior

        self.add_dictlike_namespace('behaviors')

        native_behaviors = {name: getattr(behavior, name) for name in behavior.__all__}
        self._dictlike_types['behaviors'].update(native_behaviors)

    def add_native_agent_types(self):
        from ..agent.DiffDriveAgent import DifferentialDriveAgent, DiffDriveAgentConfig
        # from ..agent.HumanAgent import HumanDrivenAgent, HumanDrivenAgentConfig
        from ..agent.StaticAgent import StaticAgent, StaticAgentConfig
        from ..agent.MazeAgent import MazeAgent, MazeAgentConfig
        from ..agent.MazeAgentCaspian import MazeAgentCaspian, MazeAgentCaspianConfig
        from ..agent.MillingAgentCaspian import MillingAgentCaspian, MillingAgentCaspianConfig

        self._agent_types['MazeAgent'] = (MazeAgent, MazeAgentConfig)
        self._agent_types['MazeAgentCaspian'] = (MazeAgentCaspian, MazeAgentCaspianConfig)
        self._agent_types['MillingAgentCaspian'] = (MillingAgentCaspian, MillingAgentCaspianConfig)
        self._agent_types['DiffDriveAgent'] = (DifferentialDriveAgent, DiffDriveAgentConfig)
        # self._agent_types['HumanDrivenAgent'] = (HumanDrivenAgent, HumanDrivenAgentConfig)
        self._agent_types['StaticAgent'] = (StaticAgent, StaticAgentConfig)

    def add_native_spawners(self):
        from ..world.spawners.UniformSpawner import UniformAgentSpawner

        self.add_dictlike_namespace('spawners')

        self._dictlike_types['spawners']['UniformAgentSpawner'] = UniformAgentSpawner


store = LazyKnownModules()


def register_world_type(name: str, world_type, world_config=None):
    store.world_types[name] = (world_type, world_config)


def register_agent_type(name: str, agent_type, agent_config=None):
    store.agent_types[name] = (agent_type, agent_config)


def register_dictlike_namespace(key: str):
    store.add_dictlike_namespace(key)


def register_dictlike_type(key: str, name: str, cls):
    store.add_dictlike_namespace(key)
    store.dictlike_types[key][name] = cls


def get_agent_class(config):
    # get the type name
    if isinstance(config, dict):  # if it's a config dict (i.e. from yaml) then the key is 'type'
        # agent_config = agent_config.copy()
        associated_type = config.pop("type", None)
        if associated_type is None:
            raise Exception(_ERRMSG_MISSING_ASSOCIATED_TYPE)
    else:  # if it's a config object (i.e. from dataclasses) then the key is the associated_type field
        associated_type = config.associated_type

    # get the agent class and config class
    if associated_type not in store.agent_types:
        msg = f"Unknown agent type: {associated_type}"
        raise Exception(msg)
    type_entry = store.agent_types[associated_type]
    if not (isinstance(type_entry, (list, tuple)) and len(type_entry) == 2):
        msg = f"Registered agent type {associated_type} should be tuple: (AgentClass, AgentConfigClass)"
        raise TypeError(msg)
    agent_class, agent_config_class = type_entry

    # if it's a config dict (i.e. from yaml) then convert it to a config object
    if isinstance(config, dict):
        config = agent_config_class.from_dict(config)

    return agent_class, config


def get_class_from_dict(key: str, config: dict, copy=True, raise_errors=True) -> tuple[object, dict]:
    if key not in store.dictlike_types:
        msg = f"Object namespace is unknown to init system: {key}"
        raise KeyError(msg)
    if not (isinstance(config, dict) and 'type' in config):
        if raise_errors:
            msg = f"Config dict in namespace '{key}' is missing the 'type'"
            raise KeyError(msg)
        else:
            return
    if copy:
        config = config.copy()
    cls_name = config.pop('type')
    if cls_name not in store.dictlike_types[key]:
        msg = f"Class is unknown to init system: {cls_name}"
        raise KeyError(msg)
    return store.dictlike_types[key][cls_name], config


def initialize_natives():
    store.initialize_natives()
