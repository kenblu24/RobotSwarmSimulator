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
        for key, value in unexpected_kwargs.items():
            setattr(self, key, value)
        original_init(self, *args, **cleaned_kwargs)

    cls.__init__ = new_init
    return cls


class LazyKnownModules:
    def __init__(self):
        self._world_types = {}
        self._agent_types = {}
        self._sensor_types = {}
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
    def sensor_types(self):
        self.initialize_natives()
        return self._sensor_types

    @property
    def controllers(self):
        self.initialize_natives()
        return self._controllers

    @property
    def behaviors(self):
        self.initialize_natives()
        return self._behaviors

    def initialize_natives(self):
        if self.initialized_natives:
            return
        self.initialized_natives = True
        self.add_native_world_types()
        self.add_native_agent_types()
        self.add_native_sensors()
        self.add_native_controllers()
        self.add_native_behaviors()

    def add_native_world_types(self):
        from ..world.RectangularWorld import RectangularWorld, RectangularWorldConfig

        self._world_types["RectangularWorld"] = (RectangularWorld, RectangularWorldConfig)

    def add_native_sensors(self):
        from ..sensors.BinaryFOVSensor import BinaryFOVSensor
        from ..sensors.BinaryLOSSensor import BinaryLOSSensor
        from ..sensors.GenomeDependentSensor import GenomeBinarySensor
        from ..sensors.RegionalSensor import RegionalSensor
        from ..sensors.StaticSensor import StaticSensor

        self._sensor_types["BinaryFOVSensor"] = BinaryFOVSensor
        self._sensor_types["BinaryLOSSensor"] = BinaryLOSSensor
        self._sensor_types["GenomeBinarySensor"] = GenomeBinarySensor
        self._sensor_types["RegionalSensor"] = RegionalSensor
        self._sensor_types["StaticSensor"] = StaticSensor

    def add_native_controllers(self):
        from ..agent.control.Controller import Controller
        from ..agent.control.StaticController import StaticController
        from ..agent.control.HomogeneousController import HomogeneousController

        self._controllers["Controller"] = Controller
        self._controllers["StaticController"] = StaticController
        self._controllers["HomogeneousController"] = HomogeneousController

    def add_native_agent_types(self):
        from ..agent.DiffDriveAgent import DifferentialDriveAgent, DiffDriveAgentConfig
        # from ..agent.HumanAgent import HumanDrivenAgent, HumanDrivenAgentConfig
        from ..agent.StaticAgent import StaticAgent, StaticAgentConfig
        from ..agent.MazeAgent import MazeAgent, MazeAgentConfig
        from ..agent.MazeAgentCaspian import MazeAgentCaspian, MazeAgentCaspianConfig
        from ..agent.MillingAgentCaspian import MillingAgentCaspian, MillingAgentCaspianConfig

        self._agent_types["MazeAgent"] = (MazeAgent, MazeAgentConfig)
        self._agent_types["MazeAgentCaspian"] = (MazeAgentCaspian, MazeAgentCaspianConfig)
        self._agent_types["MillingAgentCaspian"] = (MillingAgentCaspian, MillingAgentCaspianConfig)
        self._agent_types["DiffDriveAgent"] = (DifferentialDriveAgent, DiffDriveAgentConfig)
        # self._agent_types["HumanDrivenAgent"] = (HumanDrivenAgent, HumanDrivenAgentConfig)
        self._agent_types["StaticAgent"] = (StaticAgent, StaticAgentConfig)

    def add_native_behaviors(self):
        from .. import behavior

        native_behaviors = {name: getattr(behavior, name) for name in behavior.__all__}
        self._behaviors.update(native_behaviors)


store = LazyKnownModules()


def register_world_type(name: str, world_type, world_config=None):
    store.world_types[name] = (world_type, world_config)


def register_agent_type(name: str, agent_type, agent_config=None):
    store.agent_types[name] = (agent_type, agent_config)


def register_sensor_type(name: str, sensor_type, sensor_config=None):
    store.sensor_types[name] = (sensor_type, sensor_config)


def register_controller_type(name: str, controller_type, controller_config=None):
    store.controllers[name] = controller_type


def register_behavior(name: str, behavior_class):
    store.behaviors[name] = behavior_class


def initialize_natives():
    store.initialize_natives()
