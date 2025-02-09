***************
Config Registry
***************

The config module stores references to classes and config dataclasses.
It's used internally to know how to create many types of objects in the simulator:

* :py:mod:`novel_swarms.agent` s
   * :py:mod:`novel_swarms.agent.control` lers
* :py:mod:`novel_swarms.world` s
   * :py:mod:`novel_swarms.world.objects`
   * :py:mod:`novel_swarms.world.spawners`
* :py:mod:`novel_swarms.sensors`
* :py:mod:`novel_swarms.metrics`

This allows you to register your own class definitions which will work in ``.yaml`` files.

:py:mod:`~novel_swarms.agent` and :py:mod:`~novel_swarms.world` Configs
=======================================================================
RobotSwarmSimulator uses dataclasses to define the configurations for agents and worlds.
These dataclasses are used to create the corresponding objects in the simulator.

For example, let's create a new agent class called ``MyAgent`` that has a ``MyAgentConfig`` dataclass.

.. code-block:: python
   :caption: MyAgent.py

   from dataclasses import dataclass
   from novel_swarms.config import associated_type, filter_unexpected_fields
   from novel_swarms.agent.BaseAgent import Agent, BaseAgentConfig

   @associated_type("MyAgent")
   @filter_unexpected_fields
   @dataclass
   class MyAgentConfig(BaseAgentConfig):
       my_custom_field: int = 999

   class MyAgent(Agent):
       pass

The :py:func:`~novel_swarms.config.associated_type` decorator associates the ``MyAgentConfig`` dataclass
with the ``MyAgent`` class by adding a ``config.type = 'MyAgent'`` field to the dataclass.

.. code-block:: python
   :caption: test_custom_agent.py

   from novel_swarms.config import register_agent_type
   from MyAgent import MyAgent, MyAgentConfig

   register_agent_type('MyAgent', MyAgent, MyAgentConfig)

Once your agent class is registered with the config system, you can
load a ``.yaml`` file with a ``type: MyAgent`` field, :py:mod:`~novel_swarms.world.RectangularWorld`
will know how to create a ``MyAgentConfig`` from your ``.yaml`` and 
subsequently create an instance of ``MyAgent``.

.. code-block:: yaml
   :caption: world.yaml

   type: "RectangularWorld"
   agents:
     - type: MyAgent  # this becomes MyAgentConfig
       my_custom_field: 100

A similar system is used for :py:mod:`~novel_swarms.world.World` types and their
associated config classes, but there's currently only one world type: :py:mod:`~novel_swarms.world.RectangularWorld`

Note that :py:mod:`~novel_swarms.world.objects` are a special type of :py:mod:`~novel_swarms.agent` ,
so they also use this system.

Everything Else (dict-like config objects)
==========================================

For everything that isn't an :py:mod:`~novel_swarms.agent` or :py:mod:`~novel_swarms.world` ,
the config system doesn't use dataclasses.
Instead, it uses a dictionary-like object that has a ``type`` field.
This includes everything from :py:mod:`~novel_swarms.world.spawners` to :py:mod:`~novel_swarms.metrics` and :py:mod:`~novel_swarms.agent.control` .

For example, the :py:class:`~novel_swarms.agent.control.StaticController` has a ``type`` field
that is used to determine how to create the controller.

.. code-block:: python
   :caption: SpinningController.py

   from novel_swarms.agent.control.Controller import AbstractController

   class SpinningController(AbstractController):
       def __init__(self, parent,
          angular_velocity: float,
       ):
           super().__init__(parent)
           self.angular_velocity = angular_velocity
      
       def get_actions(self, agent):
           return 0, self.angular_velocity

Then, register the controller with the config system:

.. code-block:: python
   :caption: test_custom_controller.py

   from novel_swarms.config import register_dictlike_type
   from SpinningController import SpinningController

   register_dictlike_type('controller', 'SpinningController', SpinningController)

And then you can use it in a ``.yaml`` file:

.. code-block:: yaml
   :caption: world.yaml

   type: "RectangularWorld"
   agents:
     - type: MyAgent
       controller:
         type: SpinningController
         angular_velocity: 0.1  # rad/s

----

Module
======

.. autosummary::
   :toctree: _gen
   
   novel_swarms.config

.. autodata:: novel_swarms.config.store

.. autoclass:: novel_swarms.config.LazyKnownModules


Functions
=========

.. currentmodule:: novel_swarms.config

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
   There is no ``novel_swarms.config.get_world_class`` function,
   world type lookup is handled inside :py:mod:`~novel_swarms.world.World.World_from_config`.

