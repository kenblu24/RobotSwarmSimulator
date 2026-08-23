***************
Config Registry
***************

The config module stores references to classes and config dataclasses.
It's used internally to know how to create many types of objects in the simulator:

* :py:mod:`swarmsim.agent`\ s
   * :py:mod:`swarmsim.agent.control`\ lers
* :py:mod:`swarmsim.world`\ s
   * :py:mod:`swarmsim.world.objects`
   * :py:mod:`swarmsim.world.spawners`
* :py:mod:`swarmsim.sensors`
* :py:mod:`swarmsim.metrics`

This allows you to register your own class definitions which will work in ``.yaml`` files.

.. seealso::

   See the :py:mod:`swarmsim.config` module for the API.
   
   See the :doc:`/guide/yaml` for information on RobotSwarmSimulator's custom ``.yaml`` parser and tags

:py:mod:`~swarmsim.agent` and :py:mod:`~swarmsim.world` Configs
=======================================================================
RobotSwarmSimulator uses dataclasses to define the configurations for agents and worlds.
These dataclasses are used to create the corresponding objects in the simulator.

For example, let's create a new agent class called ``MyAgent`` that has a ``MyAgentConfig`` dataclass.

.. code-block:: python
   :caption: MyAgent.py

   from dataclasses import dataclass
   from swarmsim.config import associated_type, filter_unexpected_fields
   from swarmsim.agent.BaseAgent import Agent, BaseAgentConfig

   @associated_type("MyAgent")
   @filter_unexpected_fields
   @dataclass
   class MyAgentConfig(BaseAgentConfig):
       my_custom_field: int = 999

   class MyAgent(Agent):
       pass

The :py:func:`~swarmsim.config.associated_type` decorator associates the ``MyAgentConfig`` dataclass
with the ``MyAgent`` class by adding a ``config.type = 'MyAgent'`` field to the dataclass.

.. code-block:: python
   :caption: test_custom_agent.py

   from swarmsim.config import register_agent_type
   from MyAgent import MyAgent, MyAgentConfig

   register_agent_type('MyAgent', MyAgent, MyAgentConfig)

Once your agent class is registered with the config system, you can
load a ``.yaml`` file with a ``type: MyAgent`` field, :py:mod:`~swarmsim.world.RectangularWorld`
will know how to create a ``MyAgentConfig`` from your ``.yaml`` and 
subsequently create an instance of ``MyAgent``.

.. code-block:: yaml
   :caption: world.yaml

   type: "RectangularWorld"
   agents:
     - type: MyAgent  # this becomes MyAgentConfig
       my_custom_field: 100

A similar system is used for :py:mod:`~swarmsim.world.World` types and their
associated config classes, but there's currently only one world type: :py:mod:`~swarmsim.world.RectangularWorld`

Note that :py:mod:`~swarmsim.world.objects` are a special type of :py:mod:`~swarmsim.agent` ,
so they also use this system.

Everything Else (dict-like config objects)
==========================================

For everything that isn't an :py:mod:`~swarmsim.agent` or :py:mod:`~swarmsim.world` ,
the config system doesn't use dataclasses.
Instead, it uses a dictionary-like object that has a ``type`` field.
This includes everything from :py:mod:`~swarmsim.world.spawners` to :py:mod:`~swarmsim.metrics` and :py:mod:`~swarmsim.agent.control` .

For example, the :py:class:`~swarmsim.agent.control.StaticController` has a ``type`` field
that is used to determine how to create the controller.

.. code-block:: python
   :caption: SpinningController.py

   from swarmsim.agent.control.Controller import AbstractController

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

   from swarmsim.config import register_dictlike_type
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

