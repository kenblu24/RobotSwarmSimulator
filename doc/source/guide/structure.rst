********************************
Structure of RobotSwarmSimulator
********************************

Let's take a look at the different components of RobotSwarmSimulator.

World Map
=========

.. include:: world_map.rst


.. _initialization_order:

Initialization Order
====================

The initialization system machinery typically starts with the :py:func:`~swarmsim.world.World.World_from_config` function.
Then, the world object runs its ``__init__()`` method.

If using the :py:mod:`~swarmsim.world.simulate` module, the simulation initializes the world by calling ``world.setup()``.

Here is the order in which the initialization system runs:

* :fas:`desktop` :py:mod:`~swarmsim.world.simulate`

   First, the simulation initializes the world.

   * :fas:`earth-americas` :py:mod:`.World.setup`

      The world then runs ``setup()``, which creates the following:

      .. currentmodule:: swarmsim.agent

      #. :fas:`users-viewfinder` :py:attr:`.World.population` :fas:`arrow-left` :far:`user` :py:func:`Agent.__init__`

         The world then creates agents from agent configs in its ``config.agents`` list, and back-references to the ``world``
         are added to these agents. These agents are then appended to the world's ``population`` list.
         
         Upon initialization, each agent also initializes its controller and sensors from its config, 
         and back-references to the agent are passed to them.

         #. :far:`user` :fas:`brain` :py:attr:`Agent.controller` :fas:`arrow-left` :fas:`gamepad` :py:func:`Controller.__init__`

         #. :far:`user` :fas:`group-arrows-rotate` :py:attr:`Agent.sensors` :fas:`arrow-left` :far:`compass` :py:func:`Sensor.__init__`

      #. :fas:`user-plus` :py:attr:`World.spawners` :fas:`arrow-left` :fas:`hands-holding-child` :py:func:`Spawner.__init__`

         Spawners are created from spawner configs and appended to the world's ``config.spawners`` list.

         .. note::

            Spawners are ``step()``\ ed once during the world's :py:meth:`World.setup` method.

      #. :far:`object-group` :py:attr:`World.objects` :fas:`arrow-left` :fas:`draw-polygon` :py:func:`WorldObject.__init__`

         A similar process to the population initialization is carried out for world objects.

      #. :fas:`chart-column` :py:attr:`World.metrics` :fas:`arrow-left` :fas:`ruler-combined` :py:mod:`metrics.__init__`

         Lastly, any metrics in the world's ``config.metrics`` list are created and appended to the world's ``metrics`` list.

      #. :fas:`user-plus` :py:attr:`World.spawners` :fas:`arrows-spin` :fas:`hands-holding-child` :py:func:`Spawner.step`
      
         By default, ``step()`` is called on all spawners in the world's ``spawners`` list.
         This is done so that spawners have a chance to run before the first ``step()`` of the simulation, but this can
         be disabled with the ``step_spawners=False`` argument to :py:func:`.World.setup`.


.. hint::

   Keep in mind that the initialization system operates on the configs, but you can
   side-step it entirely by creating the objects in code yourself!


.. _step_execution_order:

Simulation Loop
===============

The simulator runs on a single thread. Let's take a look at the execution order
inside the simulation loop. On each tick of the simulation, the following happens:

.. card::  :py:mod:`~swarmsim.world.simulate` :fas:`arrows-spin`

   The :py:func:`swarmsim.world.simulate.main` function runs the main simulation loop.

   .. card::  :far:`keyboard` :fas:`arrow-pointer` Event Handling

      If the simulation is **not** in headless mode, input events are handled first.


   The simulator then asks the world object to ``step()`` itself.

   Outputting to the screen can be slow, so when the user requests the simulation to speedup,
   ``step()`` will be called multiple times, skipping event handling and ``draw()`` calls.

   .. card::  :fas:`earth-americas` :py:meth:`.RectangularWorld.step` :fas:`arrows-spin`

      .. card::  :fas:`hands-holding-child` ``spawner.step()`` :fas:`arrows-spin`
         
         For each spawner in :fas:`user-plus` :py:attr:`.World.spawners`
         ^^^^

         Each :fas:`hands-holding-child` spawner performs its ``step()`` method at this point.

         Spawners have a ``mark_for_deletion`` flag to remove them from the list of :fas:`user-plus` :py:attr:`.World.spawners`.
         If a spawner has ``oneshot=True``, then it will be marked for deletion after its first ``step()``.
         
      .. card::  :far:`user` :py:meth:`Agent.step` :fas:`arrows-spin`
         
         For each agent in :fas:`users-viewfinder` :py:attr:`.World.population`\ :
         ^^^^

         Each :far:`user` :py:mod:`~swarmsim.agent` performs its ``step()`` method at this point.

         Its ``step()`` method may call ``step()`` on its :far:`user` :fas:`brain` :py:attr:`~swarmsim.agent.Agent.Agent.controller` 
         or :far:`user` :fas:`group-arrows-rotate` :py:attr:`~swarmsim.agent.Agent.Agent.sensors`.

         Here's the execution order for :py:meth:`.MazeAgent.step`\ :

         .. card::  :fas:`gamepad` :py:meth:`.Controller.get_actions()`

            The controller returns actions to take based on the :far:`user` :fas:`group-arrows-rotate` :py:attr:`Agent.sensors`\ .

         .. card::  :far:`user` :fas:`up-down-left-right` agent changes its state

            The :far:`user` agent carries out the actions from the :far:`user` :fas:`brain` :py:attr:`~swarmsim.agent.Agent.Agent.controller`\ ,
            moving according to its agent-specific dynamics.

         .. card::  :far:`compass` :py:func:`Sensor.step()` :fas:`arrows-spin`
            
            For each sensor in :far:`user` :fas:`group-arrows-rotate` :py:attr:`Agent.sensors`\ :
            ^^^^

            The :far:`user` agent updates its sensors so that the new :far:`compass` :py:attr:`~swarmsim.sensors.AbstractSensor.AbstractSensor.current_state`
            will be available on the next :far:`user` ``step()``.

      .. card:: :fas:`draw-polygon` :py:meth:`WorldObject.step` :fas:`arrows-spin`
         
         For each object in :far:`object-group` :py:attr:`World.objects`\ :
         ^^^^

         A world object is based on the :py:class:`~swarmsim.agent.Agent.Agent` class, so its ``step()``
         method may also call ``step()`` on its :far:`user` :fas:`brain` :py:attr:`~swarmsim.world.objects.StaticObject.StaticObject.controller`
         or :far:`user` :fas:`group-arrows-rotate` :py:attr:`~swarmsim.world.objects.StaticObject.StaticObject.sensors`\ , if it has any.

      .. card::  :fas:`ruler-combined` :py:func:`.AbstractMetric.calculate()` :fas:`arrows-spin`
         
         For each metric in :fas:`earth-americas` :fas:`chart-column` :py:attr:`World.metrics`\ :
         ^^^^

         Following the :fas:`earth-americas` ``step()`` :fas:`arrows-spin`\ , each metric takes an
         observation of the world's state and performs its calculations, storing the results in its
         :py:attr:`~swarmsim.metrics.AbstractMetric.AbstractMetric.value_history` attribute.


   If the simulation is not operating headlessly, then the simulator calls ``draw()`` on the world object.

   .. admonition:: Step rate is not tied to FPS!

      ``draw()`` is not guaranteed to be called once per ``step()``! It is called
      even when the simulation is :fas:`pause` paused.

      When the simulation is sped up, some ``draw()`` calls will be skipped.

   .. card::  :fas:`earth-americas` :py:mod:`~swarmsim.world.World.RectangularWorld.draw` :fas:`pen-to-square`

      There is currently no layering system, so the first things that are drawn are the bottom-most 'layer'
      and succeeding ``draw()`` calls draw on top.

      First, the :fas:`window-maximize` screen is cleared. Then, ``draw()`` is called:

      .. card:: :fas:`draw-polygon` :py:meth:`.Object.draw` :fas:`pen-to-square`
         
         For each object in :far:`object-group` :py:attr:`World.objects`\ :
         ^^^^

      .. card:: :far:`user` :py:meth:`Agent.draw` :fas:`pen-to-square`
         
         For each agent in :fas:`users-viewfinder` :py:attr:`.World.population`\ :
         ^^^^

         As an example, see :py:meth:`.StaticAgent.draw`\ .









