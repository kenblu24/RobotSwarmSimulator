********************************
Structure of RobotSwarmSimulator
********************************

Let's take a look at the different components of RobotSwarmSimulator.

World Map
=========

.. card::  The World

   The :py:class:`~novel_swarms.world.World` contains all the objects that will be simulated.

   .. card::  Population

      The :py:attr:`~novel_swarms.world.World.population` is the collection of agents in the world.

      .. card::  Agent

         An :py:mod:`~novel_swarms.agent` is an object that exists in the world and has volition.

         .. grid::

            .. grid-item-card::  Sensor

               An agent can have :py:mod:`~novel_swarms.sensors` which distill information from the world
               into observations for the agent to use on each ``step()`` of the simulation.

            .. grid-item-card::  Controller

                  Each agent has a :py:mod:`~novel_swarms.agent.control.Controller` that
                  can control the agent's movement and act based on the sensor information
                  each ``step()`` of the simulation.

      .. card:: Agent

         A :py:attr:`~novel_swarms.world.World.population` often has multiple agents, each of
         which can have a different type, controller, or set of sensors.

   .. card::  World Objects (props)

         An :py:mod:`~novel_swarms.world.object` is a special type of agent that is not part of the population.
         It is used to represent objects in the world that are not agents, such as walls, props, and triggers.
         They are stored in the world's :py:attr:`~novel_swarms.world.World.objects` list.

   .. card::  Spawners
         
         A :py:mod:`~novel_swarms.world.spawners.Spawner` can create new agents and
         add them to the :py:attr:`~novel_swarms.world.World.population`.

   .. card::  Metrics

         A world can have one or more :py:mod:`~novel_swarms.metrics` which reduce the state
         of the world. They can describe the behavior of the agents and are useful for 
         quantifying or training global behaviors.

   .. card::  Subscribers

         A world can have :py:mod:`~novel_swarms.world.subscribers` which allow user-defined
         hooks to run each ``step()`` of the simulation.



Initialization Order
====================

Here is the order in which the initialization system runs:

* :fas:`desktop` :py:mod:`~novel_swarms.world.simulate`

   First, the simulation initializes the world.

   * :fas:`earth-americas` :py:mod:`World.setup`

      The world then runs ``setup()``, which creates the following:

      .. currentmodule:: novel_swarms.agent

      #. :fas:`users-viewfinder` :py:attr:`World.population` :fas:`arrow-left` :far:`user` :py:func:`Agent.__init__`

         The world then creates agents from agent configs in its ``config.agents`` list, and back-references to the ``world``
         are added to these agents. These agents are then appended to the world's ``population`` list.
         
         Upon initialization, each agent also initializes its controller and sensors from its config, 
         and back-references to the agent are passed to them.

         #. :far:`user` :py:attr:`Agent.controller` :fas:`arrow-left` :fas:`gamepad` :py:func:`Controller.__init__`

         #. :fas:`group-arrows-rotate` :py:attr:`Agent.sensors` :fas:`arrow-left` :far:`compass` :py:func:`Sensor.__init__`

      #. :fas:`user-plus` :py:attr:`World.spawners` :fas:`arrow-left` :fas:`hands-holding-child` :py:func:`Spawner.__init__`

         Spawners are created from spawner configs and appended to the world's ``config.spawners`` list.

         .. note::

            Spawners do not become active until the first ``step()`` of the simulation.

      #. :far:`object-group` :py:attr:`World.objects` :fas:`arrow-left` :fas:`draw-polygon` :py:func:`WorldObject.__init__`

         A similar process to the population initialization is carried out for world objects.

      #. :fas:`chart-column` :py:attr:`World.metrics` :fas:`arrow-left` :fas:`ruler-combined` :py:mod:`metrics.__init__`

         Lastly, 







Simulation Loop
===============

The simulator runs on a single thread. Let's take a look at the execution order
inside the simulation loop. On each tick of the simulation, the following happens:

.. card::  :py:mod:`~novel_swarms.world.simulate` :fas:`arrows-spin`

   The :py:func:`novel_swarms.world.simulate.main` function runs the main simulation loop.

   .. card::  :py:mod:`~novel_swarms.world.World.step` :fas:`earth-americas` :fas:`arrows-spin`

      .. card::  :py:mod:`~novel_swarms.world.World.spawners` :fas:`arrows-spin`


   .. card::  :py:mod:`~novel_swarms.world.World.draw` :fas:`earth-americas` :fas:`pen-to-square`





