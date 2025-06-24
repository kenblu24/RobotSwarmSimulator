.. card::  :fas:`earth-americas` The World

   The :py:mod:`~swarmsim.world` contains all the objects that will be simulated.

   .. card::  :fas:`users-viewfinder` Population

      The :py:attr:`~swarmsim.world.World.World.population` is the collection of agents in the world.

      .. card::  :far:`user` Agent

         An :py:mod:`~swarmsim.agent` is an object that exists in the world and has volition.

         .. grid::

            .. grid-item-card::  :far:`compass` Sensor

               An agent can have :py:mod:`~swarmsim.sensors` which distill information from the world
               into observations for the agent to use on each ``step()`` of the simulation.

            .. grid-item-card::  :fas:`gamepad` Controller

                  Each agent has a :py:mod:`~swarmsim.agent.control`\ ler that
                  can control the agent's movement and act based on the sensor information
                  each ``step()`` of the simulation.

      .. card:: :fas:`users` Agent

         A :py:attr:`~swarmsim.world.World.World.population` often has multiple agents, each of
         which can have a different type, controller, or set of sensors.

   .. card::  :fas:`draw-polygon`  World Objects (props)

         :py:mod:`~swarmsim.world.objects` are a special type of agent that are not part of the population.
         It is used to represent objects in the world that are not agents, such as walls, props, and triggers.
         They are stored in the world's :py:attr:`~swarmsim.world.World.World.objects` list.

   .. card::  :fas:`hands-holding-child` Spawners
         
         :py:mod:`~swarmsim.world.spawners` can create new agents and
         add them to the :py:attr:`~swarmsim.world.World.World.population`.

   .. card::  :fas:`ruler-combined` Metrics

         A world can have one or more :py:mod:`~swarmsim.metrics` which reduce the state
         of the world. They can describe the behavior of the agents and are useful for 
         quantifying or training global behaviors.

   .. card::  :fas:`cart-plus` Subscribers

         A world can have :py:mod:`~swarmsim.world.subscribers` which allow user-defined
         hooks to run each ``step()`` of the simulation.