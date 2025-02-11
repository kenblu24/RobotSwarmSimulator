***********
Basic Usage
***********

For your first run after `installing RobotSwarmSimulator <guide/install>`_, let's walk through some simple examples.


Your first simulation
=====================

Let's start with a simple simulation.

We'll use the :py:mod:`~novel_swarms.world.RectangularWorld` class to create a world with a single agent.

.. hint::

   Remember to :ref:`activate the virtual environment <activate-venv>` so that you can import ``novel_swarms``!

Open a Python shell with ``python``, and make sure you can ``import novel_swarms`` with no errors.

.. code-block:: python-console
   :caption: ``python``

   Python 3.11.0 (or newer)
   Type "help", "copyright", "credits" or "license" for more information.
   >>> import novel_swarms
   >>> 


Creating a :fas:`earth-americas` world
----------------------------------------

First, let's create a world. To do that, we first need to create a
:py:class:`~novel_swarms.world.RectangularWorld.RectangularWorldConfig` object.

Then, we can create the world by passing the config to the
:py:class:`~novel_swarms.world.RectangularWorld.RectangularWorld` class.

.. code-block:: python

   from novel_swarms.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
   world_config = RectangularWorldConfig(size=[10, 10], time_step=1 / 40)
   world = RectangularWorld(world_config)


Creating an :fas:`user` agent
-----------------------------

We now have a world that we can add things to. Let's add an agent to it!

Let's create the :py:class:`~novel_swarms.agent.MazeAgent.MazeAgentConfig`
and use it to initialize the :py:class:`~novel_swarms.agent.MazeAgent.MazeAgent` class.

.. code-block:: python

   agent_config = MazeAgentConfig(pos=(5, 5), agent_radius=0.1)
   agent = MazeAgent(agent_config, world)

   world.population.append(agent)  # add the agent to the world

Notice how we passed the ``world`` to the agent. This is so that the agent
has a reference to the world, allowing it to access the world's properties.


Starting the :fas:`arrows-spin` simulation
------------------------------------------

Now that we have something to look at, let's start the simulation!

.. code-block:: python

   from novel_swarms.world.simulate import main as sim
   sim(world)

You should see a window pop up with a single agent in the center of the world.

But it's not doing anything yet. Let's make it move.
Stop the simulation by sending :kbd:`Ctrl+C` to the terminal.


Adding a :fas:`gamepad` controller
----------------------------------

Let's add a controller to the agent. Controllers make the agent move.
We'll use the :py:class:`~novel_swarms.agent.control.StaticController.StaticController` class,
which sends the same movement signals to the agent every step.
:py:class:`~novel_swarms.agent.MazeAgent.MazeAgent` takes two movement signals:

1. A forwards speed, in in units per second.
2. A turning speed, in radians per second.

.. code-block:: python

   from novel_swarms.agent.control.StaticController import StaticController
   controller = StaticController(output=[0.01, 0.1])  # 10 cm/s forwards, 0.1 rad/s clockwise.
   agent.controller = controller

   sim(world)

Now the agent should move forwards and turn slowly.


:fas:`hands-holding-child` Spawners
-----------------------------------

But why settle for just one agent? Let's try spawning a bunch of agents.

First, we need to create a :py:class:`~novel_swarms.world.spawners.AgentSpawner.PointAgentSpawner`.

.. code-block:: python

   from novel_swarms.world.spawners.AgentSpawner import PointAgentSpawner
   spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True, agent=agent, oneshot=True)
   world.spawners.append(spawner)

Now, remove the existing agent from the :py:attr:`~novel_swarms.world.World.World.population` and run the simulation again.

The spawner will create copies of the agent and controller and add the copies to the world's population.

The agents will spawn in the same location, but get pushed apart as they spawn.

.. code-block:: python

   del world.population[-1]  # remove the most recently added agent
   sim(world)

Because of the ``oneshot=True`` argument, the spawner will spawn all its agents once,
and then delete itself.


Congrats! You've created your first simulation!
To stop the simulation, press :kbd:`Ctrl+C` in the Python shell,
and type ``exit()`` to exit Python (or press :kbd:`Ctrl+D` or :kbd:`Ctrl+Z`).


All together now!
-----------------

Let's recap what we've done so far:

* We created a world with a single agent.
* We added a controller to the agent.
* We spawned a bunch of agents.
* We ran the simulation.

.. code-block:: python
   :caption: ``my_first_simulation.py``

   from novel_swarms.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
   from novel_swarms.agent.control.StaticController import StaticController
   from novel_swarms.world.spawners.AgentSpawner import PointAgentSpawner
   from novel_swarms.agent.MazeAgent import MazeAgent, MazeAgentConfig
   from novel_swarms.world.simulate import main as sim

   world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
   world = RectangularWorld(world_config)
   controller = StaticController(output=[0.01, 0])
   agent = MazeAgent(MazeAgentConfig(pos=(5, 5), agent_radius=0.1,
                                     controller=controller), world)
   spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True,
                               agent=agent, oneshot=True)
   world.spawners.append(spawner)

   sim(world)

