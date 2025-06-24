***********
Basic Usage
***********

For your first run after `installing RobotSwarmSimulator <guide/install>`_, let's walk through some simple examples.


Your first simulation
=====================

Let's start with a simple simulation.

We'll use the :py:mod:`~swarmsim.world.RectangularWorld` class to create a world with a single agent.

.. hint::

   Remember to :ref:`activate the virtual environment <activate-venv>` so that you can import ``swarmsim``!

Open a Python shell with ``python``, and make sure you can ``import swarmsim`` with no errors.

.. code-block:: python-console
   :caption: ``python``

   Python 3.11.0 (or newer)
   Type "help", "copyright", "credits" or "license" for more information.
   >>> import swarmsim
   >>>


Creating a :fas:`earth-americas` world
----------------------------------------

First, let's create a world. To do that, we first need to create a
:py:class:`~swarmsim.world.RectangularWorld.RectangularWorldConfig` object.

Then, we can create the world by passing the config to the
:py:class:`~swarmsim.world.RectangularWorld.RectangularWorld` class.

.. code-block:: python

   from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
   world_config = RectangularWorldConfig(size=[10, 10], time_step=1 / 40)
   world = RectangularWorld(world_config)


Creating an :fas:`user` agent
-----------------------------

We now have a world that we can add things to. Let's add an agent to it!

Let's create the :py:class:`~swarmsim.agent.MazeAgent.MazeAgentConfig`
and use it to initialize the :py:class:`~swarmsim.agent.MazeAgent.MazeAgent` class.

.. code-block:: python

   from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
   agent_config = MazeAgentConfig(position=(5, 5), agent_radius=0.1)
   agent = MazeAgent(agent_config, world)

   world.population.append(agent)  # add the agent to the world

Notice how we passed the ``world`` to the agent. This is so that the agent
has a reference to the world, allowing it to access the world's properties.


Starting the :fas:`arrows-spin` simulation
------------------------------------------

Now that we have something to look at, let's start the simulation!

.. code-block:: python

   from swarmsim.world.simulate import main as sim
   sim(world)

You should see a window pop up with a single agent in the center of the world.

.. figure:: /i/rss-hes_just_sitting_there_menacingly.png
   :width: 70 %
   :alt: A simulation with a single agent

   A simulation with a single agent.

But it's not doing anything yet. Let's make it move.
Stop the simulation by sending :kbd:`Ctrl+C` to the terminal.


Adding a :fas:`gamepad` controller
----------------------------------

Let's add a controller to the agent. Controllers make the agent move.
We'll use the :py:class:`~swarmsim.agent.control.StaticController.StaticController` class,
which sends the same movement signals to the agent every step.
:py:class:`~swarmsim.agent.MazeAgent.MazeAgent` takes two movement signals:

1. A forwards speed, in in units per second.
2. A turning speed, in radians per second.

.. code-block:: python

   from swarmsim.agent.control.StaticController import StaticController
   controller = StaticController(output=[0.01, 0.1])  # 10 cm/s forwards, 0.1 rad/s clockwise.
   agent.controller = controller

   sim(world)

Now the agent should move forwards and turn slowly.

.. figure:: /i/rss-you_spin_me_right_round.gif
   :width: 70 %
   :alt: Agent spinning in circle

   Now the agent goes round in circles.


:fas:`hands-holding-child` Spawners
-----------------------------------

But why settle for just one agent? Let's try spawning a bunch of agents.

First, we need to create a :py:class:`~swarmsim.world.spawners.AgentSpawner.PointAgentSpawner`.

.. code-block:: python

   from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
   spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True, agent=agent, mode="oneshot")
   world.spawners.append(spawner)

Now, remove the existing agent from the :py:attr:`~swarmsim.world.World.World.population`
and run the simulation again.

When you run ``sim()``, during the :py:func:`.World.setup`\ , the spawner will create copies of the agent and
controller and add the copies to the world's population. But because of the ``mode="oneshot"`` argument,
the spawner will then delete itself.

The agents will spawn in the same location, but get pushed apart as they spawn.

.. code-block:: python

   del world.population[-1]  # remove the most recently added agent
   sim(world)


Congrats! You've created your first simulation!
To stop the simulation, press :kbd:`Ctrl+C` in the Python shell,
and type ``quit()`` or ``exit()`` to exit Python (or press :kbd:`Ctrl+D` or :kbd:`Ctrl+Z`).


All together now!
-----------------

.. image:: /i/rss-tutorial_circling.png
   :align: right
   :width: 45 %
   :alt: Six agents in a world

Let's recap what we've done so far:

* We created a world with a single agent.
* We added a controller to the agent.
* We spawned a bunch of agents.
* We ran the simulation.


.. _babys_first_simulation:

Here's all of the code in one file:

.. code-block:: python
   :caption: ``my_first_simulation.py``

   from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
   from swarmsim.agent.control.StaticController import StaticController
   from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
   from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
   from swarmsim.world.simulate import main as sim

   world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
   world = RectangularWorld(world_config)
   controller = StaticController(output=[0.01, 0])
   agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1,
                                     controller=controller), world)
   spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True,
                               agent=agent, mode="oneshot")
   world.spawners.append(spawner)

   sim(world)


Simulator Features
==================

Let's have a look at some of the features of the simulator.

First, let's start the simulation again, but in a paused state.

.. code-block:: python

   sim(world, start_paused=True)

Click :fas:`arrow-pointer` on the simulation to focus the window. If you tap :kbd:`L` or the :kbd:`▷` right arrow key,
the simulation will perform a single step :fas:`forward-step`.

You can :fas:`pause` **pause** or :fas:`play` **unpause** the simulation by pressing :kbd:`Space`.

:kbd:`⇧LShift` and :kbd:`⇧RShift` will **slow down** or :fas:`gauge-high` **speed up** the simulation.
The speed multiplier is shown in the top left corner of the window. Values beginning with a ``/`` slash
are divisors, i.e. ``/2`` half or ``/4`` quarter speed. The number of :far:`clock` elapsed time steps is also shown.

The number in between the timesteps and multiplier is the :fas:`stopwatch` step rate and :fas:`film` framerate, respectively, in frames per second.

You can also see the :fas:`ruler-combined` world coordinates under your cursor displayed in this area.

Clicking and dragging the :fas:`computer-mouse` :kbd:`MMB` inside the simulation window will
allow you to :far:`hand` **pan** the simulation, and :fas:`computer-mouse` scrolling up or down will
:fas:`magnifying-glass-plus` **zoom** in or :fas:`magnifying-glass-minus` **zoom** out.

You can reset the viewport and :fas:`magnifying-glass` zoom level with the :kbd:`Num0` Numpad 0 key if you get lost :fas:`house`\ .



.. figure:: /i/rss-panzoom.gif
   :width: 70 %
   :alt: Panning inside the simulation window

   You can pan the simulation with the middle mouse button, and zoom with the scroll wheel.


.. image:: /i/rss-agent_selected.png
   :width: 70 %
   :alt: Agent selected
   :align: left

Clicking on an agent will **select** it. This will show some information about the agent on the right side of the window.
You can **de-select** by clicking on the blank background.

The time-related functions are handled by the :py:func:`.simulate.main` function, while panning, zooming, and other
event-handling is done inside the :py:class:`~.World.World` class.


Sensors & Controllers
=====================

Earlier, we saw how to add a static controller to an agent.
Static controllers, as you saw, cause the agent to move with a constant
speed and turn at a constant rate. But "Agent" implies that they can make
decisions and control their actions in response to changes in their environment.

So let's add sensors to our agents, and connect those sensors
to the agents' controllers.

For this example, we'll use a binary field of view (FOV) sensor.
This sensor will detect the presence of other agents in its field of view,
a triangular region of space that projects from the agent's front. (Actually, it's
a sector of a circle, but we'll gloss over that).

Assuming you're starting from the :ref:`first example code <babys_first_simulation>`\ ,
let's add a sensor to your existing agent like this:

.. code-block:: python

    from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor

    sensor = BinaryFOVSensor(agent, theta=0.45, distance=2,)
    agent.sensors.append(sensor)

The ``theta`` parameter is **half** the angle of the FOV in radians, and the
``distance`` parameter is the detection range. As with controllers, you should also
pass a back-reference to the agent as the first argument.

.. hint::

   If you downloaded the :ref:`my_first_simulation.py <babys_first_simulation>` file,
   you can either open a new Python REPL with ``python`` and paste the code, or run
   the file with the ``-i`` option: ``python -i my_first_simulation.py``. The ``-i``
   stants for "interactive" and will return control to you after running the file.

   The ``sim()`` function starts the sim, so don't forget to stop the simulation with :kbd:`Ctrl+C`\ !


Now let's create a controller that will read the sensor data and change how the robot moves:

.. code-block:: python

   from swarmsim.agent.control.BinaryController import BinaryController

   controller = BinaryController((0.02, -0.5), (0.02, 0.5), agent)
   agent.controller = controller

Now, if you run ``sim(world)``\ , you should see some agents that turn left if one sees something and right if one doesn't!

If not, try re-adding the spawner to the world's ``spawners`` list:

.. code-block:: python

   del world.population[:]  # Delete all agents
   spawner.mark_for_deletion = False  # Re-enable the spawner
   world.spawners.append(spawner)

.. dropdown:: Why did that work?
   :color: secondary
   :icon: light-bulb

   Depending on exactly how you set things up before this section, there's a chance nothing happened.
   Or, you might be wondering why you didn't need to re-define a new ``Spawner()`` instance to
   get the new agent.

   There's a couple things going on here.

   1. The ``Spawner()`` has the ``mode='oneshot'`` argument, which will set its ``spawner.mark_for_deletion``
   flag to ``True`` after the first simulation step, otherwise it would create new agents
   on every ``step()`` (bad). This doesn't mean the spawner deletes itself,
   but the world will simply remove it from its :py:attr:`~swarmsim.world.World.spawners` list.
   So, you don't need to re-define the spawner, you already created it before and can just
   *un-mark it for deletion* and add it back to the ``spawners`` list.

   2. Our :py:mod:`~swarmsim.spawners.AgentSpawner` stores either a config
   for the agent parameters, or in this example, a **reference** to the actual agent itself.
   In the case of the latter, the spawner will attempt to make a :py:func:`~copy.deepcopy`
   of the ``agent`` we gave it earlier. Because ``agent`` is a reference to the agent
   we created earlier, and because we modified the same reference to ``agent`` by setting
   ``agent.controller = controller``, you're modifying *the same* ``agent`` object that
   the spawner has. If you create a new ``agent`` instance and assign it with ``agent = Agent(...)``,
   the spawner will not have access to it automatically.

   .. rubric:: Exercise

   One way to understand this would be to try adding your ``agent`` to the ``world.population``
   multiple times.

   .. code-block:: python

      world.population.append(agent)
      world.population.append(agent)
      world.population.append(agent)

   You won't see three new agents, **just one**. This is because *you didn't create copies* of ``agent``,
   the world just has three extra **references** to the same ``agent`` in the population list.
   This means that we're calling ``agent.step()`` three times as often now, but it's still only
   the same actual agent.


.. hint::

   If the above still didn't work, dry ``exit()``\ -ing your Python shell and starting
   from scratch:

   .. toggle::

      .. code-block:: python
         :caption: ``milling.py``
         :class: dropdown

         from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
         from swarmsim.agent.control.BinaryController import BinaryController
         from swarmsim.world.spawners.AgentSpawner import PointAgentSpawner
         from swarmsim.agent.MazeAgent import MazeAgent, MazeAgentConfig
         from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
         from swarmsim.world.simulate import main as sim

         world_config = RectangularWorldConfig(size=(10, 10), time_step=1 / 40)
         world = RectangularWorld(world_config)
         agent = MazeAgent(MazeAgentConfig(position=(5, 5), agent_radius=0.1), world)
         sensor = BinaryFOVSensor(agent, theta=0.45, distance=2,)
         agent.sensors.append(sensor)
         controller = BinaryController(agent, (0.02, -0.5), (0.02, 0.5))
         agent.controller = controller
         spawner = PointAgentSpawner(world, n=6, facing="away", avoid_overlap=True,
                                     agent=agent, mode="oneshot")
         world.spawners.append(spawner)

         sim(world)

History
=======

.. card::
   :img-top: /i/rss-first_milling.png

   This circular formation is an example of milling!

   In 2014, a group of researchers discovered that a simple rule could be used to
   create this milling formation [#gauci_evolving]_.

   You can even mill with a group of humans! The rule is simple:

      If you see someone, turn left.

      If you don't see anyone, turn right.

   However, the speed is important to get right. In fact, if you adjust the :fas:`gauge-high` speed and how
   quickly you :fas:`arrows-turn-to-dots` turn, you can create a variety of different behaviors, not just milling.

.. sidebar::

   Ants can also mill! `Ant mills <https://en.wikipedia.org/wiki/Ant_mill>`_
   are an example of emergent behaviors.

   .. image:: /i/ant_mill.gif
      :alt: Ants milling
      :loading: lazy

   img: Clemzouzou69, `CC BY-SA 4.0 <https://creativecommons.org/licenses/by-sa/4.0>`_, via `Wikimedia Commons <https://commons.wikimedia.org/wiki/File:Ant_mill.gif>`_

This is actually why RobotSwarmSimulator was created. We needed a way to test what
swarm behaviors result from different :fas:`gauge-high` speeds and :fas:`rotate-right` turning rates.

We've used it to automatically discover interesting behaviors [#novel_discovery]_ [#novel_human]_,
train Spiking Neural Networks [#snnicons]_, and even train real robots [#snnnice]_!

.. note::

   That's also why the package is called :py:mod:`swarmsim`\ .


YAML Configuration
==================

So far, we've only been configuring our world and agents using Python code.

This has benefits, but it's not the only way to configure RobotSwarmSimulator.
You can also use a YAML file to configure your world and agents.

Let's start by replicating the previous example, but this time we'll use a YAML file.

First, let's create a new file called ``world.yaml`` and add the following:

.. code-block:: yaml
   :caption: ``world.yaml``

    type: "RectangularWorld"
    size: [10, 10]
    time_step: !np 1 / 40
    spawners:
      - type: "PointAgentSpawner"
        n: 6
        facing: "away"
        avoid_overlap: true
        mode: oneshot
        agent:
          type: "MazeAgent"
          position: [5, 5]
          agent_radius: 0.1
          sensors:
            - type: "BinaryFOVSensor"
              theta: 0.45
              distance: 2
          controller:
            type: "BinaryController"
            a: [0.02, -0.5]
            b: [0.02, 0.5]

Then, let's create a python file or open a new Python shell and run the following:

.. code-block:: python
   :caption: ``run.py``

   from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
   from swarmsim.world.simulate import main as sim

   world_config = RectangularWorldConfig.from_yaml('world.yaml')

   sim(world_config)

.. hint::

   You can run the file with ``python run.py`` or ``python -i run.py``\ .
   Make sure your ``world.yaml`` file is in the same directory as ``run.py``\ .

You should see the same milling formation as before.

What just happened?
-------------------

:py:mod:`~swarmsim.world`\ s and :py:mod:`~swarmsim.agent`\ s use Config classes,
but to see configuration options for sensors, controllers, and spawners, the arguments are simply
passed as a ``dict`` to the constructors.

The ``world.yaml`` file is a YAML file that describes the world, and :py:meth:`.RectangularWorldConfig.from_yaml`
loads it as a ``dict`` and turns it into a :py:class:`~swarmsim.world.RectangularWorld.RectangularWorldConfig`\ .
Just as ``dict``\ s can contain nested ``dict``\ s, Configs can contain other configs, so the ``spawners:`` sequence
becomes a list of dictionaries, which are then turned into :py:class:`~swarmsim.world.spawners.AgentSpawner.AgentSpawner`\ s.

We cover the order that things are initialized in :ref:`initialization_order`\ .

.. card::

   **Exercise**
   ^^^

   Try changing the parameters in the ``world.yaml`` file and see what happens.

   You can also try adding a single agent to the ``world.population`` list
   adding the ``agents: `` sequence to the ``world.yaml`` file.

   If you've never used YAML before, check out `Learn YAML in Y minutes <https://learnxinyminutes.com/docs/yaml/>`_


What can I change?
==================

If you tried the exercise above, you might be wondering what the parameters are
called and what they do. This information can be gleaned from the :doc:`/api/index`\ .

For example, the options for configuring ``RectangularWorld`` are the parameters
for the :py:class:`~swarmsim.world.RectangularWorld.RectangularWorldConfig` class, which
also inherits options and defaults from the :py:class:`~swarmsim.world.World.AbstractWorldConfig` class.

Similarly, the options for configuring ``MazeAgent`` are the parameters
for the :py:class:`~swarmsim.agent.MazeAgent.MazeAgentConfig` class, and so on.

For objects that don't use Config classes, such as sensors, controllers, and spawners,
remember that the arguments are simply passed as a ``dict`` to the constructors. So the
options are the parameters for the constructor. This is how you might set the controller
of an agent to a :py:mod:`~swarmsim.agent.control.BinaryController`\ :

.. grid:: 2
   :gutter: 3

   .. grid-item::

      .. code-block:: python
         :caption: Python

         agent.controller = BinaryController(
             a=(0.02, -0.5),
             b=(0.02, 0.5),
             agent=agent,
         )

   .. grid-item::

      .. code-block:: yaml
         :caption: YAML

         controller:
           type: "BinaryController"
           a: [0.02, -0.5]
           b: [0.02, 0.5]

.. todo::

   * new controller type tutorial
   * new sensor type tutorial
   * metrics tutorial
   * advanced yaml tutorial (np, include)
   * new agent type tutorial
   * world objects tutorial

   * add pictures and animated gifs


.. rubric:: Citations

.. [#gauci_evolving] \M. Gauci, J. Chen, T. J. Dodd, and R. Groß, “Evolving Aggregation Behaviors in Multi-Robot Systems with Binary Sensors,” 2014, doi: `10.1007/978-3-642-55146-8_25 <https://doi.org/10.1007/978-3-642-55146-8_25>`_.

.. [#novel_discovery] \D. S. Brown, R. Turner, O. Hennigh, and S. Loscalzo, “Discovery and Exploration of Novel Swarm Behaviors Given Limited Robot Capabilities,” 2018, doi: `10.1007/978-3-319-73008-0_31 <https://doi.org/10.1007/978-3-319-73008-0_31>`_.

.. [#novel_human] \C. Mattson and D. S. Brown, “Leveraging Human Feedback to Evolve and Discover Novel Emergent Behaviors in Robot Swarms,” Jul. 2023, `doi: 10.1145/3583131.3590443 <https://doi.org/10.1145/3583131.3590443>`_.

.. [#snnicons] \K. Zhu et al., “Spiking Neural Networks as a Controller for Emergent Swarm Agents,” Jul. 2024, doi: `10.1109/ICONS62911.2024.00055 <https://doi.org/10.1109/ICONS62911.2024.00055>`_.

.. [#snnnice] \K. Zhu, S. Snyder, R. Vega, M. Parsa, and C. Nowzari, “A Milling Swarm of Ground Robots using Spiking Neural Networks.” at the 2025 Neuro Inspired Computational Elements (NICE), Mar. 2025.

