********************
Configuration System
********************

.. toctree::
   :maxdepth: 2
  
   yaml

This article will cover the :py:mod:`~novel_swarms.config` module and explain
why it was created.

.. seealso::

   :doc:`/api/config_store_api`


Why use the config system?
==========================

RobotSwarmSimulator is designed for simulations to be definable with :fab:`python` Python code,
but also for simulation objects to be modified easily in an object-oriented way.

This is why :py:mod:`~novel_swarms.world` and :py:mod:`~novel_swarms.agent` must
be created using dataclasses: it allows us to inherit properties from the base class
in a much more manageable way. This also separates the configuration from the object
itself, so the parameters of ``agent`` instance can be moved around, copied, or
modified without needing to have an actual ``agent`` instance.

``controllers`` and other simulation objects use ``dict`` s for configuration for the same reason.

However, sometimes this can be cumbersome.

Take this example from the :doc:`firstrun` guide:

.. code-block:: python
   :caption: Run a simulation purely using Python

   from novel_swarms.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
   from novel_swarms.world.spawners.AgentSpawner import PointAgentSpawner
   from novel_swarms.agent.MazeAgent import MazeAgent, MazeAgentConfig
   from novel_swarms.agent.control.StaticController import StaticController
   from novel_swarms.world.simulate import main as sim

   world_config = RectangularWorldConfig(size=[10, 10], time_step=1 / 40)
   world = RectangularWorld(world_config)
   controller = StaticController(output=[0.01, 0])
   agent = MazeAgent(MazeAgentConfig(agent_radius=0.1, controller=controller), world)
   spawner = PointAgentSpawner(n=6, facing="away", avoid_overlap=True, agent=agent, oneshot=True)
   world.spawners.append(spawner)

   sim(world)

This allows for you to have a lot of control over the simulated world and how
the agents behave, but look at how messy it is to define the world and agents.

And if you want to run the simulation with a different configuration, you have to
make an entirely new Python file, even if most of the code is the same.
That's not so bad for a simple simulation like this, but when you start building
training programs around these simulations, it can get very messy.

The config system is designed to make this easier by allowing you to
define a lot of the simulation in a single file with easy-to-read YAML.

For example, the above simulation can be defined in a single file like this:

.. code-block:: yaml
   :caption: world.yaml

   type: "RectangularWorld"
   size:  # in meters
     - 10
     - 10
   time_step: !np 1 / 40
   spawners:
     - type: PointAgentSpawner
       oneshot: true
       n: 6
       facing: away
       avoid_overlap: true
       agent:
         type: MazeAgent
         agent_radius: 0.1
         controller:
           type: StaticController
           output: [0.01, 0]

Then, we can run the simulation with:

.. code-block:: python
    :caption: Run a simulation using ``world.yaml``

    from novel_swarms.world.RectangularWorld import RectangularWorldConfig
    from novel_swarms.world.simulate import main as sim
    
    world_config = RectangularWorldConfig.from_yaml(code_dir / 'world.yaml')

    sim(world_config)

Now, if we need to change the simulation, we only need to change the ``world.yaml`` file.
And we can save different ``.yaml`` files for different simulations.

Our :py:mod:`~novel_swarms.yaml` module also provides a custom YAML loader
that defines some nice tags, such as the ``!np`` tag for numpy objects
and the ``!include`` tag for including other YAML files as a mapping.

This allows you to do cool things like:

.. code-block:: yaml
    :caption: world.yaml (example)

    type: "RectangularWorld"
    size: [10, 10]  # yaml flow style
    agents:
      - !include robot1.yaml  # add robot1 agent
      - !include robot2.yaml  # add robot2 agent
      - &anchor003  # save this robot as an anchor
        type: MazeAgent
        name: robot3
        agent_radius: 0.1
        angle: !np radians(90 + 45)  # convert degrees to radians
        poly: !include body_shape.svg  # load an SVG file
        controller:
          type: StaticController
          output: !np [1e-2, pi / 2]  # pi constant from numpy
    spawners:
      - type: ExcelSpawner
        path: !relpath positions.xlsx  # path is relative to cwd or this YAML file
        agent: *anchor003  # use the robot3 agent from above
