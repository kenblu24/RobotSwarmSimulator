********************
Writing YAML Configs
********************

The :py:mod:`swarmsim.yaml` module provides a custom YAML loader
that defines some convenience tags. This article will explain how
the YAML files are loaded and how to use the custom tags.

.. |_| unicode:: U+0020 .. space
.. |nbsp| unicode:: U+00A0 .. non-breaking space

.. seealso::

   Looking for how to load or dump YAML files with our custom tags?
   See :py:mod:`swarmsim.yaml`

   Or for information on how to use your custom class in a YAML file,
   see :doc:`/guide/config_store_api`.


When we set up a world configuration, ``swarmsim`` lets us either create the
configuration in Python, or define it in a YAML file. Here's a simple example:


.. code-block:: yaml
   :caption: ``world.yaml``

   type: RectangularWorld
   size: [8, 8]
   time_step: !np 1 / 40
   agents:
   - type: MazeAgent
     position: [4, 4]
     controller:
       type: StaticController
       output: [0.1, 0.2]

This YAML file can be loaded and run like this:

.. code-block:: python

   from swarmsim import config_from_yaml, run_sim

   world_config = config_from_yaml('world.yaml')
   run_sim(world_config)

This is equivalent to the following Python code:

.. code-block:: python

   from swarmsim import RectangularWorldConfig, run_sim
   from swarmsim.agent.MazeAgent import MazeAgentConfig
   from swarmsim.agent.control.StaticController import StaticController

   world_config = RectangularWorldConfig(
      size=[8, 8],
      time_step=1 / 40,
      agents=[
         MazeAgentConfig(
            position=[4, 4],
            controller=StaticController(output=[0.1, 0.2]),
         ),
      ],
   )
   run_sim(world_config)

As you can see, the YAML file is much more concise than the equivalent Python code.

The YAML Language
=================

YAML is a human-readable data serialization format.
We use it to write configurations that describe simulations.

It's a superset of JSON, so it's easy to read and write.

We use the `PyYAML library <https://pypi.org/project/PyYAML/>`_ to load and dump YAML files.

.. seealso::

   Here's a nice and quick tutorial for YAML: `Learn YAML in Y minutes <https://learnxinyminutes.com/docs/yaml/>`_
   
   Or have a look at the `YAML specification <https://yaml.org/spec/1.1>`_


YAML Tags
=========

The YAML standard allows for !tags which provide information on how to parse
a YAML entry. These tags usually start with ``!`` and are defined in the
`YAML specification: Tags <https://yaml.org/spec/1.1/#id861700>`_. An
example of standard tags are type specifiers, such as ``!!str`` and ``!!int``.

.. code-block:: yaml

   explicitly-typed-value: !!int 42

RobotSwarmSimulator uses a custom PyYAML loader to allow for some nice features.
The custom YAML tags are defined in the :py:mod:`~swarmsim.yaml` module.

.. _yaml_crazy_tag_example:

Here's an example of a crazy YAML file that uses a bunch of YAML features and our custom tags:

.. include:: crazy_yaml_example.rst

If you're new to YAML or haven't seen the ``&anchor`` and ``*anchor`` syntax,
check out `Learn YAML in Y minutes`_.

To understand what the ``!include``, ``!relpath``, and ``!np`` tags do, read on.

Evaluating Simple Math Expressions with ``!np``
-----------------------------------------------

This tag is used to convert a YAML string, sequence, or mapping to a numpy object.

In this example, the following YAML files are in the same directory:

.. code-block:: yaml
   :caption: foo.yaml

   example: !np complex('2+2j')

See the :py:mod:`~swarmsim.yaml.mathexpr` module for more information on what you can do.

.. _yaml-tags-include:

Creating Hierarchical Configurations with ``!include``
------------------------------------------------------

This tag is used to include another YAML file as a mapping.

For example, see the following YAML files:

.. grid:: 2
   :margin: 0
   :gutter: 3
   :padding: 0 0 0 0

   .. grid-item::

      .. code-block:: yaml
         :caption: bar.yaml

         my_list:
           - 1
           - 2
           - 3
   
   .. grid-item::

      .. code-block:: yaml
         :caption: foo.yaml

         foo: !include bar.yaml

   .. grid-item::
      :columns: 12

Here's what that looks like when loading:

.. code-block:: python
   :caption: Loading ``foo.yaml``

   >>> from swarmsim.yaml import load
   >>> mapping = load('foo.yaml')

   >>> print(mapping)
   {'foo': {'my_list': [1, 2, 3]}}

Here's the single-file equivalent:

.. code-block:: yaml
   :caption: Equivalent Single YAML file

   foo:
     my_list:
        - 1
        - 2
        - 3

The file extension of what you're including affects the behavior of the ``!include`` tag:

* ``.yaml`` files will be loaded using the :py:func:`~swarmsim.yaml.load` function
* ``.json`` files will be loaded using ``json.load``
* All other files are read as text and returned as a string

We have added a special case for using ``!include`` with
`YAML 1.1's Merge Keys <https://yaml.org/type/merge.html>`_ (:literal:`<<: \ `).
You can use ``<<: !include FILENAME.yaml`` to include a YAML file, and the file's
contents will be placed into the same level as the :literal:`<<: \ ` |nbsp| key. This is in contrast to
``key: !include FILENAME.yaml``, which would place the file's contents under ``'key'``.

This is powered by PyYAML's support for the now deprecated Merge Key Language-Independent Type,
so if that feature is removed, this feature will be removed as well.

.. grid:: 2
   :margin: 0
   :gutter: 3
   :padding: 0 0 0 0

   .. grid-item::

      .. code-block:: yaml
         :caption: root.yaml

         hello: world
         <<: !include merged.yaml
   
   .. grid-item::

      .. code-block:: yaml
         :caption: merged.yaml

         my_list:
           - 1
           - 2
           - 3

is equivalent to:

.. code-block:: yaml

   hello: world
   my_list:
     - 1
     - 2
     - 3


.. _yaml-tags-relpath:

Referencing File Paths with ``!relpath``
----------------------------------------

This tag is used to resolve the relative path given, but unlike the
``!include`` tag, it does not load the file, and instead returns
the absolute path as a string.

.. code-block:: yaml
   :caption: /home/user/project/foo.yaml

   path: !relpath bar.yaml

This is equivalent to:

.. code-block:: yaml

   path: /home/user/project/bar.yaml

.. _relpath-resolution:

Path Resolution Order
---------------------

When loading a YAML file, the ``!include`` and ``!relpath`` tags will resolve the path
by testing the following assumptions in order:

.. card::

   #. Path is **not** relative to the current working directory

      (i.e. the path is absolute or relative to the user home directory)
   #. Path is relative to the ``.yaml`` file with the tag
   #. Path is relative to the current working directory
      (where you were when you ran ``python``).

      This is the default behavior for relative paths in Python, but it is the last place we look.

If a file isn't found at any of these locations, an error will be raised. See :py:func:`.include.search_file`.




.. container::

   |nbsp|



Parametric YAML Configs via Jinja Templating
============================================

When working with Python-based configuration files, you can create parametric configurations
by passing a variable in a class or dataclass instantiation:

.. code-block:: python

   def world_configurator(width):
       height = width / 2
       RectangularWorldConfig(size=[width, height])

You can actually do this with YAML too by using Jinja templating! You can create a
YAML file that acts like a function, taking in variables and returning a configuration:

.. code-block:: yaml
   :caption: ``world.yaml``

   #%> set height = width / 2
   type: RectangularWorld
   size:
   - <{ width }> 
   - <{ height }>

Then, you can pass in variables like this:

.. code-block:: python

   from swarmsim import config_from_yaml
   world_config = config_from_yaml('world.yaml', width=6)

The ``<{ width }>`` and ``<{ height }>`` are Jinja blocks which are converted to
text before the YAML file is loaded. The ``#%>`` is a line statement prefix, which
allows you to set Jinja variables which can be referenced in the blocks.
You can also write statements like ``<% set height = width / 2 %>`` in the YAML file.

.. note::

   The ``<% statement %>``, ``<{ expression }>``, and ``#%> line_statement`` delimiters are
   slightly different from the usual Jinja syntax. This is because YAML files may also contain
   specifications for modules that use Jinja templating inside them, such as the
   :py:mod:`~swarmsim.metrics.JinjaMetric`.
   The different delimiters allow us to distinguish between Jinja at the YAML level and Jinja
   templates and expressions that are used to specify other behavior.


.. seealso::

   For information on our custom additions to the Jinja2 API, see :py:mod:`swarmsim.util.jinja`.


Jinja templating is a powerful tool for creating parametric configurations. Here's some examples
of what you can do:

**Dynamic Includes**

Since Jinja templates are evaluated down to pure YAML, you can dynamically import other YAML files:

.. code-block:: yaml
   :caption: ``world.yaml``

   type: RectangularWorld
   agents:
   - !include <{ agent_file }>.yaml

.. code-block:: python

   from swarmsim import config_from_yaml
   world_config = config_from_yaml('world.yaml', agent_file='drone')

**Default Values**

You can set default values for variables in the YAML file with the :py:func:`~jinja-filters.default` filter:

.. code-block:: templateyaml
   :caption: ``world.yaml``

   #%> set width = width|default(10)
   type: RectangularWorld
   size: [ <{ width }>, <{ width }> ]
   time_step: <{ time_step|default('!np 1 / 40') }>

This will set the width to 10 if you don't pass in a value for ``width``.
You can also use the filter inside blocks. Note that we force it to become a string
so that YAML can parse the ``!np`` tag.

There are a lot more operators like this; see the full list of
:ref:`Jinja filters <jinja2:filters>` and :ref:`Jinja tests <jinja2:tests>`\ .

**Swappable Sections**

You can use Jinja templating to swap out sections of a YAML file.
Here's an example where we can swap between a binary and a human controller:

.. code-block:: yaml
   :caption: ``world.yaml``

   type: RectangularWorld
   agents:
   - type: MazeAgent
     #%> set controller_type = controller_type|default('binary')
     <% if evader == 'binary' %>
     sensors:
     - type: BinaryFOVSensor
     controller:
       type: BinaryController
       a: [0.2, 0.2]
       b: [0.2, 0]
       sensor_id: 0
     <% elif evader == 'human' %>
     controller:
       type: HumanController
       joystick: null
       keys: arrowkeys
     <% endif %>

Jinja2 offers a full set of control structures that haven't been covered here.
See the :ref:`jinja2:list-of-control-structures` for more information.

.. seealso::

   Jinja2 is a very powerful and widely used templating engine, and we've only
   covered a small subset of its features here. For more information, see the
   official :external+jinja2:doc:`templates` guide.
