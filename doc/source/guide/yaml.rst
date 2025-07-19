***************
Our YAML module
***************

The :py:mod:`swarmsim.yaml` module provides a custom YAML loader
that defines some convenience tags. This article will explain how
the YAML files are loaded and how to use the custom tags.


.. seealso::

   Looking for how to load or dump YAML files with our custom tags?
   See :py:mod:`swarmsim.yaml`

   Or for information on how to use your custom class in a YAML file,
   see :doc:`/guide/config_store_api`.


YAML
====

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

The ``!np`` tag
---------------

This tag is used to convert a YAML string, sequence, or mapping to a numpy object.

In this example, the following YAML files are in the same directory:

.. code-block:: yaml
   :caption: foo.yaml

   example: !np complex('2+2j')

See the :py:mod:`~swarmsim.yaml.mathexpr` module for more information on what you can do.

.. _yaml-tags-include:

The ``!include`` tag
--------------------

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

      .. code-block:: python
         :caption: Result

         >>> from swarmsim.yaml import load
         >>> mapping = load('foo.yaml')

         >>> print(mapping)
         {'foo': {'my_list': [1, 2, 3]}}

The file extension of what you're including affects the behavior of the ``!include`` tag:

* ``.yaml`` files will be loaded using the :py:func:`~swarmsim.yaml.load` function
* ``.json`` files will be loaded using ``json.load``
* All other files are read as text and returned as a string

.. _yaml-tags-relpath:

The ``!relpath`` tag
--------------------

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
=====================

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
