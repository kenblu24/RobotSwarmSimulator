.. RobotSwarmSimulator documentation master file, created by
   sphinx-quickstart on Thu Feb  6 20:58:59 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. title:: RobotSwarmSimulator documentation

RobotSwarmSimulator |release| documentation
===========================================

RobotSwarmSimulator is a Python package for simulating 2D robot swarms.

This documentation covers the https://github.com/kenblu24/RobotSwarmSimulator fork.

.. warning::
   Both the documentation and the simulator software are under heavy development.
   Expect holes in documentation and bugs aplenty.

Get Started
===========

.. tab-set::
   :class: sd-width-content-min

   .. tab-item:: pip

      .. code-block:: bash

         pip install swarmsim

   .. tab-item:: uv

      We recommend using `uv`_ to install the package. If you don't have ``uv``, check out the more detailed :doc:`installation guide <guide/install.html>`.

      .. code-block:: bash

         uv pip install swarmsim

   .. tab-item:: other

      .. rst-class:: section-toc
      .. toctree::
         :maxdepth: 2

         guide/install

For more detailed instructions, see the
:doc:`installation guide <guide/install>`
, which covers virtual environments, faster installation with uv, and more.

Then, you can import the :py:mod:`swarmsim` package:

.. code-block:: python
   :caption: Python

   import swarmsim


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   guide/index
   api/index
   devel/index


