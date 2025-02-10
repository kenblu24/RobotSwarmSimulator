************
Installation
************

This page describes how to install RobotSwarmSimulator.

Setting up your environment
===========================

.. note::
   If you're on Ubuntu or a Debian-based Linux distribution, you may need to use ``pyenv``
   to install :fab:`python` Python 3.11 or later.
   See the `pyenv installation instructions <https://github.com/pyenv/pyenv#installation>`_.
   You need to do this before creating the virtual environment.

.. important::
   If you're on Windows and need to install Python, please DO NOT use the Windows Store.
   Instead, download and install the latest version of Python from the `Python website <https://www.python.org/downloads/>`_.
   Make sure to check the box to add Python to your PATH:

   .. card::
      :far:`square-check` Add Python to PATH

   If you didn't do this when installing Python, you'll need to add it manually.
   See :ref:`python:setting-envvars` or `How to Add Python to PATH <https://realpython.com/add-python-to-path/>`_.

The recommended way to install RobotSwarmSimulator is with a **virtual environment**.

Virtual environments are isolated Python environments that allow you to install
packages without affecting your system Python installation.

First, you need to choose a location to store your virtual environment.

.. code-block:: bash

    mkdir swarm
    cd swarm

To create a virtual environment, we recommend using ``virtualenv``, but you can
still use ``venv`` if you prefer.

.. tab-set::
   :class: sd-width-content-min
   :sync-group: venv

   .. tab-item:: virtualenv (recommended)

      .. code-block:: bash

        pip install virtualenv
        virtualenv .

   .. tab-item:: venv

      .. code-block:: bash

        python -m venv .

This will create a virtual environment in your current directory.

.. _activate-venv:

Activating the virtual environment
----------------------------------

Once you have created your virtual environment, you need to activate it.

Make sure you're in the directory where you created the virtual environment.
In this example we're in the ``swarms/`` folder.

.. tab-set::
   :class: sd-width-content-min
   :sync-group: os

   .. tab-item:: Windows
      :sync: windows

      .. code-block:: bat

         .\Scripts\activate

   .. tab-item:: Linux / macOS / WSL
      :sync: posix

      .. code-block:: bash

         source bin/activate

You should see the name of your virtual environment in parentheses at the beginning of your terminal prompt:

.. tab-set::
   :class: sd-width-content-min
   :sync-group: os

   .. tab-item:: Windows
      :sync: windows

      .. code-block:: doscon

         (swarms) C:\swarms> 

   .. tab-item:: Linux / macOS / WSL
      :sync: posix

      .. code-block:: bash

         (swarms) user@host:~/swarms$

To deactivate the virtual environment, use the ``deactivate`` command:

.. code-block:: bash

   deactivate

.. _regular-install:

Installing RobotSwarmSimulator
==============================

To install RobotSwarmSimulator, we recommend using ``uv``.
You can preface most ``pip install`` commands with ``uv`` for *much* faster installation.

.. tab-set::
   :class: sd-width-content-min
   :sync-group: uv

   .. tab-item:: uv
      :sync: uv

      .. code-block:: bash

         pip install uv
         uv pip install git+ssh://git@github.com/kenblu24/RobotSwarmSimulator.git@main

   .. tab-item:: pip
      :sync: pip

      .. code-block:: bash

         pip install git+ssh://git@github.com/kenblu24/RobotSwarmSimulator.git@main

Development Installations
=========================

If you intend to contribute to RobotSwarmSimulator, you should follow the
:doc:`installation guide for development </devel/install>` instead.

.. button-ref:: /devel/install
   :color: primary
   :expand:

WSL Installation
================

If you're on Windows, you can install RobotSwarmSimulator in a Windows Subsystem for Linux (WSL) environment.
This is recommended if you're on Windows and want to use RobotSwarmSimulator.

First, you need to install WSL.
See the `Windows documentation <https://learn.microsoft.com/en-us/windows/wsl/install>`_ for instructions.

Then, follow the :ref:`regular-install` or :ref:`development-install` instructions.

Once you've done that, see :ref:`WSL Post-Installation` for instructions on how to get started with RobotSwarmSimulator.

.. todo::
   Add instructions for WSL post-installation and finish development install guide.
