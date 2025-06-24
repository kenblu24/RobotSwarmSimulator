**************************
Installing for Development
**************************

This page describes how to install RobotSwarmSimulator for development.

.. admonition:: This is a guide for **internal contributors** to RobotSwarmSimulator.
   
   It assumes you're added to the github repository as a collaborator
   and uses **SSH URLs** for git. External contributors should fork the repository <https://github.com/kenblu24/RobotSwarmSimulator/fork>

.. seealso::
   This guide was written with a Unix-like operating system in mind, such as :fab:`linux` Linux or :fab:`apple` macOS.
   If you're on :fab:`windows` Windows, consider using WSL :doc:`/guide/install-wsl` which allows you to follow this guide
   as if you were on Linux. Then, see :ref:`wsl-post-install` for further setup.


Installing git
==============


If you're contributing to RobotSwarmSimulator, you should probably be using :fab:`git-alt` :fab:`git` git version control.

.. button-link:: https://git-scm.com/downloads
   :ref-type: myst
   :color: primary

   :fab:`git-alt` Download git :fas:`arrow-up-right-from-square`

Once you have git, make sure you can run ``git`` from the command line. If not, you may need to restart your terminal.

.. _rss-install-ssh-keys:

SSH keys
========

This guide uses SSH URLs for git. You'll need to have an SSH key set up and added to your GitHub account
to clone, pull, or push to/from the remote repository.
If you don't have an SSH key on your system, you'll need to generate one.

.. code-block:: console
   :caption: "Check if you have an SSH key already"

   $ ls -A ~/.ssh
   id_rsa  id_rsa.pub  known_hosts

You should see ``id_rsa.pub`` or ``id_ed25519.pub`` in the output.
If you don't, you'll need to generate a new SSH key.

.. caution::
   Be aware that existing SSH keys may be used by other applications. If you delete or overwrite an existing key,
   you may need to re-add it wherever it was used.

.. code-block:: console
   :caption: Generate a new SSH key

   $ ssh-keygen

Copy & paste the contents of your ``id_rsa.pub`` or ``id_ed25519.pub`` file into your GitHub account's SSH keys page.
Make sure to give the key a descriptive title; you won't be able to change it later.

.. button-link:: https://github.com/settings/ssh/new
   :ref-type: myst
   :color: secondary

   :fas:`key` Add :fab:`github` GitHub SSH key :fas:`arrow-up-right-from-square`


See the `GitHub documentation <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent>`_ for more information.


Python Installation
===================

If you're on :fab:`ubuntu` Ubuntu or a :fab:`debian` Debian-based Linux distribution (including :fab:`windows` :fab:`linux` WSL), you should use ``pyenv``
to install :fab:`python` Python 3.11 or later.

This allows you to install any Python version you want, without affecting your system Python installation.
See the `pyenv installation instructions <https://github.com/pyenv/pyenv#installation>`_.

.. code-block:: bash
   :caption: Install & switch to Python>=3.11

   pyenv install 3.13
   pyenv global 3.13

Then, make sure we're actually using the right version of Python.
You should see something similar to this:

.. code-block:: console
   :caption: Check the python version and make sure ``_ctypes`` is available

   $ which python
   /home/username/.pyenv/shims/python
   $ python --version
   Python 3.13.0
   $ python -c "import _ctypes"
   $ pip --version
   pip 24.2 from /home/username/.pyenv/versions/3.13.0/lib/python3.13/site-packages/pip (python 3.13)


:fab:`windows` Native Windows users can use a Python installer from `Python.org <https://www.python.org/downloads/>`_.
Make sure to check the box to add Python to PATH.


.. hint::
   This needs to be done before creating the virtual environment, as ``uv venv`` or ``virtualenv``
   will use whatever version of Python it finds when you run it. Running ``which python`` may help you know more.

   If you already made the virtual environment, the easiest way to fix this is to delete the virtual environment and start over.

.. seealso::
   If you're running Tennlab simulations on the **Hopper cluster**, please use the `hopper install scripts <https://github.com/GMU-ASRC/neuroswarm/tree/main/scripts/hopper>`_.


.. _rss-install-editable:

Downloading & Installing as editable
====================================

We recommend using UV which provides environment tools and faster installs.

.. dropdown:: Install UV for faster installs
   :color: secondary
   :open:

   .. code-block:: bash
      :caption: Install ``uv`` <https://github.com/pyuv/uv> for faster installs

      pip install uv -U

   The ``-U`` flag is shorthand for ``--upgrade``.
   
   You can preface most ``pip install`` commands with ``uv`` for *much* faster installation.
   ``uv pip install`` may not work for some packages. If you get an error, try using regular ``pip install`` first.


First, let's make a project folder and **virtual environment**. Pick a place
to store your virtual environment. In this example, we'll use the ``swarm/`` folder.

.. code-block:: bash
   :caption: Make a project folder and virtual environment

   mkdir swarm
   cd swarm

Next, we can create the virtual environment.

.. tab-set::
   :class: sd-width-content-min
   :sync-group: uv

   .. tab-item:: uv
      :sync: uv     

      .. code-block:: bash
         :caption: Create a virtual environment

         uv venv
         

   .. tab-item:: pip
      :sync: pip

      .. code-block:: bash
         :caption: Create a virtual environment

         pip install virtualenv
         virtualenv .venv --prompt .

Now, we need to activate the virtual environment.

.. tab-set::
   :class: sd-width-content-min
   :sync-group: os

   .. tab-item:: :fab:`windows` Windows
      :sync: windows

      .. code-block:: bat

         .venv\Scripts\activate

   .. tab-item:: :fab:`linux` Linux / :fab:`apple` macOS / :fab:`windows`\ :fab:`linux` WSL
      :sync: posix

      .. code-block:: bash

         source .venv/bin/activate

.. admonition:: Activating fish, Nushell, or PowerShell

   The above activation command is for the default shell environments, such as ``bash``, ``zsh``, or ``sh`` on Unix, or ``cmd`` and ``powershell`` on Windows.
   If you're using a different shell, such as ``fish`` or ``Nushell``, or if you're using PowerShell and have activation issues, you may need to use a different activation file.

   .. tab-set::
      :class: sd-width-content-min
      :sync-group: shell

      .. tab-item:: fish
         :sync: fish

         .. code-block:: fish

            source .venv/bin/activate.fish

      .. tab-item:: PowerShell
         :sync: powershell

         .. code-block:: powershell

            .venv\bin\activate.ps1
            

      .. tab-item:: Nushell
         :sync: nushell

         .. tab-set::
            :class: sd-width-content-min
            :sync-group: os

            .. tab-item:: :fab:`windows` Windows
               :sync: windows

               .. code-block:: powershell

                  overlay use .venv\Scripts\activate.nu

            .. tab-item:: :fab:`linux` Linux / :fab:`apple` macOS / :fab:`windows`\ :fab:`linux` WSL
               :sync: posix

               .. code-block:: bash

                  overlay use .venv/bin/activate.nu

You can deactivate the virtual environment with the ``deactivate`` command.

Then, let's `git clone` the RobotSwarmSimulator repository.

.. code-block:: bash
   :caption: git clone the RobotSwarmSimulator repository and ``cd`` into it

   git clone https://github.com/kenblu24/RobotSwarmSimulator.git
   cd RobotSwarmSimulator

.. admonition:: SSH URLs

   If you're contributing to or modifying RobotSwarmSimulator, you should use SSH URLs for git.
   See :ref:`rss-install-ssh-keys` for more information.

   GitHub won't let you push to HTTPS remote URLs using password authentication. If you choose to use the HTTPS URL as shown above,
   you'll need to create a `personal access token <https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token>`_
   and use that as the password every time you push.

   However, if you successfully set up your SSH key in the `section above <snm-install-ssh-keys>`_, and have contributor-level permissions on GitHub,
   you can use the SSH URL instead.

   .. code-block:: bash
      :caption: git clone the RobotSwarmSimulator repository (using SSH URL) and ``cd`` into it

      git clone git@github.com:kenblu24/RobotSwarmSimulator.git
      cd RobotSwarmSimulator

   Again, if you're not an internal contributor, you'll need to fork the <https://github.com/kenblu24/RobotSwarmSimulator/fork> repository and use the URL for your fork.


A ``pip --editable`` install allows you to make changes to the code and see the effects immediately.

.. dropdown:: Install UV for faster installs
   :color: secondary
   :open:

   You can preface most ``pip install`` commands with ``uv`` for *much* faster installation.

   .. code-block:: bash
      :caption: Install ``uv`` <https://github.com/pyuv/uv> for faster installs

      pip install uv

   ``uv pip install`` may not work for some packages. If you get an error, try using regular ``pip install`` first.

It's finally time to install RobotSwarmSimulator into our virtual environment!

We'll use a ``pip --editable`` install allows you to make changes to the code and see the effects immediately.

.. hint::

   Don't forget to activate the virtual environment,
   and make sure you're in the RobotSwarmSimulator folder before running this command!

   (The ``.`` refers to the current directory. If you're one level above, you can do something like ``pip install -e RobotSwarmSimulator[dev,docs]``.)

.. tab-set::
   :class: sd-width-content-min
   :sync-group: uv

   .. tab-item:: uv
      :sync: uv

      .. code-block:: bash

         uv pip install -e .[dev,docs]

   .. tab-item:: pip
      :sync: pip

      .. code-block:: bash

         pip install -e .[dev,docs]
         
The ``.`` refers to the current directory, and the ``[docs,dev]`` refers to the optional dependencies.
``[docs]`` refers to the dependencies for building the documentation, and ``[dev]`` refers to development and testing dependencies.

All these dependencies are specified in the ``RobotSwarmSimulator/pyproject.toml`` file, in the ``[project]`` ``dependencies`` section,
and the ``[project.optional-dependencies]`` section.



While you're here, let's also install ``pyreadline3`` which makes the ``python`` shell much more user-friendly.

.. tab-set::
   :class: sd-width-content-min
   :sync-group: uv

   .. tab-item:: uv
      :sync: uv

      .. code-block:: bash

         uv pip install pyreadline3

   .. tab-item:: pip
      :sync: pip

      .. code-block:: bash

         pip install pyreadline3

If the installation was successful, you should be able to open a ``python`` shell and import the package:

.. code-block:: python-console
   :caption: ``python``

   Python 3.11.0 (or newer)
   Type "help", "copyright", "credits" or "license" for more information.
   >>> import swarmsim
   >>> 


If you installed ``pyreadline3`` or are using Python 3.13 or newer, you can exit the ``python`` shell with :kbd:`Ctrl+C` to stop
currently running commands and then :kbd:`Ctrl+D`. Or you can type ``quit()`` to quit the python REPL.

-----

.. card::
   :link: /guide/firstrun
   :link-type: doc
   :link-alt: First Run Tutorial
   :margin: 3

   Finished installing? Check out the :doc:`/guide/firstrun` tutorial.  :fas:`circle-chevron-right;float-right font-size-1_7em`