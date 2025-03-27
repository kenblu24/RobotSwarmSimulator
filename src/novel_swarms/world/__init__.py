"""Houses World modules.

Worlds are the root of the simulation.

All worlds must inherit from :py:class:`~novel_swarms.world.World.World`

.. currentmodule:: novel_swarms.world

The world is simulated by the :py:mod:`~novel_swarms.world.simulate` module.

.. rubric:: Subpackages

.. autosummary::

   generation
   goals
   objects
   spawners
   subscribers

Functions
=========

.. autofunction:: novel_swarms.world.World.World_from_config
   :noindex:

.. autofunction:: novel_swarms.world.World.config_from_dict
   :noindex:

.. autofunction:: novel_swarms.world.World.config_from_yaml
   :noindex:

.. autofunction:: novel_swarms.world.World.config_from_yamls
   :noindex:


"""

from .World import World_from_config, config_from_dict, config_from_yaml, config_from_yamls
from .simulate import main as sim

__all__ = ['World_from_config', 'config_from_dict', 'config_from_yaml', 'config_from_yamls', 'sim']
