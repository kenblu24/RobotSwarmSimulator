"""Module for importing/exporting agent properties from/to spreadsheets.

Examples
--------

Saving agent properties to a spreadsheet:

.. code-block:: python
    :caption: my_world.py

    from swarmsim.world.spawners.ExcelSpawner import world_to_df
    from swarmsim.world import config_from_yaml
    from swarmsim.util.pdutils import save_df

    # load the world config with your spawners and agents
    world = config_from_yaml('world.yaml').create_world()
    # setup world, create agents and spawners and do a spawn cycle
    world.setup(step_spawners=True)
    # save agent properties to a spreadsheet
    df = world_to_df(world, as_repr=repr)
    save_df(df, 'test.xlsx', sheet_name='sheet1')

Loading agent properties from a spreadsheet:

.. code-block:: YAML
    :caption: world.yaml

    type: "RectangularWorld"
      - type: ExcelSpawner
        file_name: 'test.xlsx'
        sheet: sheet1
        agent:
          type: MazeAgent
          agent_radius: 0.15

Class
-----

.. autoclass:: ExcelSpawner

Functions
---------

.. autofunction:: world_to_df
.. autofunction:: states_from_world
.. autofunction:: states_to_df

"""

import os
import pathlib as pl
from io import BytesIO

from .AgentSpawner import BaseAgentSpawner
from ...util import pdutils
from ...config import get_agent_class
from ...yaml.mathexpr import safe_eval
from ...agent.Agent import Agent, BaseAgentConfig

# typing
from typing import Callable, override


class ExcelSpawner(BaseAgentSpawner):
    def __init__(self, world=None, n=None, file_name: str | os.PathLike = '', sheet=0, load_args=None, **kwargs):
        """
        Spawn agents with properties from a spreadsheet.
        """
        super().__init__(world, n=n, **kwargs)
        # self.num_agents = dwargs.get("num_agents")

        self.file_name = pl.Path(file_name)
        self.load_args = load_args or {}
        self.sheet = sheet

        self.states = []
        self.sheets = []
        self.sheet_names = []
        self.loaded = False

    def get_sheet(self, sheet: int | str):
        if isinstance(sheet, int):
            return self.sheets[sheet]
        elif isinstance(sheet, str):
            return self.sheets[self.sheet_names.index(sheet)]

    def load_xlsx(self, pdargs=None):
        import pandas as pd
        if pdargs is None:
            pdargs = self.load_args
        with open(self.file_name, 'rb') as f:
            xlsx = f.read()
        xlsx = pd.ExcelFile(BytesIO(xlsx))
        dataframes = [pd.read_excel(xlsx, sheet, **pdargs) for sheet in xlsx.sheet_names]

        def eval_filtered(cell):
            return safe_eval(cell) if isinstance(cell, str) else cell

        self.sheets = [df.map(eval_filtered) for df in dataframes]
        self.sheet_names = xlsx.sheet_names
        self.loaded = True

    def load_csv(self, pdargs=None):
        import pandas as pd
        if pdargs is None:
            pdargs = self.load_args
        with open(self.file_name, 'r') as f:
            df = pd.read_excel(f, **pdargs)

        def eval_filtered(cell):
            return safe_eval(cell) if isinstance(cell, str) else cell

        self.sheets = [df.map(eval_filtered)]
        self.sheet_names = ['0']
        self.loaded = True

    def generate_config(self, props):
        config = super().generate_config()

        for prop, value in props.items():
            setattr(config, prop, value)

        return config

    def load_from_file(self):
        if self.file_name.suffix == '.xlsx':
            self.load_xlsx()
        elif self.file_name.suffix == '.csv':
            self.load_csv()

    def step(self):
        if self.mode.startswith('oneshot'):
            self.mark_for_deletion = True
        elif (
            self.n_objects is not None and self.spawned > self.n_objects
            or self.mark_for_deletion
            or self.loaded and not self.sheets  # if we already loaded data but no more data is available
        ):
            self.mark_for_deletion = True
            return
        if not self.loaded:
            # setup
            self.load_from_file()

        if self.mode in ('oneshot', 'onesheet') and self.sheets:
            self.states = [x for _i, x in self.get_sheet(self.sheet).iterrows()]
            for _idx in self.states[:self.n_objects]:
                self.do_spawn()

    def do_spawn(self):
        config = self.generate_config(self.states.pop(0))
        agent = self.make_agent(config)
        self.world.population.append(agent)  # make world aware of the new agent. necessary for collision handling
        self.spawned += 1
        return agent

    # LEGACY CODE
    # -----------

    @override
    def set_to_world(self, world):
        # Set the initialization of the world agents. Legacy code.
        for i, agent in enumerate(world.population):
            x, y, r = self.states[i]  # only assign to agents in index
            agent.set_pos_vec((x, y, r))
            agent.name = f"{i}"

    @staticmethod
    def extract_states_from_xlsx(fpath, sheet_number=0, usecols='B,C,D'):
        # get the states from an excel file. Legacy code.
        import pandas as pd
        with open(fpath, 'rb') as f:
            xlsx = f.read()
        xlsx = pd.ExcelFile(BytesIO(xlsx))
        dataframes = [pd.read_excel(xlsx, sheet_name=sheet, usecols=usecols) for sheet in xlsx.sheet_names]
        df = dataframes[sheet_number]
        return [(x, y, r) for _idx, x, y, r in df.itertuples()]

    def set_states_from_xlsx(self, fpath, sheet_number=0, usecols='B,C,D'):
        # Get states to be set on agents from an excel file. Legacy code.
        self.file_name = fpath
        self.states = self.extract_states_from_xlsx(fpath=fpath, sheet_number=sheet_number, usecols=usecols)


def states_from_world(world, properties=None, as_repr: str | bool | Callable = 'str'):
    if properties is None:
        properties = ['name', 'position', 'angle']
    for agent in world.population:
        states = []
        for prop_name in properties:
            prop = getattr(agent, prop_name)
            if as_repr == 'str' or as_repr is str:
                states.append(str(prop))
            elif as_repr == 'repr' or as_repr is repr or bool(as_repr):
                states.append(repr(prop))
            else:
                states.append(prop)
        yield states


def states_to_df(states, properties=None):
    import pandas as pd
    if properties is None:
        properties = ['name', 'position', 'angle']
    return pd.DataFrame(states, columns=properties)


def world_to_df(world, properties=None, as_repr: str | bool | Callable = 'str'):
    """Get agent properties from a world's population as a pandas DataFrame.

    Parameters
    ----------
    world : World
        The world to get the agent properties from.
    properties : _type_, default=[name', 'position', 'angle']
        The properties of agents to scrape. These should be accessible as :samp:`agent.{property_name}`
    as_repr : str | bool | Callable, default='str'
        How to represent the agent properties.
        If 'str', the properties will be converted to strings.
        If 'repr', ``repr()`` will be called with the properties.

    Returns
    -------
    pandas.DataFrame
    """
    import pandas as pd
    if properties is None:
        properties = ['name', 'position', 'angle']
    return pd.DataFrame(states_from_world(world, properties, as_repr), columns=properties)
