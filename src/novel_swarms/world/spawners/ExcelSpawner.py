from io import BytesIO
from .Spawner import Spawner

# typing
from typing import override


class ExcelSpawner(Spawner):
    def __init__(self, world, properties=None, **kwargs):
        """
        Initialize the ExcelSpawner Class
        @params
        num_agents: The number of agents present in the simulation (int, required)
        bb: A Bounding box of the form ((x1, y1), (x2, y2)), where x1 and y1 are the Upper Left corner and
            x2, y2 are the Bottom Right corner.
        """
        super().__init__(world)
        # self.num_agents = dwargs.get("num_agents")

        if properties is None:
            self.properties = ['name', 'position', 'angle']
        else:
            self.properties = properties

        self.file_name = ""
        self.sheets = []

        # Store the initialization information to be added to the world later using self.set_to_world()
        self.states = []

    @override
    def set_to_world(self, world):
        """
        Set the initialization of the world agents.
        """
        for i, agent in enumerate(world.population):
            x, y, r = self.states[i]  # only assign to agents in index
            agent.set_pos_vec((x, y, r))
            agent.name = f"{i}"

    @staticmethod
    def extract_states_from_xlsx(fpath, sheet_number=0, usecols='B,C,D'):
        import pandas as pd
        with open(fpath, 'rb') as f:
            xlsx = f.read()
        xlsx = pd.ExcelFile(BytesIO(xlsx))
        dataframes = [pd.read_excel(xlsx, sheet_name=sheet, usecols=usecols) for sheet in xlsx.sheet_names]
        df = dataframes[sheet_number]
        return [(x, y, r) for _idx, x, y, r in df.itertuples()]

    def extract_states_from_world(self, world, as_repr=False):
        for agent in world.population:
            states = []
            for prop_name in self.properties:
                prop = getattr(agent, prop_name)
                states.append(repr(prop) if as_repr else prop)
            yield states

    def states_to_df(self, states):
        import pandas as pd
        return pd.DataFrame(states, columns=self.properties)


    def set_states_from_xlsx(self, fpath, sheet_number=0, usecols='B,C,D'):
        self.file_name = fpath
        self.states = self.extract_states_from_xlsx(fpath=fpath, sheet_number=sheet_number, usecols=usecols)
