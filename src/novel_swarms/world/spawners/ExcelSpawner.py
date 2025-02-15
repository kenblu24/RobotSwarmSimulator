from io import BytesIO
from .Spawner import Spawner

# typing
from typing import override


class ExcelSpawner(Spawner):
    def __init__(self, **dwargs):
        """
        Initialize the ExcelSpawner Class
        @params
        num_agents: The number of agents present in the simulation (int, required)
        bb: A Bounding box of the form ((x1, y1), (x2, y2)), where x1 and y1 are the Upper Left corner and
            x2, y2 are the Bottom Right corner.
        """
        super().__init__()
        # self.num_agents = dwargs.get("num_agents")

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

    def rescale(self, zoom_factor):
        # multiply the x and y coordinates by a constant zoom_factor
        # states is expected to be a list of tuples in the following order:
        # index, x, y, rotation
        c = zoom_factor
        self.states = [(x * c, y * c, r) for x, y, r in self.states]

    @staticmethod
    def extract_states_from_xlsx(fpath, sheet_number=0, usecols='B,C,D'):
        import pandas as pd
        with open(fpath, 'rb') as f:
            xlsx = f.read()
        xlsx = pd.ExcelFile(BytesIO(xlsx))
        dataframes = [pd.read_excel(xlsx, sheet_name=sheet, usecols=usecols) for sheet in xlsx.sheet_names]
        df = dataframes[sheet_number]
        return [(x, y, r) for _idx, x, y, r in df.itertuples()]

    def set_states_from_xlsx(self, fpath, sheet_number=0, usecols='B,C,D'):
        self.file_name = fpath
        self.states = self.extract_states_from_xlsx(fpath=fpath, sheet_number=sheet_number, usecols=usecols)
