import sys
import pandas as pd
import argparse
import pathlib
import yaml

from .OptimVar import CMAESVarSet
from CMAES import CMAES_NAME, GENOMES_NAME, CONFIG_NAME


DEFAULT_GENOME = pathlib.Path("CMAES/genomes.csv")
DEFAULT_CMAES = pathlib.Path("CMAES/optim-config.yaml")


class CMAESConfigReader:
    def __init__(self, path):
        self.path = path
        if not self.path.exists():
            raise FileNotFoundError(f"Error: could not find CMAES config at the specified location: {path}")  # noqa: EM102

        self.raw_config: dict | None = None

    def _read(self, path):
        with open(path, "r") as f:
            return yaml.safe_load(f)

    def load(self):
        self.raw_config = self._read(self.path)

    def is_loaded(self, complain=False):
        loaded = self.raw_config is not None
        if complain and not loaded:
            raise RuntimeError("CMAES config not loaded.")
        return loaded

    @property
    def d(self):
        if not self.is_loaded():
            self.load()
        return self.raw_config


class CMAESExperimentReader:
    def __init__(self, path):
        self.root = path
        self.cmaes_path = self.root / CMAES_NAME
        self.genome_file = self.cmaes_path / GENOMES_NAME
        self.config = CMAESConfigReader(self.cmaes_path / CONFIG_NAME).d

        try:
            self.dvars = CMAESVarSet.from_ordered_dict(self.config['dvars'])
        except BaseException:
            self.dvars = None

        if not self.root.exists():
            raise FileNotFoundError(f"Could not find experiment at {self.root}. Is the directory path correct?")  # noqa: EM102
        if not self.genome_file.exists():
            raise FileNotFoundError(f"Error: could not find CMAES results at the specified location. Was an experiment successfully run here?")  # noqa: EM102, E501

    def get_data(self):
        return pd.read_csv(self.genome_file)

    def denormalize(self, data):
        # bounds = self.config['bounds']
        varset = CMAESVarSet.from_ordered_dict(self.config['dvars']) if self.dvars is None else self.dvars
        return varset.from_scaled_to_unit(data)

    def convert_to_EONS_df(self, data):
        # TODO: UNTESTED, was copied over without modification
        res = [[None, 0, repr([])]]
        n_epochs = max(data['gen'])
        for i in range(n_epochs):
            time = data.loc[data["gen"] == i].head(1)["time"].item()
            epoch = i + 1
            fitness = list(data.loc[data["gen"] == i]["Circliness"])
            l = [time, epoch, repr(fitness)]  # noqa: E741
            res.append(l)
        return pd.DataFrame(res, columns=["time", "epoch", "fitness"])





def main(args):
    reader = CMAESExperimentReader(args.experiment_path)

    data = reader.get_data()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment_path", type=pathlib.Path)

    args = parser.parse_args()

    if not sys.flags.interactive:
        main(args)

    # INTERACTIVE MODE
    reader = CMAESExperimentReader(args.experiment_path)
    data = reader.get_data()

