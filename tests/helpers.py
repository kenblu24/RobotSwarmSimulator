from swarmsim import yaml

# typing
from os import PathLike
from pathlib import Path


def load_custom_yaml(path: str | PathLike) -> tuple[dict, dict]:
    if isinstance(path, str):
        path = Path(path)

    with open(path, "r") as yf:
        spec, world_setup = yaml.load_all(yf)
        return spec, world_setup
