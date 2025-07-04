from swarmsim import yaml

# typing
from os import PathLike


def load_custom_yaml(path: PathLike) -> tuple[dict, dict]:
    with open(path, "r") as yf:
        spec, world_setup = yaml.load_all(yf)
        return spec, world_setup
