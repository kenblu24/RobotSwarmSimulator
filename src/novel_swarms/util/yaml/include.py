import pathlib as pl
import json

import yaml
from typing import Any, IO


class IncludeLoader(yaml.SafeLoader):
    """YAML Loader with `!include` constructor."""

    def __init__(self, stream: IO) -> None:
        """Initialise Loader."""
        self.file_path = pl.Path(stream.name)

        super().__init__(stream)


def construct_include(loader: IncludeLoader, node: yaml.Node) -> Any:
    """Include file referenced at node."""
    node_path = pl.Path(loader.construct_scalar(node))

    cwd = pl.Path.cwd()
    resolved = node_path.expanduser().resolve()
    not_cwd = resolved.exists() and not resolved.is_relative_to(cwd)

    # resolve order:
    # 1. absolute or resolvable/home paths (i.e. ~/foo.yaml)
    # 2. relative to yaml file
    # 3. relative to cwd
    if node_path.is_absolute() or not_cwd:
        node_path = resolved
    elif (path := loader.file_path.parent / node_path).exists():
        node_path = path
    elif (path := pl.Path.cwd() / node_path).exists():
        node_path = path
    else:
        msg = f"Could resolve path: {node_path}"
        raise FileNotFoundError(msg)

    ext = node_path.suffix

    with open(node_path, 'r') as f:
        if ext in ('.yaml', '.yml'):
            return yaml.load(f, IncludeLoader)
        elif ext in ('.json', ):
            return json.load(f)
        else:
            return ''.join(f.readlines())


yaml.add_constructor('!include', construct_include, IncludeLoader)


if __name__ == '__main__':
    with open('src/novel_swarms/util/yaml/test.yaml') as f:
        d = yaml.load(f, IncludeLoader)
    print(d)
