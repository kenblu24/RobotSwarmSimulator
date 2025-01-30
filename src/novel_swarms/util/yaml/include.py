import pathlib as pl
import json

import yaml
from typing import Any, IO

# se: https://stackoverflow.com/questions/528281/how-can-i-include-a-yaml-file-inside-another
# metaclass example: https://gist.github.com/joshbode/569627ced3076931b02f?permalink_comment_id=2309157#gistcomment-2309157
# possibly helpful: https://matthewpburruss.com/post/yaml/


class IncludeLoader(yaml.SafeLoader):
    """YAML Loader with `!include` constructor."""

    def __init__(self, stream: IO) -> None:
        """Initialise Loader."""
        self.file_path = pl.Path(stream.name)

        super().__init__(stream)


def search_file(loader, path_str: str) -> pl.Path:
    """Include file referenced at node."""
    node_path = pl.Path(path_str)

    cwd = pl.Path.cwd()
    resolved = node_path.expanduser().resolve()
    not_cwd = resolved.exists() and not resolved.is_relative_to(cwd)

    # resolve order:
    # 1. absolute or resolvable/home paths (i.e. ~/foo.yaml)
    # 2. relative to yaml file
    # 3. relative to cwd
    if node_path.is_absolute() or not_cwd:
        return resolved
    elif (path := loader.file_path.parent / node_path).exists():
        return path
    elif (path := pl.Path.cwd() / node_path).exists():
        return path
    else:
        msg = f"Could resolve path: {node_path}"
        raise FileNotFoundError(msg)


def construct_include(loader: IncludeLoader, node: yaml.Node) -> Any:
    node_path = search_file(loader, loader.construct_scalar(node))

    ext = node_path.suffix

    with open(node_path, 'r') as f:
        if ext in ('.yaml', '.yml'):
            return yaml.load(f, IncludeLoader)
        elif ext in ('.json', ):
            return json.load(f)
        else:
            return ''.join(f.readlines())


def construct_relative_path(loader: IncludeLoader, node: yaml.Node) -> Any:
    node_path = search_file(loader, loader.construct_scalar(node))
    return str(node_path.resolve().absolute())


yaml.add_constructor('!include', construct_include, IncludeLoader)
yaml.add_constructor('!relpath', construct_relative_path, IncludeLoader)


if __name__ == '__main__':
    with open('src/novel_swarms/util/yaml/test.yaml') as f:
        d = yaml.load(f, IncludeLoader)
    print(d)
