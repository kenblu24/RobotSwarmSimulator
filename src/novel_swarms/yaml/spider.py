import pathlib as pl

import ruamel.yaml

from .include import search_file, INCLUDE_TAG, RELPATH_TAG

from typing import Any


class Spider(ruamel.yaml.YAML):
    def __init__(self, *args, toplevel=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.constructor.add_constructor(INCLUDE_TAG, self.construct_include)
        self.constructor.add_constructor(RELPATH_TAG, self.construct_relative_path)

        self.toplevel = toplevel or self

        self.rel_paths = {}
        self.included_yamls = {}

    @property
    def file_path(self) -> pl.Path:
        return pl.Path(self.reader.stream.name)

    @staticmethod
    def construct_include(constructor, node: ruamel.yaml.Node) -> Any:
        loader = constructor.loader
        assert loader is not None  # constructor's loader should have been set to self by loader
        node_path = search_file(loader.file_path.parent, constructor.construct_scalar(node))

        if node_path in constructor.included_yamls:
            return constructor.construct_scalar(node)
        ext = node_path.suffix

        if ext in ('.yaml', '.yml'):
            with open(node_path, 'r') as f:
                new_yaml = Spider(toplevel=loader.toplevel)
                loader.included_yamls[node_path] = (new_yaml, loader.load(f))

        return constructor.construct_scalar(node)


    @staticmethod
    def construct_relative_path(constructor, node: ruamel.yaml.Node) -> str:
        loader = constructor.loader
        assert loader is not None  # constructor's loader should have been set to self by loader
        node_path = search_file(constructor.file_path.parent, loader.construct_scalar(node))
        return str(node_path.expanduser().resolve())

    @staticmethod
    def construct_moved_file(constructor, node: ruamel.yaml.Node) -> str:
        loader = constructor.loader
        assert loader is not None  # constructor's loader should have been set to self by loader
        node_path = search_file(constructor.file_path.parent, loader.construct_scalar(node))
        return str(node_path.resolve().absolute())


if __name__ == '__main__':
    this_dir = pl.Path(__file__).resolve().parent
    with open(this_dir / 'test.yaml') as f:
        spider = Spider()
        d = spider.load(f)
    print(d)
