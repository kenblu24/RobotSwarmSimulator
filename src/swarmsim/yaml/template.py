from collections import ChainMap
import json

import yaml

from ..util.jinja import make_template_env, load_template, load_template_ctx
from .include import IncludeLoader, INCLUDE_TAG, search_file

# typing
from typing import Any, IO


#
# Jinja Template Preprocessing when loading YAML
#
# Load order for load_yaml_with_ctx():
# file.yaml
# 1. Preprocess jinja into pure YAML, save exported jinja vars
# 2. Compose YAML
# 3. Merge <<: !include x.yaml, its exported vars are added beneath parent context
# 4. Construct all YAML objects, processing non-merge key !include tags
# 5. !include file2.yaml
# 6.    Preprocess file2.yaml jinja with the parent context. Parent context can't be modified by file2.yaml.
# 7.    etc.
# ... return the final YAML object


class TemplateYAMLLoader(IncludeLoader):
    def __init__(self, stream: IO | str, name=None, jinja_env=None, jinja_ctx=None) -> None:
        """Initialise Loader."""
        super().__init__(stream, name=name)
        self.jinja_env = jinja_env or make_template_env()
        self.jinja_ctx = {} if jinja_ctx is None else jinja_ctx

    # for merge keys in YAML i.e. <<: !include filename.yaml
    def compose_include_merge_node(self, node):
        path = search_file(self.file_path.parent, node.value)
        s, ctx = load_template_ctx(path, env=self.jinja_env, yaml_loader=self.__class__, **self.jinja_ctx)
        self.jinja_ctx = ChainMap(self.jinja_ctx, ctx)
        import yaml
        return yaml.compose(s, self.__class__)


def construct_include(loader: TemplateYAMLLoader, node: yaml.Node) -> Any:
    """Read the contents of a yaml/text/json file into a node"""
    node_path = search_file(loader.file_path.parent, loader.construct_scalar(node))  # pyright: ignore[reportArgumentType]

    ext = node_path.suffix

    if ext in ('.yaml', '.yml'):
        return load_yaml_template(node_path, name=node_path, env=loader.jinja_env,
                                    yaml_loader_cls=loader.__class__, context=loader.jinja_ctx)
    else:
        s = load_template(node_path)
        if ext in ('.json', ):
            return json.load(s)
        return s


yaml.add_constructor(INCLUDE_TAG, construct_include, TemplateYAMLLoader)


def load_yaml_template(path, name=None, env=None, yaml_loader_cls=TemplateYAMLLoader, context=None, **kwargs):
    """
    Parse the first YAML document in a stream
    and produce the corresponding Python object.
    """
    if name is None:
        name = path
    ctx = ChainMap(kwargs)
    if context is not None:
        ctx.maps.append(context)
    s, exported = load_template_ctx(path, env=env, **ctx)
    ctx.maps.append(exported)
    loader = yaml_loader_cls(s, name=name, jinja_env=env, jinja_ctx=ctx)
    try:
        return loader.get_single_data()
    finally:
        loader.dispose()
