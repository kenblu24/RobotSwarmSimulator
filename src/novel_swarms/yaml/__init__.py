import yaml
import pathlib

import numpy as np

from .mathexpr import construct_numexpr
from .include import IncludeLoader, construct_include
from .unknown import Tagged, construct_undefined, register_undefined

from functools import partial

yaml.add_constructor("!np", construct_numexpr, IncludeLoader)

load = partial(yaml.load, Loader=IncludeLoader)


class NaiveLoader(yaml.SafeLoader):
    pass


register_undefined(NaiveLoader)  # this ignores tags with no defined constructor and handles recursion
# Tagged objects will be wrapped in a Tagged object i.e. isinstance(obj, Tagged) will be True
# The tag is stored as a string, i.e. obj.tag
safe_load = partial(yaml.load, Loader=NaiveLoader)


class CustomDumper(yaml.Dumper):
    pass


def NDArrayRepresenter(dumper: yaml.Dumper, data: np.ndarray):
    is_vector = data.ndim == 1
    short_vector = is_vector and len(data) < 11
    very_short_vector = is_vector and len(data) < 3
    small_matrix = data.ndim <= 2 and np.prod(data.shape) <= 9
    if not (short_vector or small_matrix):
        # for big matrices, use the default representation
        return dumper.represent_object(data)
    # if short vector or small_matrix, use a more human-readable representation
    flow_style = False if very_short_vector else True  # use block style for very short vectors
    array_node = dumper.represent_sequence('tag:yaml.org,2002:seq', data.tolist(), flow_style=flow_style)
    tag = "tag:yaml.org,2002:python/object/apply:"
    function = np.ndarray
    function_name = f"{function.__module__}.{function.__name__}"
    value = {
        'kwargs': {
            'dtype': str(data.dtype),
            # 'order': 'F' if data.flags['F_CONTIGUOUS'] else 'C',
        },
        'args': (),
    }
    node = dumper.represent_mapping(tag + function_name, value)
    subnode = None
    for subnodes in node.value:
        if subnodes[0].value == 'args':
            subnodes[1].value = (array_node,)
    return node


def PathRepresenter(dumper: yaml.Dumper, data: pathlib.Path):
    tag = "tag:yaml.org,2002:python/object/apply:"
    function = type(data)
    function_name = f"{function.__module__}.{function.__name__}"
    value = {'args': [str(data)],}
    if isinstance(data, pathlib.Path):
        try:
            value['resolved'] = str(data.expanduser().resolve())
        except (FileNotFoundError, NotADirectoryError, OSError, RuntimeError):
            value['resolved'] = str(data.expanduser())
    return dumper.represent_mapping(tag + function_name, value)


CustomDumper.add_representer(np.ndarray, NDArrayRepresenter)

CustomDumper.add_representer(pathlib.Path, PathRepresenter)
CustomDumper.add_representer(pathlib.WindowsPath, PathRepresenter)
CustomDumper.add_representer(pathlib.PosixPath, PathRepresenter)
CustomDumper.add_representer(pathlib.PurePath, PathRepresenter)
CustomDumper.add_representer(pathlib.PurePosixPath, PathRepresenter)
CustomDumper.add_representer(pathlib.PureWindowsPath, PathRepresenter)


dump = partial(yaml.dump, Dumper=CustomDumper)
