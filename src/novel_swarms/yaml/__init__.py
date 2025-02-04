import yaml

from .mathexpr import construct_numexpr
from .include import IncludeLoader, construct_include
from .unknown import Tagged, construct_undefined, register_undefined
from .pathlib_representer import pathlib, represent_path
from .np_representer import numpy, represent_ndarray

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


CustomDumper.add_representer(numpy.ndarray, represent_ndarray)

CustomDumper.add_representer(pathlib.Path, represent_path)
CustomDumper.add_representer(pathlib.WindowsPath, represent_path)
CustomDumper.add_representer(pathlib.PosixPath, represent_path)
CustomDumper.add_representer(pathlib.PurePath, represent_path)
CustomDumper.add_representer(pathlib.PurePosixPath, represent_path)
CustomDumper.add_representer(pathlib.PureWindowsPath, represent_path)


dump = partial(yaml.dump, Dumper=CustomDumper)
