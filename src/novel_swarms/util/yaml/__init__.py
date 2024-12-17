import yaml
from .mathexpr import construct_numexpr
from .include import IncludeLoader, construct_include

from functools import partial

load = partial(yaml.load, Loader=IncludeLoader)
