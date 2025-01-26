import yaml
from .mathexpr import construct_numexpr
from .include import IncludeLoader, construct_include
from .unknown import Tagged, construct_undefined, register_undefined

from functools import partial

load = partial(yaml.load, Loader=IncludeLoader)


class NaiveLoader(yaml.SafeLoader):
    pass


register_undefined(NaiveLoader)  # this ignores tags with no defined constructor and handles recursion
# Tagged objects will be wrapped in a Tagged object i.e. isinstance(obj, Tagged) will be True
# The tag is stored as a string, i.e. obj.tag
safe_load = partial(yaml.load, Loader=NaiveLoader)
