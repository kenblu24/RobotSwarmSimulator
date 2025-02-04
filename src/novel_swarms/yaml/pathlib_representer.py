import yaml
import pathlib


def represent_path(dumper: yaml.Dumper, data: pathlib.Path):
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
