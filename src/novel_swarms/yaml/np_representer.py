import yaml

import numpy as np
import numpy


def represent_ndarray(dumper: yaml.Dumper, data: np.ndarray):
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
    function = np.array
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
