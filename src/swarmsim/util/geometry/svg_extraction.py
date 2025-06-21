import re
from itertools import batched
import xml.dom.minidom as pydom

import numpy as np
from math import radians

SVG_NS = 'http://www.w3.org/2000/svg'

IDENTITY = np.identity(4)

RE_TRANSLATE = re.compile(r'translate\s*\(\s*(?P<x>[+-]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][+-]?\d+)?)(?P<y>(?:\s+,?\s*|,\s*)?[+-]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][+-]?\d+)?)?\s*\)')
RE_SCALE = re.compile(r'scale\s*\(\s*(?P<x>[+-]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][+-]?\d+)?)(?P<y>(?:\s+,?\s*|,\s*)?[+-]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][+-]?\d+)?)?\s*\)')
RE_ROTATE = re.compile(r'rotate\s*\(\s*([+-]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][+-]?\d+)?)(?:((?:\s+,?\s*|,\s*)?[+-]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][+-]?\d+)?)((?:\s+,?\s*|,\s*)?[+-]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][+-]?\d+)?))?\s*\)')


def rect_from_wh(x, y, w, h):  # clockwise winding
    return np.array([
        [x, y],
        [x + w, y],
        [x + w, y + h],
        [x, y + h]
    ])


# NOT compliant with SVG spec
# https://www.w3.org/TR/css-transforms-1/
# https://www.w3.org/TR/SVG/coords.html#TransformProperty
def get_transform(transform):
    shift = IDENTITY
    scale = IDENTITY
    rotate = IDENTITY
    translates = RE_TRANSLATE.findall(transform)
    rotates = RE_ROTATE.findall(transform)
    scales = RE_SCALE.findall(transform)
    if translates:
        translation = [float(x) for x in translates[-1] if x]
        match(translation):
            case (x, y):
                x, y = x, y
            case (x,):
                x, y = x, 0
        shift = IDENTITY.copy()
        shift[0:2, 3] = [x, y]
    if rotates:
        rotates = [float(x) for x in rotates[-1] if x]
        match(rotates):
            case (_x, _y, _z):
                raise NotImplementedError("Rotation in more than 1 axis not supported.")
            case (angle,):
                rotate = IDENTITY.copy()
                angle = radians(angle)
                rotate[0:2, 0:2] = np.array([
                    [np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]],
                dtype=np.float64)
    if scales:
        scales = [float(x) for x in scales[-1] if x]
        match(scales):
            case (x, y):
                x, y = x, y
            case (x,):
                x, y = x, x
        scale = IDENTITY.copy()
        scale[0, 0] = x
        scale[1, 1] = y

    matrix = shift @ scale @ rotate
    return matrix


def apply_transform(points, transform):
    dim = points.shape[1]
    new = np.pad(points, ((0, 0), (0, (4 - dim))), 'constant', constant_values=1)
    new = np.asarray([transform @ p for p in new])
    return new[:, :dim]


def get_parents_layernames(elem):
    parents = []
    elem = elem.parentNode
    while elem is not None:
        if elem.nodeName == 'g':
            if elem.hasAttribute('data-name'):
                parents.append(elem.attributes['data-name'].value)
            elif elem.hasAttribute('id'):
                parents.append(elem.attributes['id'].value)
        elem = elem.parentNode
    return parents


def remove_classes(names: list[str] | str, classes: list[str]):
    if isinstance(names, str):
        names = names.split(' ')
        return ' '.join([name for name in names if name not in classes])
    return [name for name in names if name not in classes]


def first_match(names: list[str], classes: list[str]):
    for name in names:
        if name in classes:
            return name


class SVG:
    def __init__(self, svg_content: str):
        self.dom = pydom.parseString(svg_content)

    # def get_paths(self):
    #     path_elems = self.root.findall('.//{http://www.w3.org/2000/svg}path')
    #     return [parse_path(elem.attrib['d']) for elem in path_elems]

    def get_polygons(self):
        polys = self.dom.getElementsByTagNameNS(SVG_NS, 'polygon')
        poly_points = [elem.attributes['points'].value for elem in polys]
        poly_points = [points.split() for points in poly_points]  # split space-separated numbers
        poly_points = [[float(point) for point in points] for points in poly_points]  # convert to floats
        coords = [list(batched(points, 2)) for points in poly_points]  # group into coordinate pairs
        parents = [get_parents_layernames(elem) for elem in polys]
        return list(zip(coords, parents))  # returns [] if no polygons found

    def get_rects(self):
        rects = []
        elements = self.dom.getElementsByTagNameNS(SVG_NS, 'rect')
        for elem in elements:
            attr = elem.attributes
            x = float(attr['x'].value)
            y = float(attr['y'].value)
            width = float(attr['width'].value)
            height = float(attr['height'].value)
            points = rect_from_wh(x, y, width, height)
            if 'transform' in attr:
                transform = get_transform(attr['transform'].value)
                points = apply_transform(points, transform)
            parents = get_parents_layernames(elem)
            rects.append((points, parents))
        return rects

    def get_circles(self):
        circles = []
        elements = self.dom.getElementsByTagNameNS(SVG_NS, 'circle')
        for elem in elements:
            x = float(elem.attributes['cx'].value)
            y = float(elem.attributes['cy'].value)
            r = float(elem.attributes['r'].value)
            circles.append(((x, y, r), get_parents_layernames(elem)))
        return circles

    # def get_path_collection(self):
    #     path_elems = self.root.findall('.//{http://www.w3.org/2000/svg}path')

    #     paths = [parse_path(elem.attrib['d']) for elem in path_elems]
    #     facecolors = [elem.attrib.get('fill', 'none') for elem in path_elems]
    #     edgecolors = [elem.attrib.get('stroke', 'none') for elem in path_elems]
    #     linewidths = [elem.attrib.get('stroke_width', 1) for elem in path_elems]
    #     collection = mpl.collections.PathCollection(paths,
    #                                           edgecolors=edgecolors,
    #                                           linewidths=linewidths,
    #                                           facecolors=facecolors)
    #     return collection


