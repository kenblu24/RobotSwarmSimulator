import re
from itertools import batched
import xml.etree.ElementTree as etree

import numpy as np
from math import radians


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


class SVG:
    def __init__(self, svg_content: str):
        self.root = etree.fromstring(svg_content)

    # def get_paths(self):
    #     path_elems = self.root.findall('.//{http://www.w3.org/2000/svg}path')
    #     return [parse_path(elem.attrib['d']) for elem in path_elems]

    def get_polygons(self):
        polys = self.root.findall('.//{http://www.w3.org/2000/svg}polygon')
        poly_points = [elem.attrib['points'] for elem in polys]
        poly_points = [points.split() for points in poly_points]  # split space-separated numbers
        poly_points = [[float(point) for point in points] for points in poly_points]  # convert to floats
        coords = [list(batched(points, 2)) for points in poly_points]  # group into coordinate pairs
        return coords

    def get_rects(self):
        rects = []
        elements = self.root.findall('.//{http://www.w3.org/2000/svg}rect')
        for elem in elements:
            x = float(elem.attrib['x'])
            y = float(elem.attrib['y'])
            width = float(elem.attrib['width'])
            height = float(elem.attrib['height'])
            points = rect_from_wh(x, y, width, height)
            if 'transform' in elem.attrib:
                transform = get_transform(elem.attrib['transform'])
                points = apply_transform(points, transform)
            rects.append(points)
        return rects

    def get_circles(self):
        circles = []
        elements = self.root.findall('.//{http://www.w3.org/2000/svg}circle')
        for elem in elements:
            x = float(elem.attrib['cx'])
            y = float(elem.attrib['cy'])
            r = float(elem.attrib['r'])
            circles.append((x, y, r))
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


