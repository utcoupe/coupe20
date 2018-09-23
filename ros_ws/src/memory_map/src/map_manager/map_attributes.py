#!/usr/bin/python
import math
import map
from map_loader import LoadingHelpers
from map_bases import DictManager


class Color(DictManager):
    def __init__(self, name, initdict):
        if not (isinstance(initdict, list) and len(initdict) == 4):
            raise ValueError("ERROR Color must be a list with 4 elements in it (RBGA).")
        super(Color, self).__init__({
            "name": name,
            "r": float(initdict[0]),
            "g": float(initdict[1]),
            "b": float(initdict[2]),
            "a": float(initdict[3])
        })

class Position2D(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "frame_id", "x", "y", "type")
        super(Position2D, self).__init__(initdict)

    def transform(self, codes):
        if "x_mirror" in codes:
            self.Dict["x"] = map.Map.get("/terrain/shape/^").Dict["width"] - self.Dict["x"]
        if "a_mirror" in codes:
            if "a" in self.Dict.keys():
                self.Dict["a"] = math.pi - self.Dict["a"]
        return True

class Shape2D(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "type")
        # TODO : validate for each shape type
        super(Shape2D, self).__init__(initdict)


class MarkerRViz(DictManager):
    def __init__(self, initdict, shape = None, color = None):
        LoadingHelpers.checkKeysExist(initdict, "ns", "orientation")

        # Autofill based on other info
        if shape is not None:
            LoadingHelpers.checkKeysExist(initdict, "z_scale")
            if shape.Dict["type"] == "circle":
                LoadingHelpers.checkKeysExist(initdict, "type")
                initdict["scale"] = (float(shape.Dict["radius"]) * 2.0, float(shape.Dict["radius"]) * 2.0, initdict["z_scale"])
            elif shape.Dict["type"] == "rect":
                initdict["type"] = "cube"
                initdict["scale"] = (float(shape.Dict["width"]), float(shape.Dict["height"]), initdict["z_scale"])
            else:
                raise KeyError("Marker could not be autofilled with shape '{}', not implemented.".format(shape.Dict["type"]))
        else:
            LoadingHelpers.checkKeysExist(initdict, "scale")

        if color is not None:
            initdict["color"] = color
        else:
            LoadingHelpers.checkKeysExist(initdict, "color")
            initdict["color"] = [c for c in map.Map.Colors if c.Dict["name"] == initdict["color"]][0]

        super(MarkerRViz, self).__init__(initdict)


class Trajectory(): # TODO Inherit
    def __init__(self, initdict):
        pass
