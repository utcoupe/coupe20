#!/usr/bin/python
import math
import map
from map_loader import LoadingHelpers
from map_bases import DictManager


class Color(DictManager):
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "name", "r", "g", "b")
        self.Name = xml.get("name")
        self.R = float(xml.get("r"))
        self.G = float(xml.get("g"))
        self.B = float(xml.get("b"))
        self.A = float(xml.get("a"))

class Position2D():
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "x", "y")
        self.X = float(xml.get("x"))
        self.Y = float(xml.get("y"))
        self.A = float(xml.get("a")) if "a" in xml.attrib else 0.0
        self.Frame = xml.get("frame_id") if "frame_id" in xml.attrib else "map"

    def transform(self, codes):
        if "x_mirror" in codes: # supposes the terrain is a rect
            self.X = map.MapDict.Terrain.Shape.Width - self.X
        if "a_mirror" in codes:
            self.A = math.pi - self.A
        return True

class Shape2D():
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "type")
        
        self.Type = xml.get("type")
        LoadingHelpers.checkValueValid(self.Type, "rect", "circle", "point")

        if self.Type == "rect":
            LoadingHelpers.checkAttribExist(xml, "w", "h")
            self.Width = xml.get("w")
            self.Height = xml.get("h")
        elif self.Type == "circle":
            LoadingHelpers.checkAttribExist(xml, "r")
            self.Radius = xml.get("r")


class MarkerRViz(DictManager):
    def __init__(self, xml, shape = None, color = None):
        LoadingHelpers.checkKeysExist(xml, "ns", "orientation")

        # Autofill based on other info
        if shape is not None:
            LoadingHelpers.checkKeysExist(xml, "z_scale")
            if shape.Dict["type"] == "circle":
                LoadingHelpers.checkKeysExist(xml, "type")
                xml["scale"] = (float(shape.Dict["radius"]) * 2.0, float(shape.Dict["radius"]) * 2.0, xml["z_scale"])
            elif shape.Dict["type"] == "rect":
                xml["type"] = "cube"
                xml["scale"] = (float(shape.Dict["width"]), float(shape.Dict["height"]), xml["z_scale"])
            else:
                raise KeyError("Marker could not be autofilled with shape '{}', not implemented.".format(shape.Dict["type"]))
        else:
            LoadingHelpers.checkKeysExist(xml, "scale")

        if color is not None:
            xml["color"] = color
        else:
            LoadingHelpers.checkKeysExist(xml, "color")
            xml["color"] = [c for c in map.Map.Colors if c.Dict["name"] == xml["color"]][0]

        super(MarkerRViz, self).__init__(xml)

    def merge(self, other):
        pass