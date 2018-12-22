#!/usr/bin/python
import math
import map
from map_loader import LoadingHelpers


class Color():
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "name", "r", "g", "b")
        self.Name = xml.get("name")
        self.R = float(xml.get("r"))
        self.G = float(xml.get("g"))
        self.B = float(xml.get("b"))
        self.A = float(xml.get("a"))


class Position2D(object):
    def __init__(self, xml, validate=True):
        self.X = self.Y = self.A = 0.0
        self.HasAngle = False
        self.Frame = "map"
        if validate:
            LoadingHelpers.checkAttribExist(xml, "x", "y")
            self.X = float(xml.get("x"))
            self.Y = float(xml.get("y"))
            self.A = float(xml.get("a")) if "a" in xml.attrib else 0.0
            self.Frame = xml.get("frame_id") if "frame_id" in xml.attrib else "map"
            self.HasAngle = "a" in xml.attrib

    def transform(self, codes):
        if "x_mirror" in codes: # supposes the terrain is a rect
            self.X = map.MapDict.Terrain.Shape.Width - self.X
        if "a_mirror" in codes:
            self.A = math.pi - self.A
        return True


class Shape2D(object):
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "type")
        
        self.Type = xml.get("type")
        LoadingHelpers.checkValueValid(self.Type, "rect", "circle", "point")

        if self.Type == "rect":
            LoadingHelpers.checkAttribExist(xml, "w", "h")
            self.Width = float(xml.get("w"))
            self.Height = float(xml.get("h"))
        elif self.Type == "circle":
            LoadingHelpers.checkAttribExist(xml, "r")
            self.Radius = float(xml.get("r"))
        else:
            raise ValueError("ERROR: Shape type '{}' not supported.".format(self.Type))


class Marker(object):
    def __init__(self, xml, shape, color=None):
        LoadingHelpers.checkAttribExist(xml, "type", "ns", "z")
        LoadingHelpers.checkChildExist(xml, "scale", "orientation")

        self.Type      = xml.get("type")
        self.Namespace = xml.get("ns")
        self.Z         = float(xml.get("z"))

        LoadingHelpers.checkAttribExist(xml.find("scale"), "x", "y", "z")
        self.Scale = (xml.find("scale").get("x"), xml.find("scale").get("z"), xml.find("scale").get("z"))
        self.Color = color
