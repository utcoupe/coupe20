#!/usr/bin/python
import math
import copy
import map
from map_loader import LoadingHelpers


class Color():
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "name")
        base = [c for c in map.MapManager.Colors if c.Name == xml.get("name")]
        if base:
            color = copy.deepcopy(base[0])
            self.Name = color.Name
            self.R = color.R
            self.G = color.G
            self.B = color.B
            self.A = color.A
        else: # create a new color
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
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "type", "ns", "z")
        LoadingHelpers.checkChildExist(xml, "scale", "orientation")

        self.Type      = xml.get("type")
        LoadingHelpers.checkValueValid(self.Type, "cube", "cylinder", "sphere", "arrow", "mesh")
        self.Namespace = xml.get("ns")
        self.Z         = float(xml.get("z"))

        LoadingHelpers.checkAttribExist(xml.find("scale"), "x", "y", "z")
        self.Scale = [float(xml.find("scale").get("x")), float(xml.find("scale").get("y")), float(xml.find("scale").get("z"))]
        self.Orientation = [float(xml.find("orientation").get("x")), float(xml.find("orientation").get("z")), float(xml.find("orientation").get("z"))]

        # Type-specific attributes 
        self.MeshPath = xml.find("mesh_path").text if xml.find("mesh_path") is not None else ""
    
    def merge(self, xml_other): # other is prioritary
        self.Type        = xml_other.attrib["type"]      if "type" in xml_other.attrib else self.Type
        self.Namespace   = xml_other.attrib["ns"]        if "ns"   in xml_other.attrib else self.Namespace
        self.Z           = float(xml_other.attrib["z"])  if "z"    in xml_other.attrib else self.Z

        if xml_other.find("scale") is not None:
            LoadingHelpers.checkAttribExist(xml_other.find("scale"), "x", "y", "z")
            other_scale = [float(xml_other.find("scale").get("x")), float(xml_other.find("scale").get("y")), float(xml_other.find("scale").get("z"))]
            self.Scale = other_scale
        
        if xml_other.find("orientation") is not None:
            LoadingHelpers.checkAttribExist(xml_other.find("orientation"), "x", "y", "z")
            other_ori = [float(xml_other.find("orientation").get("x")), float(xml_other.find("orientation").get("y")), float(xml_other.find("orientation").get("z"))]
            self.Orientation = other_ori
        
        self.MeshPath    = xml_other.attrib["mesh_path"]  if "mesh_path" in xml_other.attrib else self.MeshPath
