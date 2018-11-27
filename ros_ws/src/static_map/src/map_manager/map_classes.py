#!/usr/bin/python
import copy, json
import rospy
from map_loader import LoadingHelpers
from map_attributes import Position2D, Shape2D, Color, MarkerRViz
import map


class Terrain(object):
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "type")
        self.Shape = Shape2D(xml)
        self.Layers = [Layer(l) for l in xml.findall("layer")]
        # TODO marker

        # Copy walls from other layers (includes)
        for layer in self.Layers:
            for include in layer.Includes:
                for l in self.Layers:
                    if l.Name == include:
                        for w in l.Walls:
                            layer.Walls.append(copy.deepcopy(w))
        

class Layer(object):
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "name")
        self.Name = xml.get("name")
        self.Includes = [i.get("name") for i in xml.findall("include")]
        self.Walls = [Wall(w) for w in xml.findall("wall")]


class Wall(object):
    def __init__(self, xml):
        LoadingHelpers.checkChildExist(xml, "position", "shape")
        self.Position = Position2D(xml.find("position"))
        self.Shape    = Shape2D(xml.find("shape"))


class Waypoint(object):
    def __init__(self, xml, validate = True):
        self.Name = ""
        self.Position = Position2D(None, validate = False)
        if validate:
            LoadingHelpers.checkAttribExist(xml, "name")
            self.Name = xml.get("name")
            self.Position = Position2D(xml, validate)


class Container(object):
    def __init__(self, xml, xml_classes):
        LoadingHelpers.checkAttribExist(xml, "name")
        self.Name = xml.get("name")
        self.Elements =  [Container(c, xml_classes) for c in xml.findall("container")]
        self.Elements += [Object(o, xml_classes)    for o in xml.findall("object")]

    def get_container(self, nameslist):
        if not nameslist:
            return self
        elif self.Name == nameslist[0]:
            return self
        else:
            for e in self.Elements:
                if isinstance(e, Container):
                    if e.Name == nameslist[0]:
                        return e.get_container(nameslist[1:])
        rospy.logerr("    GET Request failed : couldn't find any container named '{}'.".format(nameslist[0]))
        return None


class Object(object):
    def __init__(self, xml, obj_classes, check_valid = True):
        LoadingHelpers.checkAttribExist(xml, "name")
        self.Name = xml.get("name")
        self.Position = Position2D(xml.find("position")) if xml.find("position") is not None else None
        self.Shape    = Shape2D(xml.find("shape"))       if xml.find("shape")    is not None else None
        self.Labels   = [l.get("name") for l in xml.find("labels").findall("label")] if xml.find("labels") else []
        self.Marker   = None #TODO Markers Position2D(xml.find("position")) if xml.find("position") else None

        self.Color = None
        if xml.find("color") is not None:
            LoadingHelpers.checkAttribExist(xml.find("color"), "name")
            base = [c for c in map.MapManager.Colors if c.Name == xml.find("color").get("name")]
            if base:
                self.Color = copy.deepcopy(base[0])
            else:
                self.Color = Color(xml.find("color"))

        if xml.get("class"):
            self.merge(copy.deepcopy([c for c in obj_classes if c.Name == xml.get("class")][0]))
        
        if check_valid is True: # disabled when creating a class
            self.check_valid()

    def merge(self, other): #self is prioritary
        self.Position = self.Position if self.Position is not None else other.Position
        self.Shape    = self.Shape    if self.Shape    is not None else other.Shape
        self.Color    = self.Color    if self.Color    is not None else other.Color
        self.Labels += other.Labels
        
        #self.Marker.merge(other.Marker) #TODO Markers
    
    def check_valid(self): # Checks if all values are not None (in case of class merges)
        if None in [self.Position, self.Shape]:
            raise ValueError("ERROR : Even after merge an object still has no position or shape.")
    
    def transform(codes):
        pass #TODO

class Class(Object):
    def __init__(self, xml, obj_classes):
        LoadingHelpers.checkAttribExist(xml, "name")
        self.Name = xml.get("name")
        super(Class, self).__init__(xml, obj_classes, check_valid = False)
