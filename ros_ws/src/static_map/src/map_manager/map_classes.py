#!/usr/bin/python
import copy, json
import rospy
import xml.etree.ElementTree as ET
from map_loader import LoadingHelpers
from map_attributes import Position2D, Shape2D, Color, Marker
import map


class Robot(object):
    def __init__(self, xml, obj_classes):
        LoadingHelpers.checkChildExist(xml, "shape", "containers", "color")
        self.Shape = Shape2D(xml.find("shape"))
        xml.find("containers").attrib["name"] = "robot"
        self.Container = Container(xml.find("containers"), obj_classes)
        self.Color = Color(xml.find("color"))

        #TODO temporary, considering robot as rect. real xml marker (e.g. for meshes) ?
        m = ET.Element("marker")
        m.attrib["ns"] = "robot"
        m.attrib["type"]  = "cube"
        m.attrib["z"] = 0.35 / 2
        scale = ET.SubElement(m, "scale")
        scale.attrib["x"] = self.Shape.Height
        scale.attrib["y"] = self.Shape.Width
        scale.attrib["z"] = 0.35
        orientation = ET.SubElement(m, "orientation")
        orientation.attrib["x"] = 0.0
        orientation.attrib["y"] = 0.0
        orientation.attrib["z"] = 0.0
        self.Marker = Marker(m)

    def get_container(self, path):
        return self.Container.get_container(path)

    def add_object(self, path, obj):
        return self.Container.add_object(path, obj)
    
    def remove_object(self, path):
        return self.Container.remove_object(path)



class Terrain(object):
    def __init__(self, xml):
        LoadingHelpers.checkAttribExist(xml, "type")
        LoadingHelpers.checkChildExist(xml, "position", "marker", "color")
        self.Name = "terrain"
        self.Position = Position2D(xml.find("position"))
        self.Shape = Shape2D(xml)
        self.Marker = Marker(xml.find("marker"))
        self.Color = Color(xml.find("color"))
        
        self.Layers = [Layer(l) for l in xml.findall("layer")]

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
        LoadingHelpers.checkAttribExist(xml, "name")
        LoadingHelpers.checkChildExist(xml, "position", "shape")
        self.Name     = xml.get("name")
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
    
    def transform(self, codes):
        return self.Position.transform(codes)


class Container(object):
    def __init__(self, xml, xml_classes):
        LoadingHelpers.checkAttribExist(xml, "name")
        self.Name = xml.get("name")
        self.Elements =  [Container(c, xml_classes) for c in xml.findall("container")]
        self.Elements += [Object(o, xml_classes)    for o in xml.findall("object")]

    def get_container(self, path):
        if not path:
            return self
        elif self.Name == path[0]:
            return self
        else:
            for e in self.Elements:
                if isinstance(e, Container):
                    if e.Name == path[0]:
                        return e.get_container(path[1:])
        rospy.logerr("    GET Request failed : couldn't find any container named '{}'.".format(path[0]))
        return None
    
    def add_object(self, path, obj):
        if path[0] == self.Name:
            if len([e for e in self.Elements if (isinstance(e, Object) and e.Name == obj.Name)]):
                rospy.logerr("    ADD Request failed : an object with name '{}' already exists in container '{}'.".format(obj.Name, self.Name))
                return False
            self.Elements.append(obj)
            return True
        elif len(path) == 1:
            rospy.logerr("    ADD Request failed : couldn't find any container named '{}'.".format(path[0]))
            return False
        else:
            for e in self.Elements:
                if isinstance(e, Container) and e.Name == path[1]:
                    return e.add_object(path[1:], obj)
        return False
    
    def remove_object(self, path):
        print "remove object, path = " + str(path)
        if len(path) > 2:
            for e in self.Elements:
                if isinstance(e, Container) and e.Name == path[1]:
                    return e.remove_object(path[1:])
        elif len(path) == 2:
            if not self.Name == path[0]:
                rospy.logerr("    RMV Request failed : ended up in wrong path.")
                return None
            for e in self.Elements:
                if isinstance(e, Object) and e.Name == path[1]:
                    res_obj = e
                    self.Elements.remove(e)
                    return res_obj
            rospy.logerr("    RMV Request failed : no object with name '{}' found in container '{}'.".format(path[1], self.Name))
        return None

    def transform(self, codes):
        for e in self.Elements:
            if not e.transform(codes):
                return False
        return True


class Object(object):
    def __init__(self, xml, obj_classes, check_valid = True):
        LoadingHelpers.checkAttribExist(xml, "name")
        self.Name = xml.get("name")
        self.Position = Position2D(xml.find("position")) if xml.find("position") is not None else None
        self.Shape    = Shape2D(xml.find("shape"))       if xml.find("shape")    is not None else None
        self.Labels   = [l.get("name") for l in xml.find("labels").findall("label")] if xml.find("labels") else []

        self.Color = Color(xml.find("color")) if xml.find("color") is not None else None

        if xml.get("class"): # merge info with class
            other = copy.deepcopy([c for c in obj_classes if c.Name == xml.get("class")][0])
            # self is prioritary
            self.Position = self.Position if self.Position is not None else other.Position
            self.Shape    = self.Shape    if self.Shape    is not None else other.Shape
            self.Color    = self.Color    if self.Color    is not None else other.Color
            self.Labels += other.Labels
            
            self.Marker = copy.deepcopy(other.Marker)
            if xml.find("marker") is not None:
                self.Marker.merge(xml.find("marker"))
        else:
            self.Marker = Marker(xml.find("marker")) if xml.find("marker") is not None else None
        
        if check_valid is True:
            self.check_valid() # disabled when manually creating a class through code

    def check_valid(self): # Checks if all values are not None (in case of class merges)
        if None in [self.Position, self.Shape]:
            raise ValueError("ERROR : Even after merge object '{}' still has no position or shape.".format(self.Name))
        if self.Marker is not None and self.Color is None:
            raise ValueError("ERROR : Even after merge object '{}' still has a marker but no color.".format(self.Name))
    
    def transform(self, codes):
        return self.Position.transform(codes)

class Class(Object):
    def __init__(self, xml, obj_classes):
        LoadingHelpers.checkAttribExist(xml, "name")
        self.Name = xml.get("name")
        super(Class, self).__init__(xml, obj_classes, check_valid = False)
