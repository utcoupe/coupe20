from map_feature import Feature
from map_loader import LoadChecker


class Waypoint(object):
    def __init__(self, xml):
        self.feature = Feature(xml)


class Zone(object):
    def __init__(self, xml):
        self.feature = Feature(xml)


class Object(object):
    def __init__(self, xml, classes):
        self.name = xml.attrib["name"] if "name" in xml.attrib else xml.attrib["class"]
        self.features = []
        self.waypoints = []

        # Pre-loading class info if available
        if "class" in xml.attrib:
            base = [c for c in classes if c.name == xml.attrib["class"]]
            if len(base) == 1:
                self.features = base[0].features
                self.waypoints = base[0].waypoints

        # Loading features list
        if xml.find("features") is not None:
            for f in xml.find("features").findall("feature"):
                self.features.append(Feature(f))

        # Loading waypoints list
        if xml.find("waypoints") is not None:
            for w in xml.find("waypoints").findall("waypoint"):
                self.waypoints.append(Waypoint(w))


class Container(Object):
    def __init__(self, xml, classes):
        super(Container, self).__init__(xml, classes)

        # Loading elements list
        LoadChecker.checkNodesExist(xml, "elements")
        LoadChecker.checkAttribsExist(xml.find("elements"), "min", "max")
        self.min = xml.find("elements").attrib["min"]
        self.max = xml.find("elements").attrib["max"]

        self.elements = []
        for o in xml.find("elements").findall("object"):
            self.elements.append(Object(o, classes))

        for c in xml.find("elements").findall("container"):
            self.elements.append(Container(c, classes))
