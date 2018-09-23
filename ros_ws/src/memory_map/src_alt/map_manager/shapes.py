from map_loader import LoadChecker
from bases import Vector3

class Shape(object):
    TYPE = ''

    def __init__(self, xml):
        self.type = xml.attrib["type"]

    def get_scale(self):
        raise NotImplementedError("Method must be overritten by subclass.")


def CreateShape(xml):
    if "type" not in xml.attrib:
        raise KeyError("Shape definitions need a type.")

    for c in Shape.__subclasses__():
        if c.TYPE == xml.attrib["type"]:
            return c(xml)

    raise ValueError("No shape class defined for type '{}'.".format(xml.attrib["type"]))


class Rect(Shape):
    TYPE = 'rect'

    def __init__(self, xml):
        super(Rect, self).__init__(xml)
        LoadChecker.checkAttribsExist(xml, "w", "h")
        self.w = float(xml.attrib["w"])
        self.h = float(xml.attrib["h"])

    def get_scale(self):
        return Vector3(self.w, self.h, None)


class Circle(Shape):
    TYPE = 'circle'

    def __init__(self, xml):
        super(Circle, self).__init__(xml)
        LoadChecker.checkAttribsExist(xml, "r")
        self.r = float(xml.attrib["r"])

    def get_scale(self):
        return Vector3(self.r * 2.0, self.r * 2.0, None)
