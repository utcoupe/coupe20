from map_loader import LoadChecker
from bases import Vector3
import shapes


class Position(object):
    def __init__(self, xml):
        self.x = float(xml.attrib["x"])
        self.y = float(xml.attrib["y"])
        self.a = float(xml.attrib["a"]) if "a" in xml.attrib else None


class Marker(object):
    def __init__(self, xml, super_position=None, super_shape=None):
        self.type        = xml.find("type").text if xml.find("type") is not None else None
        self.ns          = xml.find("ns").text if xml.find("ns") is not None else None
        self.scale       = Vector3(None, None, None)
        self.position    = Vector3(None, None, None)
        self.orientation = Vector3(None, None, None)
        self.color       = None
        self._auto_complete(super_position, super_shape)

    def _auto_complete(self, position = None, shape = None):
        if position is not None:
            self.position.x = position.x
            self.position.y = position.y
            if position.a is not None:
                self.orientation.z = position.a
        if shape is not None:
            self.scale.x = shape.get_scale().x
            self.scale.y = shape.get_scale().y


class Feature(object):
    def __init__(self, xml):
        self.layer    = None
        self.position = None
        self.shape    = None
        self.marker   = None
        self.auto_complete(xml)

    def auto_complete(self, xml):
        self.layer = xml.attrib["layer"] if "layer" in xml.attrib else None

        pos = xml.find("position")
        self.position = Position(pos) if pos is not None else None

        shape = xml.find("shape")
        self.shape = shapes.CreateShape(shape) if shape is not None else None

        marker = xml.find("marker")
        self.marker = Marker(marker, self.position, self.shape) if marker is not None else None
