from map_loader import LoadChecker
from map_feature import Feature


class Layer(object):
    def __init__(self, xml):
        LoadChecker.checkAttribsExist(xml, "name")
        self.name = xml.attrib["name"]

        self.includes = []
        for include in xml.findall("include"):
            self.includes.append(include.attrib["name"])


class Color(object):
    def __init__(self, xml):
        LoadChecker.checkAttribsExist(xml, "name", "r", "g", "b")
        self.name = xml.attrib["name"]
        self.r = xml.attrib["r"]
        self.g = xml.attrib["g"]
        self.b = xml.attrib["b"]
        self.a = xml.attrib["a"] if "a" in xml.attrib else 1.0


class Config(object):
    CURRENT_TEAM = None
    DEFAULT_FRAME_ID = "map"

    LAYERS = []
    COLORS = []
    MARKERS = []

    @staticmethod
    def load(xml):
        LoadChecker.checkNodesExist(xml, "map", "layers", "colors")

        for layer in xml.find("layers").findall("layer"):
            Config.LAYERS.append(Layer(layer))

        for color in xml.find("colors").findall("color"):
            Config.COLORS.append(Color(color))
