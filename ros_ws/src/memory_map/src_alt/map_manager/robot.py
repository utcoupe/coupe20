class Robot(object):
    def __init__(self, xml):
        self.name = xml.attrib["name"]
        self.active = False # set to True externally if this robot is the current one in the system
