from map_loader import MapLoader, LoadChecker
from map_config import Config
from robot import Robot
from map_feature import Feature
from map_classes import Object, Container, Zone

class Map(object):
    CONFIG = Config()
    MAP_FEATURE = None

    ROBOTS      = []
    WAYPOINTS   = []
    ZONES       = []
    OBJECTS     = []

    @staticmethod
    def load(robot_name, team_name):
        config_xml  = MapLoader.loadFile(MapLoader.CONFIG_FILE)
        robots_xml  = MapLoader.loadFile(MapLoader.ROBOTS_FILE)
        classes_xml = MapLoader.loadFile(MapLoader.CLASSES_FILE)
        objects_xml = MapLoader.loadFile(MapLoader.OBJECTS_FILE)

        Map.CONFIG.load(config_xml)
        LoadChecker.checkNodesExist(config_xml, "map")
        Map.MAP_FEATURE = Feature(config_xml.find("map").find("feature"))

        for robot in robots_xml.findall("robot"):
            r = Robot(robot)
            if r.name == robot_name: r.active = True
            Map.ROBOTS.append(r)

        Map._load_team_objects(classes_xml, objects_xml, team_name)

    @staticmethod
    def _load_team_objects(classes_xml, objects_xml, team_name):
        team = [team for team in objects_xml.findall("team") if team.attrib["name"] == team_name][0]
        for z in classes_xml.find("zones").findall("zone"):
            Map.ZONES.append(Zone(z))

        classes = []
        for c in classes_xml.find("objects").findall("object"):
            classes.append(Object(c, []))

        for o in team.findall("object"):
            Map.OBJECTS.append(Object(o, classes))

        for c in team.findall("container"):
            Map.OBJECTS.append(Container(c, classes))
