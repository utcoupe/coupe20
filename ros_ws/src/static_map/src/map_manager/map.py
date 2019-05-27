#!/usr/bin/python
import time
import rospy
from map_loader import MapLoader, LoadingHelpers
from map_attributes import Color
from map_classes import Robot, Terrain, Waypoint, Container, Object, Class
from map_teams import Team

class MapDict():
    Dirty = True # Whether a service changed the dict since the last RViz Marker publish.

    Terrain   = None # future Terrain instance
    Objects   = None # future Container instance
    Waypoints = []   # List of Waypoint instances
    Robot     = None # future Container instance

class MapManager():
    # Internal global variables
    Colors = []
    Teams = []
    CurrentTeam = ''

    @staticmethod
    def load():
        # Loading XML files
        starttime = time.time() * 1000
        xml_config    = MapLoader.loadFile("1_config.xml")
        xml_terrain   = MapLoader.loadFile("2_terrain.xml")
        xml_waypoints = MapLoader.loadFile("3_waypoints.xml")
        xml_objects   = MapLoader.loadFile("4_objects.xml")
        xml_robot     = MapLoader.loadFile("5_robot.xml")
        rospy.loginfo("Loaded files in {0:.2f}ms.".format(time.time() * 1000 - starttime))

        # Setting current team to the default set one.
        for xml_team in xml_config.find("teams").findall("team"):
            if "default" in xml_team.attrib and bool(xml_team.attrib["default"]):
                if MapManager.CurrentTeam != "":
                    raise ValueError("ERROR Two or more teams have been set to default.")
                MapManager.CurrentTeam = xml_team.attrib["name"]
            MapManager.Teams.append(Team(xml_team))
        if MapManager.CurrentTeam == "":
            raise ValueError("ERROR One team has to be set as default.")

        # Loading the color palette
        for xml_color in xml_config.find("colors").findall("color"):
            MapManager.Colors.append(Color(xml_color))

        # Constructing object classes
        if not xml_objects.findall("classes"):
            raise KeyError("Objects file must have a 'classes' tag.")
        obj_classes = []
        for c in xml_objects.find("classes").findall("class"):
            obj_classes.append(Class(c, obj_classes))

        # Instantiate objects and create the map dict
        MapDict.Terrain   = Terrain(xml_terrain)
        xml_objects.attrib["name"] = "map"
        MapDict.Objects   = Container(xml_objects, obj_classes)
        MapDict.Waypoints = [Waypoint(w) for w in xml_waypoints.findall("waypoint")]
        MapDict.Robot     = Robot(xml_robot, obj_classes)

        rospy.loginfo("Loaded map in {0:.2f}ms.".format(time.time() * 1000 - starttime))

    @staticmethod
    def swap_team(team_name):
        if team_name != MapManager.CurrentTeam:
            for team in MapManager.Teams:
                if team.name == team_name:
                    team.swap()
                    return
            rospy.logerr("Found no team with name '{}', aborting team swap.".format(team_name))

    @staticmethod
    def is_dirty():
        dirty = MapManager.Dirty
        MapManager.Dirty = False
        return dirty
    
    @staticmethod
    def get_waypoint(name, position):
        if name is not None and name is not '':
            for w in MapDict.Waypoints:
                if w.Name == name:
                    return w
        elif position is not None:
            for w in MapDict.Waypoints:
                if w.Position.X == position.X and w.Position.Y == position.Y:
                    if position.HasAngle is True:
                        if w.Position.A == position.A:
                            return w
                    else:
                        return w
        else:
            rospy.logerr("    GET Request failed : incomplete waypoint must have a name or position already.")
        return None
    
    @staticmethod
    def get_container(path):
        if path[0] == "map":
            return MapDict.Objects.get_container(path[1:])
        elif path[0] == "robot":
            return MapDict.Robot.get_container(path[1:])
        else:
            rospy.logerr("   GET Request failed: first path element must be 'map' or 'robot'.")

    @staticmethod
    def get_context():
        return MapDict.Terrain, MapDict.Robot.Shape

    @staticmethod
    def add_object(path, obj):
        if path[0] == "map":
            return MapDict.Objects.add_object(path, obj)
        elif path[0] == "robot":
            return MapDict.Robot.add_object(path, obj)
        else:
            rospy.logerr("   GET Request failed: first path element must be 'map' or 'robot'.")
    
    @staticmethod
    def remove_object(path):
        if path[0] == "map":
            return MapDict.Objects.remove_object(path)
        elif path[0] == "robot":
            return MapDict.Robot.remove_object(path)
        else:
            rospy.logerr("   GET Request failed: first path element must be 'map' or 'robot'.")
    
    @staticmethod
    def merge_object(path, obj):
        pass
        

    @staticmethod
    def transform(codes):
        if not MapDict.Objects.transform(codes):
            return False
        for w in MapDict.Waypoints:
            if not w.transform(codes):
                return False
        return True
        
