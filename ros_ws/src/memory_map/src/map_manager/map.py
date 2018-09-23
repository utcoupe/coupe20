#!/usr/bin/python
import time
import rospy
from map_loader import MapLoader, LoadingHelpers
from map_bases import DictManager, RequestPath
from map_attributes import Color
from map_classes import Terrain, Zone, Waypoint, Entity, Container, Object
from map_teams import Team

class Map():
    MAP_DICT = None
    Dirty = True # Whether a service changed the dict since the last RViz Marker publish.

    # Internal global variables
    Colors = []
    Teams = []
    CurrentTeam = ''

    @staticmethod
    def load():
        starttime = time.time() * 1000
        initdict_config    = MapLoader.loadFile("1_Config.yml")["config"]
        initdict_terrain   = MapLoader.loadFile("2_Terrain.yml")["terrain"]
        initdict_zones     = MapLoader.loadFile("3_Zones.yml")["zones"]
        initdict_waypoints = MapLoader.loadFile("4_Waypoints.yml")["waypoints"]
        initdict_entities  = MapLoader.loadFile("5_Entities.yml")["entities"]
        initdict_objects   = MapLoader.loadFile("6_Objects.yml")
        obj_classes = initdict_objects["classes"]
        rospy.loginfo("Loaded files in {0:.2f}ms.".format(time.time() * 1000 - starttime))

        # Setting current team to the default set one.
        for team in initdict_config["teams"]:
            if bool(initdict_config["teams"][team]["default"]) is True:
                if Map.CurrentTeam != '':
                    raise ValueError("ERROR Two or more teams have been set to default.")
                Map.CurrentTeam = team
            Map.Teams.append(Team(team, initdict_config["teams"][team]))

        # Loading the color palette
        for color in initdict_config["colors"]:
            Map.Colors.append(Color(color, initdict_config["colors"][color]))

        # Instantiate objects before creating the map dict
        for zone in initdict_zones:
            initdict_zones[zone] = Zone(initdict_zones[zone])
        for waypoint in initdict_waypoints:
            initdict_waypoints[waypoint] = Waypoint(initdict_waypoints[waypoint])
        for entity in initdict_entities:
            initdict_entities[entity] = Entity(initdict_entities[entity], obj_classes)

        # Create main Map dict
        Map.MAP_DICT = DictManager({
            "terrain":   Terrain(initdict_terrain),
            "zones":     DictManager(initdict_zones),
            "waypoints": DictManager(initdict_waypoints),
            "entities":  DictManager(initdict_entities),
            "objects":   Container(initdict_objects["objects"], obj_classes)
        })
        rospy.loginfo("Loaded map in {0:.2f}ms.".format(time.time() * 1000 - starttime))

    @staticmethod
    def swap_team(team_name):
        if team_name != Map.CurrentTeam:
            for team in Map.Teams:
                if team.name == team_name:
                    team.swap()
                    return
            rospy.logerr("Found no team with name '{}', aborting team swap.".format(team_name))

    @staticmethod
    def is_dirty():
        dirty = Map.Dirty
        Map.Dirty = False
        return dirty

    @staticmethod
    def get(requestpath):
        if requestpath[0] != "/":
            rospy.logerr("    GET Request failed : global search needs to start with '/'.")
            return None
        return Map.MAP_DICT.get(requestpath)

    @staticmethod
    def get_objects(collisions_only = False):
        return Map.MAP_DICT.Dict["objects"].get_objects(collisions_only)

    @staticmethod
    def set(requestpath, mode, instance = None):
        if requestpath[0] != "/":
            rospy.logerr("    SET Request failed : global search needs to start with '/'.")
            return None
        return Map.MAP_DICT.set(requestpath, mode, instance)

    @staticmethod
    def transform(codes):
        return Map.MAP_DICT.transform(codes)
