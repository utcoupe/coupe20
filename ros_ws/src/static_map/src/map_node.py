#!/usr/bin/python
import rospy
import map_manager
import map_communication
from markers import MarkersPublisher
from occupancy import OccupancyGenerator
from game_manager import StatusServices


class MapNode():
    def __init__(self):
        rospy.init_node("map", log_level=rospy.INFO)
        rospy.logdebug("Started static_map node.")

        map_manager.MapManager.load()

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()

        # Generate static occupancy images for pathfinder, etc.
        occupancy = OccupancyGenerator()

        # Starting service handlers (Get, Set, Transfer, GetOccupancy)
        map_communication.MapServices(occupancy)
        rospy.logdebug("[static_map] Map request servers ready.")

        # Tell ai/game_manager the node initialized successfuly.
        StatusServices("memory", "map").ready(True)

        self.run()

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if rospy.has_param("/current_team"):
                map_manager.MapManager.swap_team(rospy.get_param("/current_team"))
            self.markers.updateMarkers(map_manager.MapDict)
            r.sleep()


if __name__ == "__main__":
    MapNode()
