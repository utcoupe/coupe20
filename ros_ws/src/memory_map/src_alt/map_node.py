#!/usr/bin/python
import rospy
import map_manager
#import map_communication
#from markers import MarkersPublisher
#from occupancy import OccupancyGenerator


class MapNode():
    def __init__(self):
        rospy.init_node("map", log_level=rospy.INFO)

        map_manager.Map.load("gr", "green")
        print map_manager.Map.OBJECTS[0].elements[0].features


        # Starting and publishing the table STL to RViz
        #self.markers = MarkersPublisher()

        # Generate static occupancy images for pathfinder, etc.
        #occupancy = OccupancyGenerator(map_manager.Map)

        # Starting service handlers (Get, Set, Transfer, GetOccupancy)
        #map_communication.MapServices(occupancy)
        print "loaded map"

        #self.run()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if rospy.has_param("/current_team"):
                map_manager.Map.CONFIG.CURRENT_TEAM = rospy.get_param("/current_team")
            if rospy.has_param("/robot"):
                map_manager.Map.CONFIG.CURRENT_ROBOT = rospy.get_param("/robot")
            #self.markers.updateMarkers(map_manager.Map)
            r.sleep()


if __name__ == "__main__":
    MapNode()
