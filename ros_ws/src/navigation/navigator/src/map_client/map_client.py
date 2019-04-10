#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy

from static_map.srv import MapGetWaypoint
from static_map.msg import Waypoint


__author__ = "Gaëtan Blond"
__date__ = 1/2/2018

class MapClient(object):
    def __init__ (self):
        self._fillWaypointSrv = ""

        self.FILLWAYPOINTS_SERVICE_NAME = "static_map/get_waypoint"

        self._connectToServer()

    def getPosFromWaypoint(self, waypointName):
        waypoint = Waypoint(name = waypointName, has_angle = True)
        response = self._fillWaypointSrv.call(waypoint = waypoint).filled_waypoint
        return response.pose, response.has_angle
    
    def _connectToServer(self):
        rospy.loginfo("Waiting for \"" + self.FILLWAYPOINTS_SERVICE_NAME + "\"")
        rospy.wait_for_service(self.FILLWAYPOINTS_SERVICE_NAME)
        rospy.loginfo("Map found")

        try:
            self._fillWaypointSrv = rospy.ServiceProxy(self.FILLWAYPOINTS_SERVICE_NAME, MapGetWaypoint)
        except rospy.ServiceException, e:
            str_error = "Error when trying to connect to "
            str_error += self.FILLWAYPOINTS_SERVICE_NAME
            str_error += " : " + str(e)
            rospy.logerr(str_error)
