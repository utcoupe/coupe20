#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy

from memory_map.srv import FillWaypoint
from memory_map.msg import Waypoint


__author__ = "Gaëtan Blond"
__date__ = 1/2/2018

class MapClient(object):
    def __init__ (self):
        self._fillWaypointSrv = ""

        self.FILLWAYPOINTS_SERVICE_NAME = "/memory/map/fill_waypoint"

        self._connectToServer()

    def getPosFromWaypoint(self, waypointName):
        waypoint = Waypoint(name = waypointName, has_angle = True)
        return self._fillWaypointSrv.call(waypoint = waypoint).filled_waypoint.pose
    
    def _connectToServer(self):
        rospy.loginfo("Waiting for \"" + self.FILLWAYPOINTS_SERVICE_NAME + "\"")
        rospy.wait_for_service(self.FILLWAYPOINTS_SERVICE_NAME)
        rospy.loginfo("Map found")

        try:
            self._fillWaypointSrv = rospy.ServiceProxy(self.FILLWAYPOINTS_SERVICE_NAME, FillWaypoint)
        except rospy.ServiceException, e:
            str_error = "Error when trying to connect to "
            str_error += self.FILLWAYPOINTS_SERVICE_NAME
            str_error += " : " + str(e)
            rospy.logerr(str_error)
