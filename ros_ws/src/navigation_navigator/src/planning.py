#!/usr/bin/env python
# -*-coding:Utf-8 -*

__author__ = "Gaëtan Blond"
__date__ = 14/4/2018

import math
from collections import OrderedDict

import rospy

from asserv import AsservClient
from pathfinder import PathfinderClient

class PlanStatuses:
    IDLE        = 0
    NAVIGATING  = 1
    CANCELLING  = 2

class Directions:
    BACKWARD    = 0
    FORWARD     = 1
    AUTOMATIC   = 2


def pointToStr(point):
    """
    Convert a geometry_msgs/Pose2D object (or any object with x an y properties) to a string
    """
    return "(" + str(point.x) + "," + str(point.y) + ")"

class Plan(object):
    def __init__ (self, asservClient, pathfinderClient, resultCallback, updateCallback):
        self._asservClient = asservClient
        self._pathfinderClient = pathfinderClient
        self._currentPath = OrderedDict()
        self._resultCallback = resultCallback
        self._updateCallback = updateCallback
        self._status = PlanStatuses.IDLE
        self._endPos = ""
        self._hasAngle = False
        self._direction = Directions.AUTOMATIC
    
    def newPlan(self, startPos, endPos, hasAngle, direction):
        self._endPos = endPos
        self._hasAngle = hasAngle
        self._direction = direction
        self.replan(startPos)
    
    def replan(self, startPos):
        if len(self._currentPath) > 0:
            self.cancelAsservGoals()
        self._currentPath = OrderedDict() # needed ?
        debugStr = "Asked to go from "
        debugStr += pointToStr(startPos)
        debugStr += " to " + pointToStr(self._endPos)
        rospy.logdebug(debugStr)
        try:
            # sends a request to the pathfinder
            path = self._pathfinderClient.FindPath(startPos, self._endPos)
            self._printPath (path)
            # then sends the path point per point to the arduino_asserv
            path.pop(0) # Removes the first point (we are already on startPos)
            path.pop() # Removes the last point
            lastPoint = startPos
            for point in path:
                idOrder = self._asservClient.doGoto(point, self._getDirection(self._direction, point, lastPoint), False, self._asservGotoCallback)
                self._currentPath[idOrder] = point
                lastPoint = point
            idOrder = self._asservClient.doGoto(self._endPos, self._getDirection(self._direction, self._endPos, lastPoint), self._hasAngle, self._asservGotoCallback)
            self._currentPath[idOrder] = self._endPos
            self._status = PlanStatuses.NAVIGATING
            rospy.logdebug("Our path has " + str(len(self._currentPath)) + " points:")
            for key in self._currentPath.keys():
                rospy.logdebug(key)
            self._updateCallback() # needed ?
        except Exception as e:
            rospy.logerr("Navigation failed: " + e.message)
            if len(self._currentPath) > 0:
                self.cancelAsservGoals()
            else:
                self._status = PlanStatuses.IDLE
                self._resultCallback(False)

    def _asservGotoCallback(self, idOrder, result):
        rospy.logdebug("Callback for " + idOrder)
        rospy.logdebug("Path size : " + str(len(self._currentPath)))
        if idOrder in self._currentPath.keys():
            del self._currentPath[idOrder]
            if len(self._currentPath) == 0:
                if self._status == PlanStatuses.CANCELLING:
                    result = False
                self._status = PlanStatuses.IDLE
                self._resultCallback(result)
        else:
            rospy.logwarn("Trying to delete an unknown order...")
        self._updateCallback()


    def cancelAsservGoals(self):
        self._status = PlanStatuses.CANCELLING
        #for idGoal in self._currentPath.keys():
        #    self._asservClient.cancelGoal(idGoal)
        self._asservClient.cancelAllGoals()
    
    def getCurrentPath(self):
        path = []
        for idOrder in self._currentPath.keys():
            rospy.logdebug("order " + idOrder)
            path.append(self._currentPath[idOrder])
        return path

    def _getAngle(self, v1, v2):
        prodScal = v1.x*v2.x + v1.y*v2.y
        normeV1 = math.sqrt(pow(v1.x, 2) + pow(v1.y, 2))
        normeV2 = math.sqrt(pow(v2.x, 2) + pow(v2.y, 2))
        cosV1V2 = prodScal / (normeV1 * normeV2)

        # fix float precision
        if cosV1V2 > 1.0:
            cosV1V2 = 1.0
        elif cosV1V2 < -1.0:
            cosV1V2 = -1.0

        return math.acos(cosV1V2)
    
    def _getDirection(self, askedDirection, newPos, lastPos):
        # TODO debug (last angle / current angle ?)
        if askedDirection != Directions.AUTOMATIC:
            return askedDirection
        if abs(lastPos.theta - self._getAngle(lastPos, newPos)) > (math.pi / 2):
            return Directions.BACKWARD
        else:
            return Directions.FORWARD
    
    def _printPath (self, path):
        """
        Print the path in the debug log from ROS.
        @param path:    An array of Pose2D
        """
        debugStr = "Received path: ["
        for point in path:
            debugStr += pointToStr(point) + ","
        debugStr += "]"
        rospy.logdebug (debugStr)