#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy
from navigation_collisions.msg import PredictedCollision
from navigation_collisions.srv import ActivateCollisions

__author__ = "Gaëtan Blond"
__date__ = 11/12/2017

COLLISIONS_WATCHDOG_TIME = 0.3


class CollisionsClient(object):
    def __init__ (self, callbackStop, callbackResume):
        self.WARNER_TOPIC = "/navigation/collisions/warner"
        self.ACTIVATE_COLLISIONS_SERVICE_NAME = "/navigation/collisions/set_active"
        self._callbackStop = callbackStop
        self._callbackResume = callbackResume
        self._tmr_collisions_check = rospy.Timer(rospy.Duration(COLLISIONS_WATCHDOG_TIME), self._callback_timer_collisions_watchdog)
        self._last_collision = False
        self._collision_active = False
        self._activateCollisionsSrv = ""
        self._connectToServers()

    def _warnerCallback (self, message):
        if (message.danger_level <= PredictedCollision.LEVEL_DANGER):
            self._last_collision = True
            if not self._collision_active:
                self._collision_active = True
                rospy.loginfo("Obstacle detected, stopping the robot")
            self._callbackStop()

    def _connectToServers (self):
        rospy.loginfo("Waiting for \"" + self.ACTIVATE_COLLISIONS_SERVICE_NAME + "\"")
        rospy.wait_for_service(self.ACTIVATE_COLLISIONS_SERVICE_NAME)
        rospy.loginfo("Collisions found")

        try:
            rospy.Subscriber(self.WARNER_TOPIC, PredictedCollision, self._warnerCallback, queue_size=1)
            self._activateCollisionsSrv = rospy.ServiceProxy(self.ACTIVATE_COLLISIONS_SERVICE_NAME, ActivateCollisions)
        except rospy.ServiceException, e:
            str_error = "Error when trying to connect to "
            str_error += self.ACTIVATE_COLLISIONS_SERVICE_NAME
            str_error += " : " + str(e)
            rospy.logerr(str_error)

    def _callback_timer_collisions_watchdog(self, event):
        if self._collision_active:
            if not self._last_collision:
                self._callbackResume()
                self._collision_active = False
            self._last_collision = False
    
    def setEnabled(self, isEnabled):
        self._activateCollisionsSrv.call(active=isEnabled)
