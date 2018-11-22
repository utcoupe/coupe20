#!/usr/bin/env python
# -*-coding:Utf-8 -*

from threading import Thread

import rospy
import tf2_ros

from geometry_msgs.msg import Pose2D

class LocalizerClient(Thread):
    def __init__(self):
        self._lastKnownPos = Pose2D()
        self._tfBuffer = tf2_ros.Buffer()
        self._posRobotListener = tf2_ros.TransformListener(self._tfBuffer)

        # Creates the thread
        Thread.__init__(self)
        self.start()

    def run(self):
        rate = rospy.Rate(10.0)
        rospy.logdebug("Start run")
        while not rospy.is_shutdown():
            try:
                tmpPos = self._tfBuffer.lookup_transform("map", "robot", rospy.Time()).transform.translation
                newPose = Pose2D(x=tmpPos.x, y=tmpPos.y)
                if newPose != self._lastKnownPos:
                    self._lastKnownPos = newPose
                    # rospy.logdebug("tf2 frame : " + str(self._lastKnownPos.x) + ", " + str(self._lastKnownPos.y))
                rate.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
    
    def getLastKnownPos(self):
        return self._lastKnownPos