#!/usr/bin/env python

__author__ = "Gaëtan Blond"
__date__ = "20/03/20"

import rospy

from gauge import Gauge

class Chronometer(object):
    def __init__(self, topic_name):
        self.m_gauge = Gauge(topic_name)
        self.m_startTime = None
    
    def start(self):
        self.m_startTime = rospy.get_rostime()
    
    def stop(self):
        now = rospy.get_rostime()
        if self.m_startTime is None:
            raise Exception("must call start before stop")
        dur = now - self.m_startTime
        self.m_gauge.set(dur.to_nsec())
        self.reset()
    
    def reset(self):
        self.m_startTime = None
