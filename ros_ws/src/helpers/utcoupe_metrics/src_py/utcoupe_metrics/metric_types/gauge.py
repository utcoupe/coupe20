#!/usr/bin/env python

__author__ = "Gaëtan Blond"
__date__ = "20/03/20"

import rospy
from std_msgs.msg import Float64

class Gauge(object):
    def __init__(self, topic_name, start_value = 0.0):
        self.m_publisher = rospy.Publisher(topic_name, Float64, queue_size=10)
        self.m_value = start_value

    def inc(self, val_inc = 1.0):
        self.m_value += val_inc
    
    def dec(self, val_dec = 1.0):
        self.inc(-val_dec)

    def publish(self):
        self.m_publisher.publish(self.m_value)
    
    def set(self, val):
        self.m_value = val
    
    def set_to_current_time(self):
        self.m_value = rospy.get_rostime().to_nsec()
