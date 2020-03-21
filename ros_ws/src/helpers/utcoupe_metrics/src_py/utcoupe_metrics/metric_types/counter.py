#!/usr/bin/env python
# -*-coding:Utf-8 -*

__author__ = "Gaëtan Blond"
__date__ = "20/03/20"

import rospy

from std_msgs.msg import Float64

class Counter(object):
    def __init__(self, topic_name, start_value = 0.0):
        self.m_publisher = rospy.Publisher(topic_name, Float64, queue_size=10)
        self.m_value = start_value

    def inc(self, val_inc = 1.0):
        if val_inc < 0.0:
            raise Exception("counter increment value must be positive")

        self.m_value += val_inc

    def publish_value(self):
        self.m_publisher.publish(self.m_value)
