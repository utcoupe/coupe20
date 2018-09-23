#!/usr/bin/env python

import subprocess
import rospy
from drivers_ard_others.msg import *
from drivers_port_finder.srv import *
from ai_game_manager import StatusServices
from sensor_msgs.msg import Range
from numpy import inf

__author__ = "Thomas Fuhrmann"
__date__ = 06/01/2018

NODE_NAME = "teraranger"
PUBLISH_INTERVAL = 0.02  # in ms
GET_PORT_SERVICE_NAME = "/drivers/port_finder/get_port"
GET_PORT_SERVICE_TIMEOUT = 15  # in seconds
WATCHDOG_PERIOD = 0.1  # in seconds


class Teraranger:
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        teraranger_port = ""
        self._node_subprocess = None
        self._connected = False
        try:
            rospy.wait_for_service(GET_PORT_SERVICE_NAME, GET_PORT_SERVICE_TIMEOUT)
            self._src_client_get_port = rospy.ServiceProxy(GET_PORT_SERVICE_NAME, GetPort)
            teraranger_port = self._src_client_get_port("teraranger").port
        except rospy.ROSException:
            rospy.logerr("Port_finder service does not exist, can't fetch the teraranger port...")
        
        status = StatusServices("drivers", "teraranger")
        if teraranger_port != "":
            self._node_subprocess = subprocess.Popen(["rosrun", "teraranger", "one", "_portname:=" + teraranger_port, "__ns:=/"])
            self._watchdog = rospy.Timer(rospy.Duration(WATCHDOG_PERIOD), self._check_subprocess)
            self._connected = True
            status.ready(True)
        else:
            rospy.logerr("Teraranger port has not been found, start the node but can't send real data...")
            status.ready(False)
        self._range_value = 0.0
        self._pub_belt_range = rospy.Publisher("/drivers/ard_others/belt_ranges", BeltRange, queue_size=1)
        self._sub_terarange = rospy.Subscriber("/teraranger_one", Range, self._callback_teranranger_range)
        while not rospy.is_shutdown():
            if self._connected:
                self._pub_belt_range.publish(BeltRange("sensor_tera1", self._range_value))
            rospy.sleep(PUBLISH_INTERVAL)
        if self._node_subprocess:
            self._node_subprocess.terminate()

    def _callback_teranranger_range(self, data):
        if data.range == -inf:
            data.range = -1
        self._range_value = float(data.range)

    def _check_subprocess(self, event):
        if self._node_subprocess.poll() is not None:
            if self._connected:
                rospy.logwarn("Teraranger node subprocess as quit, stop publishing data.")
            self._connected = False


if __name__ == "__main__":
    Teraranger()
