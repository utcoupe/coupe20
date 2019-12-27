#!user/bin/env python

import math
import os
import rospy

from ard_asserv.srv import SetPos
from geometry_msgs.msg import Pose2D

SRV_SET_POS = "/drivers/ard_asserv/set_pos"
POS_MODES = {"AXY":0, "A":1, "Y":2, "AY":3, "X":4, "AX":5, "XY":6}
TESTS = [
    ["AXY", Pose2D(1.5, 1.2, math.pi/2),   ""],
    ["X",   Pose2D(0.5, 1.6, 0),           ""],
    ["Y",   Pose2D(1.0, 0.8, math.pi/4),   ""],
    ["A",   Pose2D(0.7, 1.7, math.pi/3),   ""],
    ["XY",  Pose2D(1.7, 1.9, math.pi),     ""],
    ["AX",  Pose2D(1.1, 1.2, 3*math.pi/4), ""],
    ["AY",  Pose2D(2.4, 0.4, math.pi/6),   ""],
]

def main():
    print("Waiting for set_pos service...")
    rospy.wait_for_service(SRV_SET_POS)

    raw_input("Testing service set_pos : " + SRV_SET_POS +
          ".\nPress enter to begin.")
    os.system('clear')

    for test_values in TESTS:
        test(test_values)

    print("Testing done for set_pos.\n")

def test(test_values):
    """
    @param test_values : [
        mode: string (AX means only A and X get modified)
        position: Pose2D
        position_waypoint: string
    ]
    """
    mode, position, position_waypoint = test_values
    set_pos = rospy.ServiceProxy(SRV_SET_POS, SetPos)
    set_pos(POS_MODES[mode], position, position_waypoint)
    raw_input("\nx = " + str(position.x) + ", y = " + 
              str(position.y) + ", a = " + str(position.theta) +
              ", mode = " + mode + ".\nPress enter for next test.")

if __name__ == "__main__":
    main()