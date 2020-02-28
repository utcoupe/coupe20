#!user/bin/env python

import os
import rospy
import math

from ard_asserv.srv import SetPos
from ard_asserv.srv import Speed
from geometry_msgs.msg import Pose2D

SRV_SPD = "/drivers/ard_asserv/speed"
SRV_SET_POS = "/drivers/ard_asserv/set_pos"

TESTS = [
    ["Auto stop forward", Pose2D(1, 1.5, math.pi/2 + 0.2), 0.5, 0, 10, True],
    ["Auto stop backwards", Pose2D(1, 1.5, - math.pi/2 + 0.2), -0.5, 0, 10, True],
    ["1m/s forward", Pose2D(0.2, 1, 0), 1, 0, 3, False],
    ["1m/s backwards", Pose2D(2.8, 1, 0), -1, 0, 3, False],
    ["0.5m/s angular anti-clockwise", Pose2D(1.5, 1, 0), 0, 0.5, 3, False],
    ["0.5m/s angular clockwise", Pose2D(1.5, 1, 0), 0, -0.5, 3, False],
    ["1m/s forward and angular anti-clockwise", Pose2D(.5, 1, 0), 1, 0.5, 3, False],
    ["1m/s forward and angular clockwise", Pose2D(.5, 1, 0), 1, -0.5, 3, False],
    ["1m/s backwards and angular anti-clockwise", Pose2D(2.5, 1, 0), -1, 0.5, 3, False],
    ["1m/s backwards and angular clockwise", Pose2D(2.5, 1, 0), -1, -0.5, 3, False]
]

def main():
    print("Waiting for services speed and set_pos...")
    rospy.wait_for_service(SRV_SPD)
    rospy.wait_for_service(SRV_SET_POS)

    raw_input("Testing service speed : " + SRV_SPD +
              ".\nPress enter to begin.")
    os.system('clear')

    for test_values in TESTS:
        test(test_values)

    print("Testing done for service SPD.\n")

def test(test_values):
    """
    @param test_values: [
        description : string
        start_pos : Pose2D
        lin : m/s (float)
        ang : m/s (float)
        duration : s (float)
        auto_stop : bool, stops when blocked
    ]
    """
    description, start_pos, lin, ang, duration, auto_stop = test_values
    set_pos = rospy.ServiceProxy(SRV_SET_POS, SetPos)
    speed = rospy.ServiceProxy(SRV_SPD, Speed)

    print("\n" + description + " :")
    raw_input("lin = " + str(lin) + ", ang = " + str(ang) +
              ", duration = " + str(duration) +
              ", auto_stop = " + str(auto_stop) + 
              ".\nPress enter to test.")
    set_pos(0, start_pos, "")
    speed(lin, ang, duration, auto_stop)

if __name__ == "__main__":
    main()