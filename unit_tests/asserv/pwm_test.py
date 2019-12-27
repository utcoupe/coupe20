#!user/bin/env python

import os
import rospy
import math

from ard_asserv.srv import SetPos
from ard_asserv.srv import Pwm
from geometry_msgs.msg import Pose2D

SRV_PWM = "/drivers/ard_asserv/pwm"
SRV_SET_POS = "/drivers/ard_asserv/set_pos"

TESTS = [
    ["Auto stop forward", Pose2D(1, 1.5, math.pi/2 + 0.2), 75, 75, 5, True],
    ["Auto stop backwards", Pose2D(1, 1.5, -math.pi/2 + 0.2), -75, -75, 5, True],
    ["Max PWM forward", Pose2D(0.2, 1, 0), 255, 255, 2, False],
    ["Max PWM backwards", Pose2D(2.8, 1, 0), -255, -255, 2, False],
    ["Left forward, right backwards", Pose2D(1.5, 1, 0), 100, -100, 2, False],
    ["Left backwards, right forward", Pose2D(1.5, 1, 0), -100, 100, 2, False]
]

def main():
    print("Waiting for services pwm and set_pos...")
    rospy.wait_for_service(SRV_PWM)
    rospy.wait_for_service(SRV_SET_POS)

    raw_input("Testing service pwm : " + SRV_PWM + 
              ".\nPress enter to begin.")
    os.system('clear')

    for test_values in TESTS:
        test(test_values)

    print("Testing done for service PWM.\n")

def test(test_values):
    """
    @param test_values: [
        description : string to describe the test
        start_pos : starting position (Pose2D)
        left : left pwm (int)
        right : right pwm (int)
        duration : seconds (float)
        auto_stop : bool, stops when blocked against a wall
    ]
    """
    description, start_pos, left, right, duration, auto_stop = test_values
    set_pos = rospy.ServiceProxy(SRV_SET_POS, SetPos)
    pwm = rospy.ServiceProxy(SRV_PWM, Pwm)

    print("\n" + description + " :")
    raw_input("left = " + str(left) + ", right = " + str(right) +
              ", duration = " + str(duration) + 
              ", auto_stop = " + str(auto_stop) + 
              ".\nPress enter to test.")
    set_pos(0, start_pos, "")
    pwm(left, right, duration, auto_stop)

if __name__ == "__main__":
    main()