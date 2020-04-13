#!user/bin/env python

import os
import roslib
import rospy
import actionlib
import math
import time

from ard_asserv.srv import SetPos
from navigator.msg import DoGotoAction, DoGotoGoal
from geometry_msgs.msg import Pose2D

SRV_SET_POS = "/drivers/ard_asserv/set_pos"
ACTION_GOTO = "/navigation/navigator/goto_action"

MODES = {"GOTO":0, "GOTOA":1}
DIRECTIONS = {"BACKWARD":0, "FORWARD":1, "AUTOMATIC":2}

TESTS = [
    ["GOTOA", "BACKWARD", False, Pose2D(0.5, 0.5, 0), 
     Pose2D(1.5, 1.5, 0), False, False, "bouees"],
    ["GOTO", "FORWARD", False, Pose2D(1.5, 1.5, 0), 
     Pose2D(.5, .5, 0), False, False, "bouees"]
]

def main():
    print("Waiting for set_pos service...")
    rospy.wait_for_service(SRV_SET_POS)
    rospy.init_node("goto_action_client")

    raw_input("Testing action goto : " + ACTION_GOTO +
              ".\nPress enter to begin.")
    os.system('clear')

    for test_values in TESTS:
        test(test_values)

    print("Testing done for action goto.")

def test(test_values):
    """
    @param test_values: [
        mode: "GOTO" or "GOTOA"
        direction: "BACKWARD", "FORWARD" or "AUTOMATIC"
        slow_go: bool
        start: Pose2D
        end: Pose2D
        disable_colisions: bool
        disable_pathfinder: bool
        ignore_tags : ["tag1", "tag2"]
    ]
    """
    (mode, direction, slow_go, start, end, disable_collisions,
     disable_pathfinder, ignore_tags) = test_values
    set_pos = rospy.ServiceProxy(SRV_SET_POS, SetPos)
    goto = actionlib.SimpleActionClient(ACTION_GOTO, DoGotoAction)
    goto.wait_for_server()

    raw_input("\nstart : x = " + str(start.x) + ", y = " + 
              str(start.y) + ", a = " + str(start.theta) +
              "\nend : x = " + str(end.x) + ", y = " + 
              str(end.y) + ", a = " + str(end.theta) + 
              "\nmode = " + mode + ", slow_go = " + str(slow_go) + 
              ", direction = " + direction +
              "\ndisable_collisions = " + str(disable_collisions) +
              ", disable_pathfinder = " + str(disable_pathfinder) +
              "\nignore_tags = " + str(ignore_tags) +
              ".\nPress enter to test.")
    set_pos(0, start, "")
    time.sleep(0.3) # Wait for set_pos
    goal = DoGotoGoal()
    goal.mode = MODES[mode]
    goal.direction = DIRECTIONS[direction]
    goal.slow_go = slow_go
    goal.target_pos = end
    goal.disable_collisions = disable_collisions
    goal.disable_pathfinder = disable_pathfinder
    goal.ignore_tags = ignore_tags

    goto.send_goal(goal)

if __name__ == "__main__":
    main()