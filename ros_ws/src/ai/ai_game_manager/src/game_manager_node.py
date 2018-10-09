#!/usr/bin/python
import time
import rospy

from game_manager import StatusManager, TimerManager

if __name__ == "__main__":
    rospy.init_node("game_manager", log_level = rospy.INFO)
    status = StatusManager()
    timer = TimerManager()

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        status.update()
        timer.update()

