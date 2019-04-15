#!/usr/bin/python
import time
import rospy

from manager import StatusManager, TimerManager

if __name__ == "__main__":
    rospy.init_node("game_manager", log_level = rospy.INFO)
    status = StatusManager()
    timer = TimerManager()

    r = rospy.Rate(3)
    while not rospy.is_shutdown():
        status.update()
        timer.update()
        r.sleep()

