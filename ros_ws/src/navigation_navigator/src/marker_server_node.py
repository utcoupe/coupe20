#!/usr/bin/env python

import actionlib
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from navigation_navigator.msg import DoGotoAction, DoGotoGoal
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import *
from visualization_msgs.msg import Marker


def processFeedback(feedback):
    pass

def goal_done(self, result):
    if result.success:
        pose = int_marker.pose
        pose.position.x = -1
        pose.position.y = 1

        server.setPose(int_marker.name, pose)
        server.applyChanges()

def processMenu(feedback):
    goal = DoGotoGoal()
    goal.mode = goal.GOTOA
    goal.direction = goal.AUTOMATIC
    goal.disable_collisions = False
    goal.target_pos.x = feedback.pose.position.x
    goal.target_pos.y = feedback.pose.position.y

    q = [[o.x, o.y, o.z, o.w] for o in [feedback.pose.orientation]][0]

    goal.target_pos.theta = euler_from_quaternion(q)[2]

    client.send_goal(goal, goal_done)

    rospy.loginfo("Interactive marker sent goto goal !")


if __name__ == "__main__":

    rospy.init_node("navigator_interactive_marker")

    client = actionlib.SimpleActionClient('/navigation/navigator/goto_action', DoGotoAction)
    client.wait_for_server()

    server = InteractiveMarkerServer("navigator_marker")

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = "goto_pos"

    # to make the rotation controls smaller
    int_marker.scale = 0.5

    # to make the rotation controls slightly above the ground so no glitch
    int_marker.pose.position.z = 0.001

    # position the marker outside the map so it does not block the view
    int_marker.pose.position.x = -1
    int_marker.pose.position.y = 1


    # 'real' marker that will be displayed : shape of the robot
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.201
    box_marker.scale.y = 0.301
    box_marker.scale.z = 0.351

    box_marker.pose.position.z = 0.30 / 2.0 + 0.025

    # translucent purple
    box_marker.color.r = 175.0 / 255.0
    box_marker.color.g = 0.0
    box_marker.color.b = 219.0 / 255.0
    box_marker.color.a = 0.3


    menu_handler = MenuHandler()
    menu_handler.insert("Let's go !", callback=processMenu)


    # control for panning
    box_control = InteractiveMarkerControl()
    box_control.name = "translate_plane"
    box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    box_control.orientation.w = 1
    box_control.orientation.x = 0
    box_control.orientation.y = 1
    box_control.orientation.z = 0
    box_control.always_visible = True

    int_marker.controls.append(box_control)

    # so the robot can be clicked and dragged
    box_control.markers.append(box_marker)


    # control for z rotation
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "rotate_yaw"
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate_control.orientation.w = 1
    rotate_control.orientation.x = 0
    rotate_control.orientation.y = 1
    rotate_control.orientation.z = 0

    # control for the context menu (right click)
    menu_control = InteractiveMarkerControl()
    menu_control.name = "menu"
    menu_control.interaction_mode = InteractiveMarkerControl.MENU
    # left- and right-clickable text
    menu_control.description = "Goto destination"
    menu_control.always_visible = True

    # so the controls are linked to the interactive marker
    int_marker.controls.append(rotate_control)
    int_marker.controls.append(menu_control)


    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    menu_handler.apply(server, int_marker.name)

    rospy.spin()

