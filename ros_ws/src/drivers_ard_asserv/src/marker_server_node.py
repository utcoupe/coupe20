#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from visualization_msgs.msg import Marker
from drivers_ard_asserv.srv import SetPos, SetPosRequest

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose2D

import tf2_ros


def processFeedback(feedback):
    if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
        return
    
    p = feedback.pose.position
    try:
        srv = rospy.ServiceProxy("/drivers/ard_asserv/set_pos", SetPos)
        q = [[o.x, o.y, o.z, o.w] for o in [feedback.pose.orientation]][0]

        msg = SetPosRequest()
        msg.position.x = feedback.pose.position.x
        msg.position.y = feedback.pose.position.y
        msg.position.theta = euler_from_quaternion(q)[2]
        srv(msg)
    except Exception as e:
        rospy.logwarn(e)

def poseCallback(msg):
    pose = int_marker.pose
    pose.position.x = msg.x
    pose.position.y = msg.y
    q = quaternion_from_euler(0, 0, msg.theta)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    server.setPose(int_marker.name, pose)
    server.applyChanges()



if __name__ == "__main__":
    rospy.init_node("asserv_interactive_marker")

    server = InteractiveMarkerServer("asserv_marker")

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = "robot_pos"
    int_marker.scale = 0.5

    int_marker.pose.position.z = 0.001


    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.201
    box_marker.scale.y = 0.301
    box_marker.scale.z = 0.351

    box_marker.pose.position.z = 0.30 / 2.0 + 0.025

    box_marker.color.r = 191.0 / 255.0
    box_marker.color.g = 1.0
    box_marker.color.b = 244.0 / 255.0
    box_marker.color.a = 1.0



    box_control = InteractiveMarkerControl()
    box_control.name = "translate_plane"
    box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

    box_control.orientation.w = 1
    box_control.orientation.x = 0
    box_control.orientation.y = 1
    box_control.orientation.z = 0



    int_marker.controls.append(box_control)
    box_control.markers.append(box_marker)
    box_control.always_visible = True

    int_marker.controls.append(box_control)

    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "rotate_yaw"
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate_control.orientation.w = 1
    rotate_control.orientation.x = 0
    rotate_control.orientation.y = 1
    rotate_control.orientation.z = 0

    int_marker.controls.append(rotate_control)

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.Subscriber("/drivers/ard_asserv/pose2d", Pose2D, poseCallback)

    rospy.spin()

