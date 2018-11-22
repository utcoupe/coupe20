#!/usr/bin/python
import math
import rospy
from visualization_msgs.msg import Marker


class MarkersPublisher(object):
    def __init__(self):
        self.MARKERS_TOPIC = "/visualization_markers/navigation"
        self.MarkersPUBL = rospy.Publisher(self.MARKERS_TOPIC, Marker, queue_size=10)

    def _is_connected(self):
        return bool(self.MarkersPUBL.get_num_connections())

    def publishCheckZones(self, robot):
        if self._is_connected():
            for i, main_shape in enumerate(robot.get_main_shapes()):
                self._publish_marker("collisions_main", i, main_shape, 0.02, 0.01, (1.0, 0.0, 0.0, 0.8))

            for i, path_shape in enumerate(robot.get_path_shapes()):
                self._publish_marker("collisions_path", i, path_shape, 0.02, 0.01, (1.0, 0.5, 0.1, 0.8))

    def publishObstacles(self, obstacles): # Temporaire ?
        if self._is_connected():
            counter = 0
            for i, obs in enumerate(obstacles):
                self._publish_marker("collisions_obstacles", i + counter, obs, 0.35, 0.35 / 2.0, (1.0, 0.8, 0.3, 0.8))

                if obs.velocity is not None: # if the bostacle has a velocity, draw its rect too.
                    for vel_shape in obs.velocity.get_shapes(obs.position):
                        counter += 1
                        self._publish_marker("collisions_obstacles", i + counter, vel_shape, 0.02, 0.01, (1.0, 0.0, 0.0, 0.8))

    def _publish_marker(self, ns, index, obj, z_scale, z_height, color):
        markertypes = {
            "segment": Marker.CUBE,
            "rect": Marker.CUBE,
            "circle": Marker.CYLINDER,
            "mesh": Marker.MESH_RESOURCE
        }
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = markertypes[str(obj)] # TODO
        marker.ns = ns
        marker.id = index

        marker.action = Marker.ADD
        if str(obj) == "rect":
            marker.scale.x = obj.width
            marker.scale.y = obj.height
        elif str(obj) == "circle":
            marker.scale.x = obj.radius * 2.0
            marker.scale.y = obj.radius * 2.0
        elif str(obj) == "segment":
            marker.scale.x = obj.length
            marker.scale.y = 0.02
        marker.scale.z = z_scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.position.x = obj.position.x
        marker.pose.position.y = obj.position.y
        marker.pose.position.z = z_height
        orientation = self._euler_to_quaternion([0, 0, obj.position.a])
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.lifetime = rospy.Duration(0.1)

        self.MarkersPUBL.publish(marker)

    def _euler_to_quaternion(self, xyz):
        cr, sr = math.cos(xyz[0] * 0.5), math.sin(xyz[0] * 0.5)
        cp, sp = math.cos(xyz[1] * 0.5), math.sin(xyz[1] * 0.5)
        cy, sy = math.cos(xyz[2] * 0.5), math.sin(xyz[2] * 0.5)
        return (cy * sr * cp - sy * cr * sp,  # qx
                cy * cr * sp + sy * sr * cp,  # qy
                sy * cr * cp - cy * sr * sp,  # qz
                cy * cr * cp + sy * sr * sp)  # qw
