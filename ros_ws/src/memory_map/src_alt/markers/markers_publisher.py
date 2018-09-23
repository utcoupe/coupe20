#!/usr/bin/python
import math
import rospy
from map_manager import map_attributes
from visualization_msgs.msg import Marker


class MarkersPublisher():
    def __init__(self):
        self.MARKERS_TOPIC = "/visualization_markers/world"
        self.MarkersPUBL = rospy.Publisher(self.MARKERS_TOPIC, Marker, queue_size=10)

    def _is_connected(self):
        return bool(self.MarkersPUBL.get_num_connections())

    def updateMarkers(self, world):
        if self._is_connected():
            self._publish_table(world)
            self._publish_robot_stl(world)
            self._update_zones(world)
            self._publish_objects(world.get("/objects/^"))

    def _publish_table(self, world):
        pos = map_attributes.Position2D({
            "frame_id": "/map",
            "x": -0.022,
            "y": 0.022 + 2,
            "type": "fixed"
        })
        self._publish_marker(pos, world.get("/terrain/marker/^"))

    def _publish_robot_stl(self, world):
        pos = map_attributes.Position2D({
            "frame_id": "/robot",
            "x": 0,
            "y": 0,
            "type": "fixed"
        })
        self._publish_marker(pos, world.get("/entities/{}/marker/^".format(rospy.get_param("/robot"))))

    def _update_zones(self, world):
        for z in world.get("/zones/^").toList():
            self._publish_marker(z.get("position/^"), z.get("marker/^"))

    def _publish_objects(self, objects_dictman):
        for e in objects_dictman.Dict.keys():
            if "container_" in e:
                self._publish_objects(objects_dictman.get("{}/^".format(e)))
            else:
                self._publish_marker(objects_dictman.Dict[e].get("position/^"), objects_dictman.Dict[e].get("marker/^"))

    def _publish_marker(self, position, visual):
        markertypes = {
            "cube": Marker.CUBE,
            "sphere": Marker.SPHERE,
            "mesh": Marker.MESH_RESOURCE
        }
        marker = Marker()
        marker.header.frame_id = position.get("frame_id")
        marker.type = markertypes[visual.get("type")]
        marker.ns = visual.get("ns")
        marker.id = visual.get("id")

        marker.action = Marker.ADD
        marker.scale.x = visual.get("scale")[0]
        marker.scale.y = visual.get("scale")[1]
        marker.scale.z = visual.get("scale")[2]
        marker.color.r = visual.get("color")[0]
        marker.color.g = visual.get("color")[1]
        marker.color.b = visual.get("color")[2]
        marker.color.a = visual.get("color")[3]
        marker.pose.position.x = position.get("x")
        marker.pose.position.y = position.get("y")
        marker.pose.position.z = visual.get("z")
        orientation = self._euler_to_quaternion(visual.get("orientation"))
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.mesh_resource = visual.get("mesh_path") if marker.type == Marker.MESH_RESOURCE else ''

        self.MarkersPUBL.publish(marker)

    def _euler_to_quaternion(self, xyz):
        cr, sr = math.cos(xyz[0] * 0.5), math.sin(xyz[0] * 0.5)
        cp, sp = math.cos(xyz[1] * 0.5), math.sin(xyz[1] * 0.5)
        cy, sy = math.cos(xyz[2] * 0.5), math.sin(xyz[2] * 0.5)
        return (cy * sr * cp - sy * cr * sp,  # qx
                cy * cr * sp + sy * sr * cp,  # qy
                sy * cr * cp - cy * sr * sp,  # qz
                cy * cr * cp + sy * sr * sp)  # qw
