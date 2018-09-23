#!/usr/bin/python
import math, time
import rospy
from map_manager import map_attributes, DictManager
from visualization_msgs.msg import Marker


class MarkersPublisher():
    def __init__(self):
        self._prev_num_connections = 0
        self.MARKERS_TOPIC = "/visualization_markers/world"
        self._pub = rospy.Publisher(self.MARKERS_TOPIC, Marker, queue_size=10)

    def _is_connected(self):
        return bool(self._pub.get_num_connections())

    def _connections_has_changed(self):
        if self._prev_num_connections != self._pub.get_num_connections():
            time.sleep(0.3)
            return True
        else:
            return False

    def updateMarkers(self, world):
        if (self._is_connected() and self._connections_has_changed()) or world.is_dirty(): # Draw if new connection or new content
            self._publish_table(world)                      # Table STL without colors
            self._publish_robot_stl(world)                  # Simple rectangle representing the robot shape.
            self._publish_zones(world)                      # Departure areas
            self._publish_waypoints(world)                  # Navigation positions
            self._publish_objects(world.get("/objects/^"))  # game elements (balls, cubes...)

        self._prev_num_connections = self._pub.get_num_connections()

    def _publish_table(self, world):
        pos = map_attributes.Position2D({
            "frame_id": "/map",
            "x": -0.022,
            "y": 0.022 + 2,
            "type": "fixed"
        })
        self._publish_marker(0, pos, world.get("/terrain/_marker/^"))

    def _publish_robot_stl(self, world):
        pos = map_attributes.Position2D({
            "frame_id": "/robot",
            "x": 0,
            "y": 0,
            "type": "fixed"
        })
        self._publish_marker(0, pos, world.get("/entities/{}/_marker/^".format(rospy.get_param("/robot"))))

    def _publish_zones(self, world):
        for i, z in enumerate(world.get("/zones/^").toList()):
            self._publish_marker(i, z.get("position/^"), z.get("_marker/^"))

    def _publish_waypoints(self, world):
        i = 0
        for z in world.get("/waypoints/^").toList():
            m = DictManager({
                "ns": "waypoints",
                "type": "arrow",
                "scale": [0.08, 0.03, 0.03],
                "z": 0.08,
                "orientation": [0, 1.57079, 0],
                "color": map_attributes.Color("brown", [0.8, 0.5, 0.1, 0.95])
            })
            self._publish_marker(i, z.get("position/^"), m)
            if "a" in z.get("position/^").Dict.keys():
                i += 1
                n = DictManager({
                    "ns": "waypoints",
                    "type": "arrow",
                    "scale": [0.06, 0.015, 0.015],
                    "z": 0,
                    "orientation": [0, 0, z.get("position/^").Dict["a"]],
                    "color": map_attributes.Color("brown", [0.8, 0.5, 0.1, 0.95])
                })
                self._publish_marker(i, z.get("position/^"), n)
            i += 1

    def _publish_objects(self, objects_dictman, j = 0): # TODO containers inside containers
        i = 0
        for e in objects_dictman.Dict.keys():
            time.sleep(0.005)
            if "container_" in e:
                i += self._publish_objects(objects_dictman.get("{}/^".format(e)), i)
            else:
                self._publish_marker(i + j, objects_dictman.Dict[e].get("position/^"), objects_dictman.Dict[e].get("_marker/^"))
            i += 1
        return i

    def _publish_marker(self, marker_id, position, visual):
        markertypes = {
            "cube": Marker.CUBE,
            "sphere": Marker.SPHERE,
            "arrow": Marker.ARROW,
            "mesh": Marker.MESH_RESOURCE
        }

        marker = Marker()
        marker.header.frame_id = position.get("frame_id")
        marker.type = markertypes[visual.get("type")]
        marker.ns = visual.get("ns")
        marker.id = marker_id
        marker.frame_locked = True

        marker.action = Marker.ADD
        marker.scale.x = visual.get("scale")[0]
        marker.scale.y = visual.get("scale")[1]
        marker.scale.z = visual.get("scale")[2]
        marker.color.r = visual.Dict["color"].Dict["r"]
        marker.color.g = visual.get("color/^").get("g")
        marker.color.b = visual.get("color/^").get("b")
        marker.color.a = visual.get("color/^").get("a")
        marker.pose.position.x = position.get("x")
        marker.pose.position.y = position.get("y")
        marker.pose.position.z = visual.get("z")
        orientation = self._euler_to_quaternion(visual.get("orientation"))
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.mesh_resource = visual.get("mesh_path") if marker.type == Marker.MESH_RESOURCE else ''

        self._pub.publish(marker)

    def _euler_to_quaternion(self, xyz):
        cr, sr = math.cos(xyz[0] * 0.5), math.sin(xyz[0] * 0.5)
        cp, sp = math.cos(xyz[1] * 0.5), math.sin(xyz[1] * 0.5)
        cy, sy = math.cos(xyz[2] * 0.5), math.sin(xyz[2] * 0.5)
        return (cy * sr * cp - sy * cr * sp,  # qx
                cy * cr * sp + sy * sr * cp,  # qy
                sy * cr * cp - cy * sr * sp,  # qz
                cy * cr * cp + sy * sr * sp)  # qw
