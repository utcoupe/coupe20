#!/usr/bin/python
import math, time
import rospy
import xml.etree.ElementTree as ET
from map_manager import map_attributes, Container, Object
from visualization_msgs.msg import Marker


class MarkersPublisher():
    MARKERS_TOPIC = "/visualization_markers/world"

    def __init__(self):
        self._prev_num_connections = 0
        self._pub = rospy.Publisher(MarkersPublisher.MARKERS_TOPIC, Marker, queue_size=10)

    def _is_connected(self):
        return bool(self._pub.get_num_connections())

    def _connections_has_changed(self):
        if self._prev_num_connections != self._pub.get_num_connections():
            time.sleep(0.3)
            return True
        return False

    def updateMarkers(self, world):
        if (self._is_connected() and self._connections_has_changed()) or world.Dirty: # Draw if new connection or new content
            self._publish_table(world)             # Table STL without colors
            self._publish_robot_stl(world)         # Simple rectangle representing the robot shape.
            self._publish_waypoints(world)         # Navigation positions
            self._publish_objects(world.Objects)   # game elements (balls, cubes...)
            world.Dirty = False

        self._prev_num_connections = self._pub.get_num_connections()

    def _publish_table(self, world):
        self._publish_marker(0, world.Terrain.Position, world.Terrain.Marker, world.Terrain.Color)

    def _publish_robot_stl(self, world):
        pos = map_attributes.Position2D(None, validate=False)
        pos.Frame = "robot"
        self._publish_marker(0, pos, world.Robot.Marker, world.Robot.Color)

    def _publish_waypoints(self, world):
        xml = ET.Element("marker")
        xml.attrib["ns"]    = "waypoints"
        xml.attrib["type"]  = "sphere"
        xml.attrib["z"]  = 0.0
        scale = ET.SubElement(xml, "scale")
        scale.attrib["x"] = 0.05
        scale.attrib["y"] = 0.05
        scale.attrib["z"] = 0.01
        orientation = ET.SubElement(xml, "orientation")
        orientation.attrib["x"] = 0.0
        orientation.attrib["y"] = 0.0#1.57079
        orientation.attrib["z"] = 0.0#1.57079
        down_arrow = map_attributes.Marker(xml)

        side_arrow = map_attributes.Marker(xml)
        side_arrow.Type = "arrow"
        side_arrow.Scale = [0.05, 0.03, 0.001]
        side_arrow.Z = 0.0
        side_arrow.Orientation = [0, 0, 0]

        xml = ET.Element("color")
        xml.attrib["name"] = "waypoints"
        arrow_color = map_attributes.Color(xml)

        i = 0
        for w in world.Waypoints:
            self._publish_marker(i, w.Position, down_arrow, arrow_color)
            if w.Position.HasAngle:
                i += 1
                side_arrow.Orientation[2] = w.Position.A
                self._publish_marker(i, w.Position, side_arrow, arrow_color)
            i += 1

    def _publish_objects(self, container, _j = 0): # TODO containers inside containers
        i = 0
        for e in container.Elements:
            time.sleep(0.005)
            if isinstance(e, Container):
                i += self._publish_objects(e, i)
            else:
                self._publish_marker(i + _j, e.Position, e.Marker, e.Color)
            i += 1
        return i

    def _publish_marker(self, marker_id, position, marker, color):
        if not position or not marker:
            return

        markertypes = {
            "cube": Marker.CUBE,
            "cylinder": Marker.CYLINDER,
            "sphere": Marker.SPHERE,
            "arrow": Marker.ARROW,
            "mesh": Marker.MESH_RESOURCE
        }

        m = Marker()
        m.header.frame_id = position.Frame
        m.type = markertypes[marker.Type]
        m.ns = marker.Namespace
        m.id = marker_id
        m.frame_locked = True
        m.action = Marker.ADD

        m.scale.x = marker.Scale[0]
        m.scale.y = marker.Scale[1]
        m.scale.z = marker.Scale[2]
        if color:
            m.color.r = color.R
            m.color.g = color.G
            m.color.b = color.B
            m.color.a = color.A
        else: # default color to red
            m.color.r = m.color.a = 1.0
            m.color.g = m.color.b = 0.0
        m.pose.position.x = position.X
        m.pose.position.y = position.Y
        m.pose.position.z = marker.Z
        ori = marker.Orientation
        ori[2] += position.A # Also count the position yaw
        orientation = self._euler_to_quaternion(ori)
        m.pose.orientation.x = orientation[0]
        m.pose.orientation.y = orientation[1]
        m.pose.orientation.z = orientation[2]
        m.pose.orientation.w = orientation[3]

        m.mesh_resource = marker.MeshPath

        self._pub.publish(m)

    def _euler_to_quaternion(self, xyz):
        cr, sr = math.cos(xyz[0] * 0.5), math.sin(xyz[0] * 0.5)
        cp, sp = math.cos(xyz[1] * 0.5), math.sin(xyz[1] * 0.5)
        cy, sy = math.cos(xyz[2] * 0.5), math.sin(xyz[2] * 0.5)
        return (cy * sr * cp - sy * cr * sp,  # qx
                cy * cr * sp + sy * sr * cp,  # qy
                sy * cr * cp - cy * sr * sp,  # qz
                cy * cr * cp + sy * sr * sp)  # qw
