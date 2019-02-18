#!/usr/bin/python
import json
import time
import xml.etree.ElementTree as ET

import rospy
import static_map.msg
import static_map.srv
from map_manager import SetMode, MapManager, Object, Container, Waypoint, Terrain, Position2D
from occupancy import OccupancyGenerator


class Servers():
    GET_CONTAINER_SRV = "static_map/get_container"
    GET_WAYPOINT_SRV  = "static_map/get_waypoint"
    GET_TERRAIN_SRV   = "static_map/get_terrain"
    SET_SRV           = "static_map/set"
    TRANSFER_SRV      = "static_map/transfer"

class MapServices():
    def __init__(self, occupancy_generator):
        self._get_container_srv = rospy.Service(Servers.GET_CONTAINER_SRV, static_map.srv.MapGetContainer, self.on_get_container)
        self._get_waypoint_srv  = rospy.Service(Servers.GET_WAYPOINT_SRV,  static_map.srv.MapGetWaypoint,  self.on_get_waypoint)
        self._get_terrain_srv   = rospy.Service(Servers.GET_TERRAIN_SRV,   static_map.srv.MapGetTerrain,   self.on_get_terrain)
        self._transfer_srv      = rospy.Service(Servers.TRANSFER_SRV,      static_map.srv.MapTransfer,    self.on_transfer)

    def on_get_container(self, req):
        # Fetch it from the map
        ct = MapManager.get_container([s for s in req.path.split('/') if s])

        if ct is None:
            return static_map.srv.MapGetContainerResponse(False, None)

        # Construct the message reply
        msg = static_map.srv.MapGetContainerResponse()
        msg.success = True
        msg.container = self._create_container_msg(ct, req.include_subcontainers)
        rospy.loginfo("GET Container (path={}): {} object(s) returned.".format(req.path, len(msg.container.objects)))
        return msg
    
    def on_get_waypoint(self, req):
        pos = Position2D(None, validate=False)
        pos.X = req.waypoint.pose.x
        pos.Y = req.waypoint.pose.y
        pos.A = req.waypoint.pose.theta
        pos.HasAngle = req.waypoint.has_angle

        w = MapManager.get_waypoint(req.waypoint.name, pos)
        success = False
        if w:
            success = True

        rospy.loginfo("GET Waypoint (name='{}' x={} y={} a={}): returning {}".format(
            req.waypoint.name, req.waypoint.pose.x, req.waypoint.pose.y, req.waypoint.pose.theta,
            "None" if success is False else "name='{}' x={} y={} a={}".format(w.Name, w.Position.X, w.Position.Y, w.Position.A)))
        return static_map.srv.MapGetWaypointResponse(success, self._create_waypoint_msg(w))
    
    def on_get_terrain(self, req):
        terrain = MapManager.get_terrain()

        msg = static_map.srv.MapGetTerrainResponse()
        msg.success = True
        msg.shape = self._create_object_msg(terrain)
        msg.layers = []

        for l in terrain.Layers:
            msg_layer = static_map.msg.MapLayer()
            msg_layer.name  = l.Name
            msg_layer.walls = [self._create_object_msg(w) for w in l.Walls]
            msg.layers.append(msg_layer)
        return msg
    

    def on_set(self, req):
        rospy.loginfo("SET:" + str(req.path))
        path = [s for s in req.path.split('/') if s]
        obj  = self._object_from_msg(req.object)

        res_obj = None
        success = False
        if req.mode == req.MODE_ADD:
            success = MapManager.add_object(path, obj)
        elif req.mode == req.MODE_REMOVE:
            res_obj = MapManager.remove_object(path)

        if success or res_obj:
            MapManager.Dirty = True
        return static_map.srv.MapSetObjectResponse(success, self._create_object_msg(res_obj))

    def on_transfer(self, req):
        old_path = path = [s for s in req.old_path.split('/') if s]
        new_path = path = [s for s in req.new_path.split('/') if s]

        success = False
        if req.mode == req.MODE_OBJECT:
            moved_obj = MapManager.remove_object(old_path)
            if moved_obj is not None:
                success = MapManager.add_object(new_path, moved_obj)
            else:
                success = False
        elif req.mode == req.MODE_CONTAINER:
            rospy.logerr("container transfer not implemented")
        elif req.mode == req.MODE_CONTAINER_OBJECTS:
            old_ct = MapManager.get_container(old_path)
            new_ct = MapManager.get_container(new_path)

            if old_ct and new_ct:
                new_ct.Elements += old_ct.Elements
                old_ct.Elements = []
                success = True
        
        rospy.loginfo("TRANSFER (from={} to={}): {}".format(req.old_path, req.new_path, success))
        return static_map.srv.MapTransferResponse(success)

    def on_get_occupancy(self, req):
        s = time.time() * 1000
        rospy.loginfo("GET_OCCUPANCY:" + str(req.layer_name))

        path = self.occupancy_generator.generateLayer(Map, req.layer_name, req.img_width, req.margin)
        try:
            path = self.occupancy_generator.generateLayer(Map, req.layer_name, req.img_width, req.margin)
        except Exception as e:
            rospy.logerr("    Request failed : " + str(e))

        rospy.logdebug("    Responding: " + str(path))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return static_map.srv.MapGetOccupancyResponse(path)

    def _create_container_msg(self, ct, include_subcontainers):
        msg = static_map.msg.MapContainer()
        msg.name = ct.Name

        for e in ct.Elements:
            if isinstance(e, Container) and include_subcontainers is True:
                c = self._create_container_msg(e, include_subcontainers)
                msg.objects += c.objects
            elif isinstance(e, Object):
                msg.objects.append(self._create_object_msg(e))
        return msg

    def _create_object_msg(self, obj):
        msg = static_map.msg.MapObject()
        if obj is not None:
            msg.name = obj.Name

            msg.pose.x     = obj.Position.X
            msg.pose.y     = obj.Position.Y
            msg.pose.theta = obj.Position.A        
            
            if obj.Shape.Type == "rect":
                msg.shape_type = msg.SHAPE_RECT
                msg.width  = obj.Shape.Width
                msg.height = obj.Shape.Height
            elif obj.Shape.Type == "circle":
                msg.shape_type = msg.SHAPE_CIRCLE
                msg.radius = obj.Shape.Radius
            elif obj.Shape.Type == "point":
                msg.shape_type = msg.SHAPE_POINT
            
            if isinstance(obj, Object):
                msg.tags   = obj.Tags
                msg.color  = obj.Color.Name if obj.Color is not None else ""
        return msg
    
    def _create_waypoint_msg(self, way):
        msg = static_map.msg.Waypoint()
        if way is not None:
            msg.name = way.Name
            msg.pose.x = way.Position.X
            msg.pose.y = way.Position.Y
            msg.pose.theta = way.Position.A
            msg.has_angle = way.Position.HasAngle
            msg.frame_id = way.Position.Frame
        return msg

    def _object_from_msg(self, msg):
        xml = ET.Element("object")
        xml.attrib["name"] = msg.name
        position = ET.SubElement(xml, "position")
        position.attrib["x"] = float(msg.pose.x)
        position.attrib["y"] = float(msg.pose.y)
        position.attrib["a"] = float(msg.pose.theta)

        shape    = ET.SubElement(xml, "shape")
        shape.attrib["type"]  = ["rect", "circle", "point"][msg.shape_type]
        if msg.shape_type == msg.SHAPE_RECT:
            shape.attrib["w"] = float(msg.width)
            shape.attrib["h"] = float(msg.height)
        elif msg.shape_type == msg.SHAPE_CIRCLE:
            shape.attrib["r"] = float(msg.radius)
        
        tags = ET.SubElement(xml, "tags")
        for tag_name in msg.tags:
            tag = ET.SubElement(tags, "tag")
            tag.text = tag_name
        
        #TODO marker, needed ? 
        
        if msg.color:
            color = ET.SubElement(xml, "color")
            color.attrib["name"] = msg.color
        return Object(xml, [])
