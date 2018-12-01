#!/usr/bin/python
import json
import time

import rospy
import static_map.msg
import static_map.srv
from map_manager import SetMode, MapManager, Object, Container, Waypoint, Terrain, Position2D
from occupancy import OccupancyGenerator


class Servers():
    GET_CONTAINER_SRV        = "static_map/get_container"
    GET_WAYPOINT_SRV         = "static_map/get_waypoint"
    GET_TERRAIN_SRV          = "static_map/get_terrain"

    SET_SERV        = "static_map/set"
    TRANSFER_SERV   = "static_map/transfer"
    OCCUPANCY_SERV  = "static_map/get_occupancy"
    OBJECTS_SERV    = "static_map/get_objects"

class MapServices():
    def __init__(self, occupancy_generator):
        self._get_container_srv = rospy.Service(Servers.GET_CONTAINER_SRV, static_map.srv.MapGetContainer, self.on_get_container)
        self._get_waypoint_srv  = rospy.Service(Servers.GET_WAYPOINT_SRV, static_map.srv.MapGetWaypoint,   self.on_get_waypoint)
        self._get_terrain_srv   = rospy.Service(Servers.GET_TERRAIN_SRV, static_map.srv.MapGetTerrain,     self.on_get_terrain)
        # self.GetSERV       = rospy.Service(Servers.GET_SERV, static_map.srv.MapGet,                self.on_get)
        # self.SetSERV       = rospy.Service(Servers.SET_SERV, static_map.srv.MapSet,                self.on_set) #TODO
        # self.TransferSERV  = rospy.Service(Servers.TRANSFER_SERV, static_map.srv.MapTransfer,      self.on_transfer)
        # self.OccupancySERV = rospy.Service(Servers.OCCUPANCY_SERV, static_map.srv.MapGetOccupancy, self.on_get_occupancy)
        # self.ObjectsSERV   = rospy.Service(Servers.OBJECTS_SERV, static_map.srv.MapGetObjects,     self.on_get_objects)

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
        print "hasangle = " + str(pos.HasAngle)

        w = MapManager.get_waypoint(req.waypoint.name, pos)
        success = False
        if w is not None:
            success = True

        rospy.loginfo("GET Waypoint (name='{}' x={} y={} a={}): returning {}".format(
            req.waypoint.name, req.waypoint.pose.x, req.waypoint.pose.y, req.waypoint.pose.theta,
            "None" if success is False else "name='{}' x={} y={} a={}".format(w.Name, w.Position.X, w.Position.Y, w.Position.A)))
        return static_map.srv.MapGetWaypointResponse(success, self._create_waypoint_msg(w))
    
    def on_get_terrain(self, req):
        terrain = MapManager.get_terrain()

        msg = static_map.srv.MapGetTerrainResponse()
        msg.success = True
        msg.shape = self._create_object_msg(terrain.Shape)
        msg.layers = []

        for l in terrain.Layers:
            msg.layers.append(static_map.msg.MapLayer)
            msg.layers[-1].name = l.Name
            msg.layers[-1].walls = []

            for w in l.Walls:
                msg.layers[-1].walls.append(self._create_object_msg(w))
        return msg

    # def on_get(self, req):
    #     s = time.time() * 1000
    #     rospy.loginfo("GET:" + str(req.request_path))

    #     success = False
    #     response = MapManager.get(req.request_path)
    #     if isinstance(response, DictManager):
    #         rospy.logerr("    GET Request failed : '^' dict operator not allowed in services.")
    #         response = None

    #     if response != None:
    #         success = True

    #     rospy.logdebug("    Responding: " + str(response))
    #     rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
    #     return static_map.srv.MapGetResponse(success, json.dumps(response))

    # def on_get_objects(self, req):
    #     s = time.time() * 1000
    #     rospy.loginfo("GET_OBJECTS:collisions_only=" + str(req.collisions_only))

    #     success = False
    #     objects = MapManager.get_objects(collisions_only=req.collisions_only)

    #     if objects != None:
    #         success = True

    #     rospy.logdebug("    Responding: {} object(s) found.".format(len(objects)))
    #     rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
    #     return static_map.srv.MapGetObjectsResponse(success, objects)

    def on_set(self, req):
        s = time.time() * 1000
        rospy.loginfo("SET:" + str(req.request_path))

        success = False
        try:
            success = MapManager.set(req.request_path, req.mode)
        except Exception as e:
            rospy.logerr("    SET Request failed (python reason) : " + str(e))

        if success:
            MapManager.Dirty = True

        rospy.logdebug("    Responding: " + str(success))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return static_map.srv.MapSetResponse(success)

    def on_transfer(self, req):
        s = time.time() * 1000
        rospy.loginfo("TRANSFER:{} to {}".format(req.old_path, req.new_path))
        elem, elem_name = MapManager.get(req.old_path + "/^"), req.old_path.split('/')[-1]
        if elem:
            success = MapManager.set(req.old_path, SetMode.MODE_DELETE) and \
                      MapManager.set(req.new_path + "/{}".format(elem_name), SetMode.MODE_ADD, instance = elem)
            MapManager.Dirty = True
        else:
            rospy.logerr("    TRANSFER Request failed : could not find the object at old_path '{}'.".format(req.old_path))
            success = False

        rospy.logdebug("    Responding: " + str(success))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
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
        msg.name = ""
        
        if obj.Shape.Type == "rect":
            msg.shape_type = msg.SHAPE_RECT
            msg.width  = obj.Shape.Width
            msg.height = obj.Shape.Height
        elif obj.Shape.Type == "circle":
            msg.shape_type = msg.SHAPE_CIRCLE
            msg.radius = obj.Shape.Radius
        elif obj.Shape.Type == "point":
            msg.shape_type = msg.SHAPE_POINT
        
        msg.labels = obj.Labels
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