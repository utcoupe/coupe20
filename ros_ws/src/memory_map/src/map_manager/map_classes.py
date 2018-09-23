#!/usr/bin/python
import copy, json
import rospy
from map_loader import LoadingHelpers
from map_bases import DictManager
from map_attributes import Position2D, Shape2D, MarkerRViz, Trajectory
import map


class Terrain(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "shape", "_marker", "walls")

        # Instantiate the layers before creating the dict
        for layer in initdict["walls"]:
            initdict["walls"][layer] = Layer(initdict["walls"][layer])

        super(Terrain, self).__init__({
            "shape": Shape2D(initdict["shape"]),
            "_marker": MarkerRViz(initdict["_marker"]),
            "walls": DictManager(initdict["walls"]),
        })


class Layer(DictManager):
    def __init__(self, initdict):
        self.includes = []
        if "_include" in initdict.keys():
            self.includes = [i for i in initdict["_include"]]
            del initdict["_include"]

        # Instantiate the walls before creating the dict
        for wall in initdict:
            initdict[wall] = Wall(initdict[wall])

        super(Layer, self).__init__(initdict)


class Wall(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape")
        super(Wall, self).__init__({
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"])
        })


class Zone(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "_marker", "properties")
        super(Zone, self).__init__({
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"]),
            "_marker": MarkerRViz(initdict["_marker"], shape = Shape2D(initdict["shape"])),
            "properties": DictManager(initdict["properties"])
        })


class Waypoint(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position")
        super(Waypoint, self).__init__({
            "position": Position2D(initdict["position"])
        })


class Entity(DictManager):
    def __init__(self, initdict, obj_classes):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "_marker", "containers", "trajectory")

        for container in initdict["containers"]:
            initdict["containers"][container] = Container(initdict["containers"][container], obj_classes)

        super(Entity, self).__init__({
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"]),
            "_marker": MarkerRViz(initdict["_marker"]),
            "chest": DictManager(initdict["containers"]),
            "trajectory": Trajectory(initdict["trajectory"])
        })


class Container(DictManager):
    def __init__(self, initdict, obj_classes):
        for obj in initdict:
            if "container_" in obj:
                initdict[obj] = Container(initdict[obj], obj_classes)
            else:
                initdict[obj] = Object(initdict[obj], obj_classes)
        super(Container, self).__init__(initdict)

    def get_objects(self, collisions_only = False):
        objects = []
        for o in self.Dict:
            if isinstance(self.Dict[o], Container):
                objects += self.Dict[o].get_objects(collisions_only)
            elif isinstance(self.Dict[o], Object):
                if self.Dict[o].Dict["collision"] == collisions_only or collisions_only is False:
                    objects.append(json.dumps(self.Dict[o].get("*")))
            else:
                rospy.logwarn("Not recognized DictManager type found while retrieving map objects, passing.")
        return objects


class Object(DictManager):
    def __init__(self, initdict, obj_classes):
        # Autofilling if class is available
        if "class" in initdict:
            obj_class = copy.deepcopy([obj_classes[d] for d in obj_classes if d == initdict["class"]][0])
            new_initdict = LoadingHelpers.mergeDicts(obj_class, initdict)
            for field in ["position", "shape", "_marker"]:
                if field in initdict and field in obj_class:
                    new_initdict[field] = LoadingHelpers.mergeDicts(obj_class[field], initdict[field])
                elif field in initdict:
                    new_initdict[field] = initdict[field]
                elif field in obj_class:
                    new_initdict[field] = obj_class[field]
            initdict = new_initdict

        LoadingHelpers.checkKeysExist(initdict, "collision", "position", "shape", "_marker")

        d = {}
        if "collision" in initdict:
            d["collision"] = initdict["collision"]
        if "color" in initdict:
            d["color"] = [c for c in map.Map.Colors if c.Dict["name"] == initdict["color"]][0]
        if "properties" in initdict:
            d["properties"] = DictManager(initdict["properties"])
        d["shape"] = Shape2D(initdict["shape"])

        d["position"] = Position2D(initdict["position"])
        d["_marker"]   = MarkerRViz(initdict["_marker"], shape = d["shape"] if "shape" in d else None, \
                                                       color = d["color"] if "color" in d else None)
        super(Object, self).__init__(d)
