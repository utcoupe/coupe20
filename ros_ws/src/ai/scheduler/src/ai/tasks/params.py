#!/usr/bin/env python

# Functionalities of params :
# - default : <param type="int">78</param>
#      => if the value is not set in orderref, the param will take this value
#       (defaults can be set not fully) : <param type="pose2d">
#                                           <x>75</x>
#                                           <y>78</y>
#                                         </param>
#            => x and y can be overwritten or not in orderref, but 'theta' has
#                                                                    to be set
#
# - preset : <param type="int" preset="true">78</param>
#      => the param can't be set in orderref, this fixed value will be sent in
#                                                                  the message
#      => a preset has to be fully set in the declaration
#
# - optional : <param type="int" optional="true"/>
#      => the value doesn't have to be set in orderref, default value of ROS
#                                                                will be set

import copy
import std_msgs.msg
import geometry_msgs.msg
import memory_map.msg

import rospy


class Param(object):    # base class for parsing xml to param object
    TYPE_NAME = ""      # name used in the xml (<param type="XX" ...)
    TYPE_ROS = None     # ros message class (geometry_msgs.msg.XX)

    def __init__(self, xml=None):
        if xml is not None:             # if called from a type attr in the xml
            self.parseDefinition(xml)
            self.parseValue(xml)        # parse defaults or presets if there is

            if self.preset:
                self.checkValues()

            self.condition = xml.attrib["condition"] if "condition" in xml.attrib else "==" # used for comparing
        else:                           # if in the struct of another parser
            self.optional = False
            self.name = self.TYPE_NAME

    def getRos(self):                   # return a dict with only ros messages
        ret = copy.deepcopy(self.value)
        for k in ret:
            if isinstance(ret[k], Param):
                ret[k] = ret[k].getRos()

        return self.TYPE_ROS(**ret)

    def checkValues(self):              # check if no value is missing
        if self.optional:
            return

        for k in self.value:
            if isinstance(self.value[k]
                          , Param) and not self.value[k].optional:
                try:
                    self.value[k].checkValues()
                except KeyError as e:
                    raise KeyError("Value '{}' is needed for the parameter '{}' !"
                                   .format(k, self.name))
            elif self.value[k] is None:
                raise KeyError("A value for the parameter '{}' is needed !"
                               .format(self.name))
    def getBoundParams(self):
        b = []
        if hasattr(self, "bind") and self.bind is not None:
            b.append(self)

        for p in self.value:
            if isinstance(self.value[p], Param):
                b = b + self.value[p].getBoundParams()

        return b

    def parseBind(self, xml):
        if "bind" in xml.attrib:
            self.bind = xml.attrib["bind"]
        else:
            self.bind = None

    def parseValue(self, xml):  # parse the content, in orderref or for presets
        self.parseBind(xml)

        for child in xml:
            if child.tag in self.value:
                if isinstance(self.value[child.tag], Param):
                    self.value[child.tag].parseValue(child)
                else:
                    self.value[child.tag] = child.text
            else:
                raise KeyError("Parameter of type '{}' doesn't need a value "
                               "'{}', only {} !"
                               .format(self.TYPE_NAME, child.tag,
                                       self.value.keys()))

    def parseDefinition(self, xml):  # parse name, type, required and preset
        if "name" not in xml.attrib:
            raise KeyError("Parameters need a 'name' attribute")

        self.name = xml.attrib["name"].lower()

        self.preset = False
        if "preset" in xml.attrib and xml.attrib["preset"] == "true":
            self.preset = True

        self.optional = False
        if "optional" in xml.attrib:
            self.optional = xml.attrib["optional"].lower() == "true"

        if self.preset and self.optional:
            raise KeyError("Parameter {} cannot be preset and optional !"
                           .format(self.name))

    def compare(self, obj_ros): # only used in orders response checks
        for child in self.value: #TOBETESTED
            if not self.value[child].compare(getattr(obj_ros, child)):
                return False
        return True



def ParamCreator(xml):  # factory : give parser given the type as string
    if "type" not in xml.attrib:
        raise KeyError("Parameters definitions need a type !")

    for cls in Param.__subclasses__():
        if cls.TYPE_NAME == xml.attrib["type"]:
            return cls(xml)

    raise ValueError("No parser defined for type '{}' ! Please add a subclass"
                     "of 'Param' in the file ai_params.py".format(xml.attrib["type"]))


# Child classes - one for each type of param

# base classes : getRos return a primitive type
class BoolParser(Param):
    TYPE_NAME = "bool"
    TYPE_ROS = std_msgs.msg.Bool

    def __init__(self, xml=None):
        self.value = {
            'data': None
        }
        super(BoolParser, self).__init__(xml)

    def parseValue(self, xml):
        self.parseBind(xml)
        if xml.text:
            self.value["data"] = bool(int(xml.text))

    def getRos(self):
        return self.value["data"]

    def compare(self, obj_ros):
        if self.condition == "==":
            return self.value["data"] == bool(obj_ros)
        elif self.condition == "!=":
            return self.value["data"] != bool(obj_ros)
        else:
            rospy.logerr("Error : message response check has no '{}' condition in '{}' type.".format(self.condition, self.TYPE_NAME))

class StringParser(Param):
    TYPE_NAME = "string"
    TYPE_ROS = std_msgs.msg.String

    def __init__(self, xml=None):
        self.value = {
            'data': None
        }
        super(StringParser, self).__init__(xml)

    def parseValue(self, xml):
        self.parseBind(xml)
        if xml.text:
            self.value["data"] = xml.text

    def getRos(self):
        return self.value["data"]

    def compare(self, obj_ros):
        if self.condition == "==":
            return self.value["data"] == str(obj_ros)
        else:
            rospy.logerr("Error : message response check has no '{}' condition in '{}' type.".format(self.condition, self.TYPE_NAME))


class IntParser(Param):
    TYPE_NAME = "int"
    TYPE_ROS = std_msgs.msg.Int64

    def __init__(self, xml=None):
        self.value = {
            'data': None
        }
        super(IntParser, self).__init__(xml)

    def parseValue(self, xml):
        self.parseBind(xml)
        if xml.text:
            self.value["data"] = int(xml.text)

    def getRos(self):
        return self.value["data"]

    def compare(self, obj_ros):
        if self.condition in ["==", "!=", "<=", ">=", "<", ">"]:
            exec("result = int(obj_ros) {} self.value['data']".format(self.condition))
            return result
        else:
            rospy.logerr("Error : message response check has no '{}' condition in '{}' type.".format(self.condition, self.TYPE_NAME))


class FloatParser(Param):
    TYPE_NAME = "float"
    TYPE_ROS = std_msgs.msg.Float64

    def __init__(self, xml=None):
        self.value = {
            'data': None
        }
        super(FloatParser, self).__init__(xml)

    def parseValue(self, xml):
        self.parseBind(xml)
        if xml.text:
            self.value["data"] = float(xml.text)

    def getRos(self):
        return self.value["data"]

    def compare(self, obj_ros):
        if self.condition in ["==", "!=", "<=", ">=", "<", ">"]:
            exec("result = float(obj_ros) {} self.value['data']".format(self.condition))
            return result
        else:
            rospy.logerr("Error : message response check has no '{}' condition in '{}' type.".format(self.condition, self.TYPE_NAME))


# complex classes
# you can choose the override parseValue and getRos if you need to,
# but for the simplest, just declaring the struct of self.value should do
class Pose2DParser(Param):
    TYPE_NAME = "pose2d"
    TYPE_ROS = geometry_msgs.msg.Pose2D

    def __init__(self, xml=None):
        self.value = {
            'x': FloatParser(),
            'y': FloatParser(),
            'theta': FloatParser()
        }
        super(Pose2DParser, self).__init__(xml)


class WaypointParser(Param):
    TYPE_NAME = "waypoint"
    TYPE_ROS = memory_map.msg.Waypoint

    def __init__(self, xml=None):
        self.value = {
            'name': StringParser(),
            'frame_id': StringParser(),
            'pose': Pose2DParser()
        }
        super(WaypointParser, self).__init__(xml)
