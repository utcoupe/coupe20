#!/usr/bin/env python

from definitions.srv import *
from game_manager import StatusServices
import rospy
import rospkg
import os


PACKAGE_NAME = "definitions"

def callback(req):
    def_dir = rospkg.RosPack().get_path(PACKAGE_NAME) + "/def"

    path = os.path.join(def_dir, req.request)
    if os.path.isfile(path):
        return GetDefinitionResponse(path, True)
    else:
        if rospy.has_param('/robot'):
            req_final = "robots/{}/{}".format(rospy.get_param("/robot").lower(), req.request)
            path = os.path.join(def_dir, req_final)

            if os.path.isfile(path):
                return GetDefinitionResponse(path, True)
            else:
                rospy.logerr("Request failed, file {} does not exist !".format(path))
        else:
            rospy.logerr("Parameter '/robot' not set, cannot provide the definition file {}".format(req.request))
    
    return GetDefinitionResponse("", False)


def server():
    rospy.init_node('definitions')
    s = rospy.Service('memory/definitions/get', GetDefinition, callback)
    rospy.logdebug("Definitions server ready")

    # Tell ai/game_manager the node initialized successfuly.
    StatusServices("memory", "definitions").ready(True)

    rospy.spin()


if __name__ == "__main__":
    server()
