import rospy
from map_bases import DictManager
import map


class Team(object):
    def __init__(self, name, initdict):
        self.name = name
        self.default = bool(initdict["default"])

        self.transforms = []
        if "transforms" in initdict:
            self.transforms = [t for t in initdict["transforms"]]

    def swap(self):
        success = map.Map.transform(self.transforms)

        if success:
            rospy.loginfo("Swapped map to team '{}' successfully.".format(self.name))
            map.Map.CurrentTeam = self.name
        else: rospy.logerr("Couldn't swap map to team '{}'.".format(self.name))
        return success
