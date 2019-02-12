import rospy
import map


class Team(object):
    def __init__(self, xml):
        self.name = xml.get("name")
        self.default = bool(xml.get("default"))

        self.transforms = [t.get("type") for t in xml.findall("transform")]

    def swap(self):
        success = map.MapManager.transform(self.transforms)

        if success:
            rospy.loginfo("Swapped map to team '{}' successfully.".format(self.name))
            map.MapManager.CurrentTeam = self.name
        else: rospy.logerr("Couldn't swap map to team '{}'.".format(self.name))
        return success
