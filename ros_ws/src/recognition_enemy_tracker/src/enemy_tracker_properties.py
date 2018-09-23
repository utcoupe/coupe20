#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import xml.etree.ElementTree as ET
import rospy
import rospkg

_xml_path = rospkg.RosPack().get_path("recognition_enemy_tracker") + '/def/enemy_tracker_properties.xml'

class EnemyTrackerProperties():
    """Properties of enemy_tracker_node"""

    def __init__(self, xml=_xml_path):
        xml_root = ET.parse(xml).getroot()
        self.properties = {}
        for prop in xml_root.findall('prop'):
            self.properties[prop.get('name')] = prop.get('value')

    def __getitem__(self, name):
        return self.properties[name]


class RospyProperties():
    """Properties of enemy_tracker_node"""

    def __init__(self):
        pass

    def __getitem__(self, name):
        return rospy.get_param(name)

if __name__ == '__main__':
    print EnemyTrackerProperties().properties
