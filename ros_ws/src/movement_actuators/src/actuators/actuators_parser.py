#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import xml.etree.ElementTree as ET
import rospy
from memory_definitions.srv import GetDefinition

__author__ = "Thomas Fuhrmann", "P. Potiron"
__date__ = 11/04/2018

XML_FILE = "movement/actuators.xml"
GET_DEFINITION_SERVICE_NAME = "/memory/definitions/get"
GET_DEFINITION_SERVICE_TIMEOUT = 15


class ActuatorsProperties:
    def __init__(self, xml_actuators):
        self.id = int(xml_actuators.get('id'))
        self.name = xml_actuators.get('name')
        self.type = xml_actuators.get('type')
        self.family = xml_actuators.get('family')
        self.default_timeout = int( xml_actuators.find('timeout').get('value') )
        positions = xml_actuators.find('positions')
        self.default_position = positions.get('default')
        self.preset = {}
        for preset in positions.findall('preset'):
            self.preset[preset.get('name')] = int( preset.get('value') )


class ActuatorsParser:
    def __init__(self):
        self._actuators_dictionary = {}
        self.parse_xml_file(self.get_xml_path(XML_FILE))

    # if no actuator_name, use actuator_id
    def get_actuator_properties(self, actuator_name, actuator_id):
        if actuator_name == "" and actuator_id < 0:
            rospy.logerr("You ask actuators properties but not provide valid name or id...")
            return None
        actuator_properties = None
        if actuator_name != "":
            try:
                actuator_properties = self._actuators_dictionary[actuator_name]
            except KeyError as ex:
                rospy.logdebug("Asked for {} properties but no actuator found. Searching by id.")
        if actuator_properties is None:
            for actuator_element in self._actuators_dictionary.itervalues():
                if actuator_id == actuator_element.id:
                    actuator_properties = actuator_element
                    break
        return actuator_properties

    def get_xml_path(self, filename):
        get_def = rospy.ServiceProxy(GET_DEFINITION_SERVICE_NAME, GetDefinition)
        xml_path = ""
        try:
            get_def.wait_for_service(GET_DEFINITION_SERVICE_TIMEOUT)
            res = get_def(filename)
            if not res.success:
                rospy.logerr("Error when fetching '{}' definition file".format(filename))
            else:
                xml_path = res.path
        except rospy.exceptions.ROSException as ex:
            rospy.logerr("memory_defintion service can not be reached...")
        except rospy.ServiceException as exc:
            rospy.logerr("Unhandled error while getting def file: {}".format(str(exc)))
        return xml_path

    def parse_xml_file(self, filepath):
        if filepath != "":
            try:
                xml_root = ET.parse(filepath).getroot()
                for child in xml_root.iter('act'):
                    self._parse_xml_element(child)
            except IOError as err:
                rospy.logerr("Actuators xml file does not exists : {}".format(filepath))
        else:
            rospy.logerr("The path of file to parse is empty...")

    def _parse_xml_element(self, element):
        self._actuators_dictionary[element.get("name")] = ActuatorsProperties(element)


if __name__ == '__main__':
    ActuatorsParser()
