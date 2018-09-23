#!/usr/bin/env python

import re
import subprocess
import serial
import rospy
import rospkg
import xml.etree.ElementTree as ET
from drivers_port_finder.srv import *
from ai_game_manager import StatusServices


__author__ = "Thomas Fuhrmann"
__date__ = 18/01/2017

NODE_NAME = "port_finder"
PACKAGE_NAME = "drivers_" + NODE_NAME
ARDUINO_LIST = ("mega", "nano", "uno", "leo")
#TODO put it in xml file
ARDUINO_NODE_LIST = ("ard_asserv",)
SERIAL_READ_SIZE = 50
# Because of the ugly process with multiple different baudrate, this value has to be >= 3
ARDUINO_LOOP_MAX_TRY = 3


class PortFinder:
    def __init__(self):
        rospy.logdebug("Starting the node.")
        # List containing the content of the xml definition file
        self._components_list = []
        # List containing the connected components and the corresponding port
        self._connected_component_list = []
        # List containing the final processing information, matching the serial port and the serial component
        self._associated_port_list = []
        # List of file descriptor for calls to rosserial
        self._rosserial_call_list = []
        # Init ROS stuff
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        def_dir = rospkg.RosPack().get_path(PACKAGE_NAME) + "/def"
        self._parse_xml_file(def_dir + "/components_list.xml")
        self._parse_dmesg()
        self._associate_port()
        self._identify_arduino()
        rospy.loginfo("Port_finder ready, found : " + str(self._associated_port_list))
        self._srv_goto = rospy.Service("/drivers/" + NODE_NAME + "/get_port", GetPort, self._callback_get_port)

        # TODO find an other way ?
        # Launch urg_node directly from port_finder to have the correct port
        hokuyo_subprocess = None
        hokuyo_port = self.get_port("hokuyo")
        if hokuyo_port is not "":
            hokuyo_subprocess = subprocess.Popen(["rosrun", "urg_node", "urg_node", "_serial_port:=" + hokuyo_port, "__ns:=sensors", "scan:=/processing/lidar_objects/scan"])
            StatusServices("drivers", "port_finder").ready(True)
        else:
            rospy.logwarn("Couldn't start urg_node")
            StatusServices("drivers", "port_finder").ready(False)

        rospy.spin()
        for rosserial_fd in self._rosserial_call_list:
            rosserial_fd.terminate()
        if hokuyo_subprocess:
            hokuyo_subprocess.terminate()

    def get_port(self, device_name):
        port_name = ""
        if device_name == "all":
            port_name = str(self._associated_port_list)
        elif device_name in dict(self._associated_port_list).keys():
            port_name = dict(self._associated_port_list)[device_name]
        return port_name

    def _callback_get_port(self, request):
        response = self.get_port(request.component)
        return GetPortResponse(response)

    def _parse_xml_file(self, file):
        rospy.logdebug("Parsing port_finder definition...")
        try:
            root = ET.parse(file).getroot()
        # TODO catch a real exception instead of all of them...
        except:
            rospy.logerr("File {} not found...".format(file))
            root = None
        components = []
        if root is not None:
            for component in root.findall("component"):
                if "name" not in component.attrib:
                    rospy.logerr("Can't parse component: a 'name' attribute is required. Skipping this component.")
                    continue
                required = ["name", "vendor_id", "product_id", "port_type", "rosserialable"]
                for param in required:
                    if component.attrib[param] is None:
                        rospy.logerr("Can't parse component definition: a '{}' element is required. Skipping this component.".format(p))
                components.append({
                    "name": component.attrib["name"],
                    "vendor_id": component.attrib["vendor_id"],
                    "product_id": component.attrib["product_id"],
                    "port_type": component.attrib["port_type"],
                    "rosserialable": component.attrib["rosserialable"]
                })
            if not components:
                rospy.logwarn("No component found in component_list definition.")
            rospy.logdebug("{} components found in component_list definition".format(len(components)))
            self._components_list = components

    def _parse_dmesg(self):
        dmesg_output = self._get_dmesg().split('\n')
        id_dict = {}
        tty_dict = {}
        id_dict_filtered = {}
        merged_filtered_id_tty_list = []
        vendor_id_regex = re.compile('usb (.*):.*idVendor=([a-z0-9]+).*idProduct=([a-z0-9]+)')
        tty_regexp = re.compile(' ([0-9\-.]+):.*(ttyACM.|ttyUSB.)')
        # Parse the whole file to extract a list of lines containing idVendor and an other list containing ttyX
        for line in dmesg_output:
            vendor_id_matched = vendor_id_regex.search(line)
            tty_matched = tty_regexp.search(line)
            if vendor_id_matched is not None:
                id_dict[vendor_id_matched.group(1)] = (vendor_id_matched.group(2), vendor_id_matched.group(3))
            if tty_matched is not None:
                tty_dict[tty_matched.group(1)] = tty_matched.group(2)
        # Filter the idVendor list to keep only the idVendor we use
        for element in id_dict:
            for component in self._components_list:
                if id_dict[element][0] == component["vendor_id"] and id_dict[element][1] == component["product_id"]:
                    id_dict_filtered[element] = (id_dict[element] + (component["name"],))
        # Merge the information of vendor_id and tty port to have a single tuple in list
        for element in id_dict_filtered:
            for tty_element in tty_dict:
                if tty_element == element:
                    merged_filtered_id_tty_list.append((id_dict_filtered[element][0], id_dict_filtered[element][1], tty_dict[tty_element], id_dict_filtered[element][2]))
                    break
        self._connected_component_list = merged_filtered_id_tty_list
        rospy.logdebug("Connected components : " + str(self._connected_component_list))

    def _get_dmesg(self):
        """
        From https://gist.github.com/saghul/542780
        """
        try:
            sub = subprocess.Popen("dmesg", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = sub.communicate()
            returncode = sub.returncode
        except OSError, e:
            if e.errno == 2:
                raise RuntimeError('"dmesg" is not present on this system')
            else:
                raise
        if returncode != 0:
            raise RuntimeError('Got return value %d while executing "%s", stderr output was:\n%s' % (
            returncode, " ".join("dmesg"), stderr.rstrip("\n")))
        return stdout

    def _associate_port(self):
        for element in self._connected_component_list:
            associated_element = (element[3], "/dev/" + element[2])
            if associated_element not in self._associated_port_list:
                self._associated_port_list.append(associated_element)
            else:
                rospy.logdebug("Port_finder double found : " + str(associated_element))
        rospy.logdebug("Associated components : " + str(self._associated_port_list))

    def _identify_arduino(self):
        # Temporary list used to store identified components
        final_port_list = []
        for counter, element in enumerate(self._associated_port_list):
            if self._check_rosseriable(element[0]):
                read_data = ""
                serial_port_disconnected = False
                arduino_node_flag = False
                teraranger_flag = False
                loop_counter = 0
                try:
                    while (read_data == "") and (loop_counter < ARDUINO_LOOP_MAX_TRY):
                        # TODO find a better way than using loop_counter...
                        if loop_counter == 1 and element[0] in ("ard_nano", "teraranger"):
                            com_line = serial.Serial(element[1], 115200, timeout=2)
                        else:
                            com_line = serial.Serial(element[1], 57600, timeout=2)
                        read_data = com_line.read(SERIAL_READ_SIZE)
                        com_line.close()
                        # Received a null character, close and open again the port
                        if len(read_data) != 0 and read_data[0] == '\x00':
                            read_data = ""
                        # TODO fin a better way to do it...
                        if loop_counter == 1 and element[0] in ("ard_nano", "teraranger"):
                            # Check for the teraranger protocol header
                            if read_data.find('\x54') != -1:
                                rospy.loginfo("Found a teraranger sensor !")
                                teraranger_flag = True
                            else:
                                read_data = ""
                        rospy.logdebug("Try number {} for {}, data = {}".format(loop_counter, element[1], read_data))
                        # Always skip the first try, this is because sometimes serial data are present, sometimes not
                        # Do not remove this because it's the ugly way found to start a serial line with an other baudrate and keep a "correct" detection
                        if loop_counter == 0:
                            read_data = ""
                        # rospy.loginfo(read_data.encode('hex'))
                        loop_counter += 1
                except serial.SerialException:
                    rospy.logerr("Try to open port {} but it fails...".format(element[1]))
                    serial_port_disconnected = True
                if not serial_port_disconnected:
                    if teraranger_flag:
                        final_port_list.append(("teraranger", element[1]))
                    # Check if it's an arduino using UTCoupe protocol
                    for arduino_node in ARDUINO_NODE_LIST:
                        if read_data.find(arduino_node) != -1:
                            final_port_list.append((arduino_node, element[1]))
                            rospy.loginfo("Found a real arduino named " + arduino_node)
                            arduino_node_flag = True
                    # Otherwise, in any case, start rosserial
                    # TODO try to check with rosserial protocol (0xFF0xFE)
                    if not (arduino_node_flag or teraranger_flag):
                        rospy.loginfo("Found an arduino to start with rosserial : " + element[1] + ", start it.")
                        self._rosserial_call_list.append(subprocess.Popen(["rosrun", "rosserial_python", "serial_node.py", element[1], "__name:=serial_node_" + str(counter)]))
                        # Replace the tuple in list to keep a track that the port is used by rosserial
                        # Add an arbitrary id to rosserial to avoid having 2 components with the same name
                        final_port_list.append(("rosserial_node_" + str(counter), element[1]))
            else:
                final_port_list.append((element[0], element[1]))
        self._associated_port_list = final_port_list

    def _check_rosseriable(self, component_name):
        returned_value = False
        if component_name != "":
            for element in self._components_list:
                if element["name"] == component_name:
                    if element["rosserialable"] == "true":
                        returned_value = True
        return returned_value


if __name__ == "__main__":
    PortFinder()
