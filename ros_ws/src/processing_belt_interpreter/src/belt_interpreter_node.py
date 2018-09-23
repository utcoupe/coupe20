#!/usr/bin/env python

import rospy
from belt_parser import BeltParser
import tf
import tf2_ros
import math
import copy

from memory_definitions.srv import GetDefinition
from processing_belt_interpreter.msg import *
from drivers_ard_others.msg import BeltRange
from geometry_msgs.msg import Pose2D, TransformStamped, PointStamped
from ai_game_manager import StatusServices
from dynamic_reconfigure.server import Server
from processing_belt_interpreter.cfg import BeltInterpreterConfig

from multiprocessing import Lock

class BeltInterpreter(object):
    def __init__(self):
        super(BeltInterpreter, self).__init__()

        rospy.init_node("belt_interpreter")

        rospy.loginfo("Belt interpreter is initializing...")

        # template for the sensor frame id, with '{}' being the sensor id
        self.SENSOR_FRAME_ID = "belt_{}"
        self.DEF_FILE = "processing/belt.xml"
        self.TOPIC = "/processing/belt_interpreter/rects"
        self.SENSORS_TOPIC = "/drivers/ard_others/belt_ranges"

        self.PUB_RATE = rospy.Rate(10)

        self.RECT_SCALE_WIDTH = 1.0
        self.RECT_SCALE_HEIGHT = 1.0

        self.WATCHDOG_PERIOD_BELT = rospy.Duration(0.015)
        self.WATCHDOG_PERIOD_TERA = rospy.Duration(0.05)

        self.PREVIOUS_DATA_SIZE = 2

        filepath = self.fetch_definition()

        self._belt_parser = BeltParser(filepath)
        self._pub = rospy.Publisher(self.TOPIC, BeltRects, queue_size=1)
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.pub_static_transforms()

        self._sensors_sub = rospy.Subscriber(self.SENSORS_TOPIC, BeltRange,
                                             self.callback)

        self.syn_param_srv = Server(BeltInterpreterConfig, self.dyn_param_cb)

        self._mutex = Lock()

        self._watchdog = rospy.Timer(self.WATCHDOG_PERIOD_TERA, self.publish, oneshot=True)

        self._current_rects = {}
        self._current_statuses = {}
        self._data_to_process = []
        self._previous_rects = []
        self._previous_statuses = []

        self._same_bad_value_counter = {s: 0 for s in self._belt_parser.Sensors.keys()}
        self._last_bad_value = {s: 0 for s in self._belt_parser.Sensors.keys()}

        rospy.loginfo("Belt interpreter is ready. Listening for sensor data on '{}'.".format(self.SENSORS_TOPIC)) # TODO duplicate log with status_services.ready()
        # Tell ai/game_manager the node initialized successfuly.
        StatusServices("processing", "belt_interpreter").ready(True)

        rospy.spin()

    def dyn_param_cb(self, config, level):
        self.RECT_SCALE_HEIGHT = config["RECT_SCALE_HEIGHT"]
        self.RECT_SCALE_WIDTH = config["RECT_SCALE_WIDTH"]
        rospy.loginfo("Set rect scale to (%f, %f)" % (self.RECT_SCALE_WIDTH, self.RECT_SCALE_HEIGHT))

        return config

    def publish(self, event):
        with self._mutex:
            if self._current_rects.keys() == ["sensor_tera1"] or not self._current_rects:
                if self._watchdog:
                    self._watchdog.shutdown()
                self._watchdog = rospy.Timer(self.WATCHDOG_PERIOD_TERA, self.publish, oneshot=True)
            if len(self._current_rects) > 0:
                self._previous_rects.append(copy.deepcopy(self._current_rects))
                self._previous_statuses.append(copy.deepcopy(self._current_statuses))

                if(len(self._previous_rects) > self.PREVIOUS_DATA_SIZE):
                    self._previous_rects.pop(0)

                if (len(self._previous_statuses) > self.PREVIOUS_DATA_SIZE):
                    self._previous_statuses.pop(0)

                self._pub.publish(self._current_rects.values())
                self._current_rects.clear()
                self._current_statuses.clear()

    def process_range(self, data):
        if data.sensor_id not in self._belt_parser.Sensors.keys():
            rospy.logerr("Received data from belt sensor '{}' but no such sensor is defined"
                         .format(data.sensor_id))
            return

        with self._mutex:

            params = self._belt_parser.Params[self._belt_parser.Sensors[data.sensor_id]["type"]]

            if data.range > params["max_range"] or data.range <= 0:
                self._current_statuses.update({data.sensor_id: False})
                if data.range == self._last_bad_value[data.sensor_id]:
                    self._same_bad_value_counter[data.sensor_id] += 1
                else:
                    self._same_bad_value_counter[data.sensor_id] = 0
                    self._last_bad_value[data.sensor_id] = data.range

                if self._same_bad_value_counter[data.sensor_id] > 100:
                    rospy.logwarn_throttle(1, "Sensor %s might be disconnected !" % data.sensor_id)

                # If we published this sensor most of the time and its bad, publish the last one we got
                l = [data.sensor_id in d and d[data.sensor_id] for d in self._previous_statuses]
                if sum(l) > math.ceil((self.PREVIOUS_DATA_SIZE + 1) / 2):
                    for d in reversed(self._previous_rects):
                        if data.sensor_id in d:
                            rospy.logdebug('Got bad data for sensor %s but publishing the last good data' % data.sensor_id)
                            r = d[data.sensor_id]
                            r.header.stamp = rospy.Time.now()
                            self._current_rects.update({data.sensor_id: d[data.sensor_id]})
                            return
                return

            self._same_bad_value_counter[data.sensor_id] = 0

            if params["scale_responsive"]:
                width = self.get_rect_width(data.range, params) * self.RECT_SCALE_WIDTH
                height = self.get_rect_height(data.range, params) * self.RECT_SCALE_HEIGHT
            else:
                width = self.get_rect_width(data.range, params)
                height = self.get_rect_height(data.range, params)

            rect = RectangleStamped()
            rect.header.frame_id = self.SENSOR_FRAME_ID.format(data.sensor_id)

            rect.header.stamp = rospy.Time.now()
            rect.x = self.get_rect_x(data.range, params)
            rect.y = 0
            rect.w = width
            rect.h = height
            rect.a = 0

            self._current_rects.update({data.sensor_id: rect})
            self._current_statuses.update({data.sensor_id: True})


    def get_rect_width(self, r, params):
        prec = r * params["precision"]
        angle = params["angle"]

        x_far = r + prec
        x_close = math.cos(angle / 2) * (r - prec)

        # called width because along x axis, but it is the smaller side
        width = abs(x_far - x_close)
        return width

    def get_rect_height(self, r, params):
        prec = r * params["precision"]
        angle = params["angle"]

        return abs(2 * math.sin(angle / 2) * (r + prec))


    def get_rect_x(self, r, params):
        prec = r * params["precision"]
        angle = params["angle"]

        x_far = r + prec
        x_close = math.cos(angle / 2) * (r - prec)
        return (x_far + x_close) / 2

    def callback(self, data):
        publish_now = False
        if data.sensor_id in self._current_rects and data.sensor_id != 'sensor_tera1':
            publish_now = True

        self.process_range(data)

        if data.sensor_id != 'sensor_tera1' and not publish_now:
            if self._watchdog:
                self._watchdog.shutdown()
            self._watchdog = rospy.Timer(self.WATCHDOG_PERIOD_BELT, self.publish, oneshot=True)
        elif publish_now:
            self.publish(None)


    def pub_static_transforms(self):
        tr_list = []
        for id, s in self._belt_parser.Sensors.items():
            tr = TransformStamped()

            tr.header.stamp = rospy.Time.now()
            tr.header.frame_id = "robot"
            tr.child_frame_id = self.SENSOR_FRAME_ID.format(id)

            tr.transform.translation.x = s["x"]
            tr.transform.translation.y = s["y"]
            tr.transform.translation.z = 0

            quat = tf.transformations.quaternion_from_euler(0, 0, s["a"])
            tr.transform.rotation.x = quat[0]
            tr.transform.rotation.y = quat[1]
            tr.transform.rotation.z = quat[2]
            tr.transform.rotation.w = quat[3]

            tr_list.append(tr)

        self._broadcaster.sendTransform(tr_list)

    def fetch_definition(self):
        get_def = rospy.ServiceProxy('/memory/definitions/get', GetDefinition)
        get_def.wait_for_service()

        try:
            res = get_def(self.DEF_FILE)

            if not res.success:
                msg = "Can't fetch belt definition file. Shutting down."
                rospy.logfatal(msg)
                raise rospy.ROSInitException(msg)
            else:
                rospy.logdebug("Belt definition file fetched.")
                return res.path

        except rospy.ServiceException as exc:
            msg = "Exception when fetching belt definition file. Shutting down.\n {}".format(str(exc))
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)


if __name__ == '__main__':
    b = BeltInterpreter()
