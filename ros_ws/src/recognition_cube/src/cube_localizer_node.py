#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import math
from geometry_msgs.msg import Pose2D, TransformStamped
from ai_game_manager import StatusServices
from processing_lidar_objects.msg import Obstacles
from recognition_cube.srv import *
from recognition_objects_classifier.msg import *

class Localizer(object):
    def __init__(self):
        super(Localizer, self).__init__()

        rospy.init_node('cube_localizer_node', anonymous=False)

        self._br = tf2_ros.TransformBroadcaster()

        # TODO : subscribe to all sources of info
        self._sub_asserv = rospy.Subscriber("/drivers/ard_asserv/pose2d", Pose2D,
                                          self.callback_asserv)
        self._lidar_sub = rospy.Subscriber("/processing/lidar_objects/raw_cube", Obstacles, self.lidar_callback)
        self._srv_lidar_pos = rospy.Service("/recognition/localizer/recognition_cube", Cube_localizer, self.callback_lidar_pos)
        self._data_asserv = None

        self._segments = {'map': [], 'unknown': []}
        self._to_process = {'rects': [], 'circles': [], 'segments': []}

        StatusServices("recognition", "cube_localizer_node").ready(True)

    def lidar_callback(self, data):
        self._to_process['segments'] = []
        self._to_process['segments'] = [SegmentObstacleStamped(data.header, s) for s in data.segments]

    def callback_asserv(self, data):
        self._data_asserv = data

    def clear_objects(self):
        self._segments['map'] = []
        self._segments['unknown'] = []

    def callback_lidar_pos(self, info):
        return self.process_segments()


    def process_segments(self):
        min = 2000
        closest = None
        for segment in self._to_process['segments']:
            seg = segment_properties(segment)
            #rospy.loginfo('center_y : ' + str(seg.center_y) + ' norm : ' + str(seg.norm))
            if seg.distance < min and seg.distance < 0.4 and seg.norm > 0.015:
                min = seg.distance
                closest = seg

        if closest == None:
            return ('laser', Pose2D(float(0), float(0), 0))

        closest = self.calculate_cube_center(closest)
        # rospy.loginfo(
        #     'Segment le plus proche: ' + str(min) + ' X : ' + str(closest.center_x) + ' Y : ' + str(closest.center_y)
        #     + '\nFpoint X : ' + str(closest.segment.first_point.x) + ' Y : ' + str(closest.segment.first_point.y)
        #     + '\nLpoint X : ' + str(closest.segment.last_point.x) + ' Y : ' + str(closest.segment.last_point.y)
        #     + '\nCube X : ' + str(closest.orth_x) + ' Y : ' + str(closest.orth_y))
        #ret = str(min)
        pos = Pose2D(float(closest.orth_x), float(closest.orth_y), 0)
        return ('laser', pos)

    def run(self):
        rospy.loginfo("Cube localizer node started")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()

    #  TODO : merge all data gathered
    def calculate(self):
        return self._data_asserv
    
    def calculate_cube_center(self, seg):
        d = 0.03
        vect_orth_x = (-seg.y)/seg.norm
        vect_orth_y = seg.x/seg.norm
        if (seg.center_x*vect_orth_x + seg.center_y*vect_orth_y) < 0:
            vect_orth_x = -vect_orth_x
            vect_orth_y = -vect_orth_y
        
        seg.orth_x = seg.center_x + d*vect_orth_x
        seg.orth_y = seg.center_y + d*vect_orth_y
        return seg
        
class segment_properties:
    def __init__(self, seg):
        segment = seg.segment
        self.segment = seg.segment
        self.x = segment.first_point.x - segment.last_point.x
        self.y = segment.first_point.y - segment.last_point.y
        self.vect = [self.x, self.y]
        self.norm = math.sqrt(self.x * self.x + self.y * self.y)
        self.center_x = (segment.first_point.x + segment.last_point.x) / 2
        self.center_y = (segment.first_point.y + segment.last_point.y) / 2
        self.distance = math.sqrt(self.center_x * self.center_x + self.center_y * self.center_y)
        self.orth_x = None
        self.orth_y = None

if __name__ == '__main__':
    loc = Localizer()
    loc.run()
