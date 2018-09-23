#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Pose2D, TransformStamped
from ai_game_manager import StatusServices


class Localizer(object):
    def __init__(self):
        super(Localizer, self).__init__()

        rospy.init_node('localizer_node', anonymous=False)

        self._br = tf2_ros.TransformBroadcaster()

        # TODO : subscribe to all sources of info
        self._sub_asserv = rospy.Subscriber("/drivers/ard_asserv/pose2d", Pose2D,
                                          self.callback_asserv)

        self._data_asserv = None

        # Tell ai/game_manager the node initialized successfuly.
        StatusServices("recognition", "localizer").ready(True)

    def callback_asserv(self, data):
        self._data_asserv = data

    def run(self):
        rospy.loginfo("Localizer node started")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            data = self.calculate()
            if data:
                t = TransformStamped()

                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = "robot"
                t.transform.translation.x = data.x
                t.transform.translation.y = data.y
                t.transform.translation.z = 0.0
                q = tf.transformations.quaternion_from_euler(0, 0, data.theta)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self._br.sendTransform(t)

            rate.sleep()

    #  TODO : merge all data gathered
    def calculate(self):
        return self._data_asserv

if __name__ == '__main__':
    loc = Localizer()
    loc.run()
