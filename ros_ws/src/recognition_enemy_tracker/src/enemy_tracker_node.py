#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

from recognition_objects_classifier.msg import ClassifiedObjects
from libtools import Rect
from enemy_tracker_tracker import EnemiesData
import enemy_tracker_properties
import rospy
from ai_game_manager import StatusServices


class EnemyTrackerNode():
    """Track enemies"""

    def __init__(self):
        self._node = rospy.init_node('enemy_tracker')
        self._namespace = '/recognition/enemy_tracker/'
        self._belt_sub = rospy.Subscriber('/recognition/objects_classifier/objects', ClassifiedObjects, self.importPoint)
        self.configure(None)
        self.rect = []
        self.data = []

        # Tell ai/game_manager the node initialized successfuly.
        StatusServices("recognition", "enemy_tracker").ready(True)

    def importPoint(self, data):
        rects = []
        for rect in data.unknown_rects:
            # TODO check referentiel
            rects.append(Rect(rect.x, rect.y, rect.w, rect.h, rect.header.stamp))
        self.saveRect(rects)
        self.trackEnemies()

    def saveRect(self, rect):
        if len(self.rect) >= self.maxRectHistory:
            del self.rect[0]
        self.rect.append(rect)

    def trackEnemies(self):
        pass

    def updateData(auto_detect_change = False):
        pass

    def configure(self, prop):
        if prop is None:
            #Default values
            prop = {'maxRectHistory': '3','maxEnemiesVelocity': '0.5'}
        self.maxRectHistory = int(prop['maxRectHistory'])
        self.maxEnemiesVelocity = float(prop['maxEnemiesVelocity'])


if __name__ == '__main__':
    enemy_tracker_node = EnemyTrackerNode()
    rospy.loginfo('Enemy tracker node started')
    #TODO Use rospy.get_param(name) instead
    enemy_tracker_node.configure(enemy_tracker_properties.EnemyTrackerProperties())
    rospy.spin()
