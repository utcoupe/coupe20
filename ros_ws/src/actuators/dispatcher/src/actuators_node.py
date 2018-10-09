#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy
import actuators
from ai_game_manager import StatusServices
from ai_game_manager.msg import GameStatus

__author__ = "P. Potiron", "Thomas Fuhrmann"
__date__ = 9/04/2018

NODE_NAME = "actuators"


class ActuatorsNode:
    def __init__(self):
        rospy.init_node(NODE_NAME, log_level=rospy.INFO)
        self.dispatch_instance = actuators.ActuatorsDispatch()

        self._robot = rospy.get_param('/robot')
        if self._robot.lower() == "gr":
            self.arm_instance = actuators.ActuatorsArm()
            self.cubePicker_instance = actuators.ActuatorsCubePicker()
        elif self._robot.lower() == "pr":
            self.barrel_instance = actuators.ActuatorsBarrel()
            self.canon_instance = actuators.ActuatorsCanon()

        self._isHalted = False

        rospy.loginfo("Movement actuators has correctly started.")
        StatusServices("movement", "actuators", None, self._on_gameStatus).ready(True)
        rospy.spin()
    
    def _on_gameStatus(self, msg):
        stateChanged = False
        if not self._isHalted and msg.game_status == GameStatus.STATUS_HALT:
            self._isHalted = True
            stateChanged = True
        elif self._isHalted and msg.game_status != GameStatus.STATUS_HALT:
            self._isHalted = False
            stateChanged = True
        
        if stateChanged:
            self.dispatch_instance.setHalted(self._isHalted)
            if self._robot.lower() == "gr":
                self.arm_instance.setHalted(self._isHalted)
            elif self._robot.lower() == "pr":
                self.barrel_instance.setHalted(self._isHalted)
                self.canon_instance.set_halted(self._isHalted)


if __name__ == '__main__':
    ActuatorsNode()
