#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
import tf
import tf2_ros

from geometry_msgs.msg import Pose2D, Pose, PoseStamped, Point

from ard_asserv.msg import *

from ard_asserv.srv import *

TF_SOURCE = "robot"


class AsservClient(object):
    ASSERV_NODE_NAME = "drivers/ard_asserv"
    ASSERV_GOTO_SERVICE_NAME = ASSERV_NODE_NAME + "/goto"
    ASSERV_POSE_TOPIC_NAME = ASSERV_NODE_NAME + "/pose2d"
    ASSERV_GOTOACTION_NAME = ASSERV_NODE_NAME + "/goto_action"
    ASSERV_MANAGE_SERVICE_NAME = ASSERV_NODE_NAME + "/management"
    ASSERV_EMERSTP_SERVICE_NAME = ASSERV_NODE_NAME + "/emergency_stop"

    def __init__(self):
        self._asservGotoService = ""
        self._asservGotoActionClient = ""
        self._asservManageService = ""
        self._asservEmergencyStopService = ""

        self._callbacksDoGoto = {}
        self._currentActions = {}

        self._connectToServers()

    def _connectToServers(self):
        rospy.loginfo('Waiting for "' + self.ASSERV_GOTO_SERVICE_NAME + '"...')
        rospy.wait_for_service(self.ASSERV_GOTO_SERVICE_NAME)
        rospy.loginfo("Asserv found.")

        # Goto service
        try:
            self._asservGotoService = rospy.ServiceProxy(
                self.ASSERV_GOTO_SERVICE_NAME, Goto
            )
            self._asservGotoActionClient = actionlib.ActionClient(
                self.ASSERV_GOTOACTION_NAME, DoGotoAction
            )
            self._asservGotoActionClient.wait_for_server()
            self._asservManageService = rospy.ServiceProxy(
                self.ASSERV_MANAGE_SERVICE_NAME, Management
            )
            self._asservEmergencyStopService = rospy.ServiceProxy(
                self.ASSERV_EMERSTP_SERVICE_NAME, EmergencyStop
            )
        except rospy.ServiceException as e:
            error_str = "Error when trying to connect to "
            error_str += self.ASSERV_GOTO_SERVICE_NAME
            error_str += " : " + str(e)
            rospy.logfatal(error_str)

    def cancelGoal(self, idOrder):
        if idOrder in self._currentActions:
            self._currentActions[idOrder].cancel()
        else:
            rospy.logwarn("Trying to cancel an unknown goal")

    def goto(self, pos, hasAngle):
        response = False
        mode = GotoRequest.GOTOA if hasAngle else GotoRequest.GOTO
        try:
            response = self._asservGotoService(mode=mode, position=pos).response
        except rospy.ServiceException as e:
            error_str = "Error when trying to use "
            error_str += self.ASSERV_GOTO_SERVICE_NAME
            error_str += " : " + str(e)
            rospy.logerr(error_str)
            raise Exception
        else:
            if not response:
                raise Exception("Path valid but can't reach a point.")

    def _getGoalId(self, clientDoGotoHandle):
        str_id = clientDoGotoHandle.comm_state_machine.action_goal.goal_id.id
        return str_id

    def doGoto(self, pos, direction, slowGo, hasAngle=False, callback=None):
        mode = DoGotoGoal.GOTO
        if hasAngle:
            mode = DoGotoGoal.GOTOA
        position = PoseStamped()
        position.pose = pose2dToPose(pos)
        position.header.stamp = rospy.Time.now()
        position.header.frame_id = TF_SOURCE
        goal = DoGotoGoal(
            mode=mode, position=position, direction=direction, slow_go=slowGo
        )   
        goalHandle = self._asservGotoActionClient.send_goal(
            goal, transition_cb=self._handleDoGotoResult
        )
        idAct = self._getGoalId(goalHandle)

        if callback:
            self._callbacksDoGoto[idAct] = callback

        self._currentActions[idAct] = goalHandle

        return idAct

    def _handleDoGotoResult(self, clientDoGotoHandle):
        idAct = self._getGoalId(clientDoGotoHandle)
        isDone = clientDoGotoHandle.get_comm_state() == CommState.DONE
        if (clientDoGotoHandle.get_goal_status() == GoalStatus.SUCCEEDED) and isDone:
            if idAct in self._callbacksDoGoto:
                callback = self._callbacksDoGoto[idAct]
                del self._callbacksDoGoto[idAct]
                del self._currentActions[idAct]
                raw_result = clientDoGotoHandle.get_result()
                if hasattr(raw_result, "result"):
                    rospy.logdebug("Finished: " + idAct)
                    callback(idAct, raw_result.result)
                else:
                    rospy.logwarn(
                        "No results found for "
                        + idAct
                        + ", node may be in undefined behavior"
                    )
                    rospy.logwarn(
                        "Status was " + clientDoGotoHandle.get_goal_status_text()
                    )
        elif clientDoGotoHandle.get_goal_status() in [
            GoalStatus.ABORTED,
            GoalStatus.PREEMPTED,
        ]:
            if idAct in self._callbacksDoGoto:
                rospy.logdebug("Cancelled: " + idAct)
                self._callbacksDoGoto[idAct](idAct, False)
                del self._callbacksDoGoto[idAct]
                del self._currentActions[idAct]

    def cancelAllGoals(self):
        self._asservManageService.call(mode=ManagementRequest.CLEANG)
        self._asservManageService.call(mode=ManagementRequest.KILLG)

    def stopAsserv(self):
        rospy.loginfo("stop asserv")
        self._asservEmergencyStopService.call(enable=True)
        # self._asservManageService.call(mode=ManagementRequest.CLEANG)
        # self._asservEmergencyStopService.call(enable=False)

    def resumeAsserv(self):
        rospy.loginfo("resume asserv")
        self._asservEmergencyStopService.call(enable=False)


def pose2dToPose(pose2d):
    pose = Pose()
    pose.position = Point(pose2d.x, pose2d.y, 0.0)
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, pose2d.theta)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose
