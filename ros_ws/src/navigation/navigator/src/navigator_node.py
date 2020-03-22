#!/usr/bin/env python
# -*-coding:Utf-8 -*

from functools import partial
import math

import rospy
import actionlib

from geometry_msgs.msg import Pose2D
from navigator.srv import Goto
from navigator.msg import Status, DoGotoResult, DoGotoGoal, DoGotoAction, DoGotoWaypointResult, DoGotoWaypointAction

from pathfinder_client import PathfinderClient
from asserv_client import AsservClient
from localizer_client import LocalizerClient
from collisions_client import CollisionsClient
from map_client import MapClient
from planning import Plan
#

from game_manager import StatusServices
from utcoupe_metrics import MetricExporter


__author__ = "Gaëtan Blond"
__date__ = 17/10/2017

NODE_NAME       = "navigator"
NODE_NAMESPACE  = "navigation"
FULL_NODE_NAME  = NODE_NAMESPACE + "/" + NODE_NAME

NB_MAX_TRY      = 7
TIME_MAX_STOP   = 1 # sec

GAUGE_NB_TRIES_NAME = "goal_nb_tries"

# Constants used for the status of goto requests
class GotoStatuses(object):
    WAITING_FOR_RESULT = 0
    SUCCESS = 1
    FAILURE = 2

# Constants used for the status topic
class NavigatorStatuses(object):
    NAV_IDLE = 0
    NAV_NAVIGATING = 1
    NAV_STOPPED = 2

class NavigatorNode(object):
    """
    The NavigatorNode class is the link between the AI, the Pathfinder and the Asserv.
    This node gets a movement order on ROS service. It can accept many at a time but is not designed to.
    The node will wait for the Pathfinder and the Asserv to advertize their services and actions before starting itself.
    """
    def __init__ (self):
        """
        Initializes the node. Does not start it.
        """

        self._actionSrv_Dogoto = ""
        self._actionSrv_doGotoWaypoint = ""
        self._statusPublisher = ""

        self._pathfinderClient = ""
        self._asservClient = ""
        self._localizerClient = ""
        self._collisionsClient = ""
        self._mapClient = ""

        self._currentStatus = NavigatorStatuses.NAV_IDLE
        self._currentPlan = ""
        self._currentGoal = ""
        self._lastStopped = rospy.Time(0)
        self._isCanceling = False
        self._idCurrentTry = 0

        self._metricExporter = MetricExporter(NODE_NAME)

    def _planResultCallback (self, result):
        self._isCanceling = False
        if result == True or self._idCurrentTry == NB_MAX_TRY or self._currentPlan.invalidStartOrEndPos:
            if (self._idCurrentTry == NB_MAX_TRY and not result) or self._currentPlan.invalidStartOrEndPos:
                rospy.logerr("Something wrong happened with our goal, aborting...")
            else:
                rospy.loginfo("Goal successful")
            self._currentStatus = NavigatorStatuses.NAV_IDLE
            self._collisionsClient.setEnabled(False, [])
            self._updateStatus()
            self._currentGoal.set_succeeded(DoGotoResult(result))
            self._metricExporter.gaugeSet(GAUGE_NB_TRIES_NAME, self._idCurrentTry)
        else:
            self._idCurrentTry += 1
            rospy.loginfo("Trying a new time to reach the goal : try number " + str(self._idCurrentTry))
            self._currentStatus = NavigatorStatuses.NAV_NAVIGATING
            self._currentPlan.replan(self._localizerClient.getLastKnownPos())

    def _handleDoGotoRequest (self, handledGoal):
        """
        Callback for the navigator's goto action request.
        The start position is the last received one from the localizer.
        If there are no path between start and end positions, it will respond with success=False.
        Else all waypoints are send to the asserv and the navigator will respond when the asserv
        will have treated all points.
        @param handledGoal: the received goal
        """
        posStart = self._localizerClient.getLastKnownPos()
        posEnd = handledGoal.get_goal().target_pos
        hasAngle = False
        if handledGoal.get_goal().mode == handledGoal.get_goal().GOTOA:
            hasAngle = True

        if handledGoal.get_goal().disable_collisions:
            rospy.loginfo("Collisions disabled")
        
        disablePathfinder = handledGoal.get_goal().disable_pathfinder
        slowGo = handledGoal.get_goal().slow_go
        tags = handledGoal.get_goal().ignore_tags
        self._executeGoto(posStart, posEnd, hasAngle, handledGoal, disablePathfinder, slowGo, tags)

    def _handleDoGotoWaypointRequest(self, handledGoal):
        rospy.logdebug("request waypoint")
        startPos = self._localizerClient.getLastKnownPos()
        endPos, hasAngle = self._mapClient.getPosFromWaypoint(handledGoal.get_goal().waypoint_name)
        if handledGoal.get_goal().mode == handledGoal.get_goal().GOTOA: # Force GOTOA if asked to
            hasAngle = True
        
        disablePathfinder = handledGoal.get_goal().disable_pathfinder
        slowGo = handledGoal.get_goal().slow_go
        tags = handledGoal.get_goal().ignore_tags
        self._executeGoto(startPos, endPos, hasAngle, handledGoal, disablePathfinder, slowGo, tags)

    def _executeGoto (self, startPos, endPos, hasAngle, handledGoal, disablePathfinder, slowGo, ignoreTags):
        self._currentStatus = NavigatorStatuses.NAV_NAVIGATING
        self._collisionsClient.setEnabled(not handledGoal.get_goal().disable_collisions, ignoreTags)
        handledGoal.set_accepted()

        if hasAngle:
            rospy.loginfo("Received a request to (" + str(endPos.x) + ", " + str(endPos.y) + ", " + str(endPos.theta) + ")")
        else:
            rospy.loginfo("Received a request to (" + str(endPos.x) + ", " + str(endPos.y) + ")")
        
        if disablePathfinder:
            rospy.loginfo("Pathfinder will NOT be used.")
        if slowGo:
            rospy.loginfo("SlowGo activated.")
        if ignoreTags:
            rospy.loginfo("Ignoring tags {}.".format(str(ignoreTags)))

        self._currentGoal = handledGoal
        self._idCurrentTry = 1
        rospy.loginfo("Try 1")
        self._currentPlan.newPlan(startPos, endPos, hasAngle, handledGoal.get_goal().direction, disablePathfinder, slowGo, ignoreTags)

    def _callbackEmergencyStop (self):
        """
        Ask the asserv to stop and update the status
        """
        if self._currentStatus != NavigatorStatuses.NAV_STOPPED:
            self._currentStatus = NavigatorStatuses.NAV_STOPPED
            self._asservClient.stopAsserv()
            self._lastStopped = rospy.Time.now()
            self._updateStatus()
        elif rospy.Time.now() - self._lastStopped > rospy.Duration(TIME_MAX_STOP) and not self._isCanceling:
            self._isCanceling = True
            rospy.loginfo("Something is blocking our way, cancelling all goals")
            self._currentPlan.cancelAsservGoals()

    def _callbackAsservResume(self):
        self._currentStatus = NavigatorStatuses.NAV_NAVIGATING
        self._isCanceling = False
        #self._collisionsClient.setEnabled(True, todo_ignoretags)
        self._asservClient.resumeAsserv()
        self._updateStatus()

    def _updateStatus (self):
        """
        Send the current status and waypoint list in the navigator's status topic.
        """
        statusMsg = Status()
        statusMsg.status = self._currentStatus
        statusMsg.currentPath = self._currentPlan.getCurrentPath()
        self._statusPublisher.publish(statusMsg)

    def startNode(self):
        """
        Start the node and the clients.
        """
        rospy.init_node (NODE_NAME, anonymous=False, log_level=rospy.INFO)
        # Create metrics
        self._metricExporter.createGauge(GAUGE_NB_TRIES_NAME)
        # Create the clients
        self._pathfinderClient = PathfinderClient()
        self._asservClient = AsservClient()
        self._localizerClient = LocalizerClient()
        self._collisionsClient = CollisionsClient(self._callbackEmergencyStop, self._callbackAsservResume)
        self._mapClient = MapClient()
        # Create action servers and topic publisher
        self._actionSrv_Dogoto = actionlib.ActionServer(FULL_NODE_NAME + "/goto_action", DoGotoAction, self._handleDoGotoRequest, auto_start=False)
        self._actionSrv_doGotoWaypoint = actionlib.ActionServer(FULL_NODE_NAME + "/gotowaypoint_action", DoGotoWaypointAction, self._handleDoGotoWaypointRequest, auto_start=False)
        self._statusPublisher = rospy.Publisher(FULL_NODE_NAME + "/status", Status, queue_size=1)
        # Launch the node
        self._actionSrv_Dogoto.start()
        self._actionSrv_doGotoWaypoint.start()
        rospy.loginfo ("Ready to navigate!")
        self._currentPlan = Plan(self._asservClient, self._pathfinderClient, self._planResultCallback, self._updateStatus)
        self._updateStatus()

        # Tell ai/game_manager the node initialized successfuly.
        StatusServices(NODE_NAMESPACE, NODE_NAME).ready(True)

        rospy.spin ()

if __name__ == "__main__":
    try:
        node = NavigatorNode ()
        node.startNode ()
    except rospy.ROSInterruptException:
        pass
