#!/usr/bin/env python

import rospy, time
import actionlib
from actuators_abstract import ActuatorsAbstract
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D
from movement_actuators.srv import *
from recognition_cube.srv import *
from movement_actuators.msg import CubePickerAction, ArmGoal, ArmAction, CubePickerResult, DispatchAction, DispatchGoal
from geometry_msgs.msg import PointStamped

STEPPER_ELEVATOR_NAME = 'arm_elevator'
DEFAULT_ELEVATOR_TIMEOUT = 1000

class ActuatorsCubePicker(ActuatorsAbstract):
    def __init__(self):
        self._client_arm = actionlib.SimpleActionClient('/movement/actuators/arm', ArmAction)
        self._client_arm.wait_for_server()
        self._client_act = actionlib.SimpleActionClient('/movement/actuators/dispatch', DispatchAction)
        self._client_act.wait_for_server()
        #self._srv_cube_picker = rospy.Service("/movement/actuators/pick_cube", CubePicker, self._callback_pick_cube)
        ActuatorsAbstract.__init__(self,
                                   action_name='cubePicker',
                                   action_type=CubePickerAction)
        self._goal_id = None


    def _process_action(self, goal, goal_id):
        self._goal_id = goal_id
        if self._isHalted:
            rospy.logerr("CubePicker actuator is turned off!")
            return False

        # Get cube center position
        try:
            cube_p = self._get_cube_pos()
        except Exception as e:
            rospy.logerr(e)
            self._quit_action(goal_id, False)
            return False

        self._move_elevator('GO_UP')
        # Move arm to the cube center position
        self._move_arm(cube_p)

        # Elevator should move now
        try:
            # Wait for arm to finish movement
            self._client_arm.wait_for_result(rospy.Duration(2))
        except rospy.ROSException as e:
            rospy.logerr('Erreur  : ' + str(e))


        self._move_elevator('GO_DOWN')

        #rospy.loginfo('Result : ' + str(self._client_arm.get_result()))
        if self._client_arm.get_result().success == False:
            rospy.logwarn('Arm cant reach cube')
            self._quit_action(goal_id, False)
            return False

        rospy.loginfo('Arm positioned on cube')
        #Wait for elevator to finish action

        #Pick cube

        self._quit_action(goal_id, True)
        return True


    def _quit_action(self, goal_id, success):
        self._action_reached(goal_id, success, CubePickerResult(success=success))

    def _get_cube_pos(self):
        cube_srv = rospy.ServiceProxy('/recognition/localizer/recognition_cube', Cube_localizer)
        # rospy.loginfo('--------------------------- CubePicker wait for service')
        # try :
        #     rospy.wait_for_service(cube_srv, 2)
        # except rospy.ROSException :
        #     rospy.loginfo('Cant reach cube_localizer service')
        try:
            cube_p = cube_srv.call()
        except rospy.ServiceException as e:
            rospy.logerr("Cube recognition service unavailable")
            self._quit_action(self.goal_id, False)
            return False

        #rospy.loginfo(cube_p)
        if cube_p.position.x == 0 and cube_p.position.y == 0:
            raise Exception('Cant find any cube in range')
        return cube_p

    def _move_arm(self, cube_p):
        goal = ArmGoal()
        goal.frame_id = 'arm_origin' # cube_p.frame_id
        goal.x = cube_p.position.x
        goal.y = cube_p.position.y
        self._client_arm.send_goal(goal)

    def _move_elevator(self, direction):
        goal_arm_elevator = DispatchGoal()
        goal_arm_elevator.id = 0
        goal_arm_elevator.name = STEPPER_ELEVATOR_NAME
        goal_arm_elevator.preset = direction
        goal_arm_elevator.timeout = DEFAULT_ELEVATOR_TIMEOUT
        self._client_act.send_goal(goal_arm_elevator)

