#!/usr/bin/env python

import rospy
import threading
from functools import partial
from actuators_abstract import *
import actuators_parser
from movement_actuators.msg import DispatchAction, DispatchResult
from drivers_ax12.msg import Ax12CommandGoal, Ax12CommandAction
from actionlib_msgs.msg import GoalStatus
import drivers_ard_others.msg
import drivers_ax12.msg

__author__ = "Thomas Fuhrmann"
__date__ = 9/04/2018


class ActuatorsDispatch(ActuatorsAbstract):
    def __init__(self):
        ActuatorsAbstract.__init__(self, "dispatch", DispatchAction)
        self._lock = threading.RLock()
        # goal_id, order_id for arduino and goal for ax12
        self._active_goals = {}
        # goal_id and timer
        self._active_goal_timers = {}
        # This counter enables to keep a track of orders sent to arduino
        self._order_id_counter = 0
        self._act_parser = actuators_parser.ActuatorsParser()
        self._pub_ard_move = rospy.Publisher('/drivers/ard_others/move', drivers_ard_others.msg.Move, queue_size=3)
        self._sub_ard_response = rospy.Subscriber('/drivers/ard_others/move_response', drivers_ard_others.msg.MoveResponse, self._callback_move_response)
        self._act_cli_ax12 = actionlib.ActionClient('/drivers/ax12', Ax12CommandAction)
        self._act_cli_ax12.wait_for_server(rospy.Duration(10))
        self._isHalted = False

    def _process_action(self, goal, goal_id):
        if self._isHalted:
            rospy.logerr("Actuators are turned off!")
            return False
        to_return = False
        actuator_properties = self._act_parser.get_actuator_properties(goal.name, goal.id)
        if actuator_properties is None:
            return False
        param = goal.param
        if param == "":
            if goal.preset != "":
                param = actuator_properties.preset[goal.preset]
            else:
                rospy.logwarn("Received goal ({}) as no param nor preset, reject the goal.".format(goal_id))
                return False
        if actuator_properties.family.lower() == 'arduino':
            rospy.loginfo("Received an arduino goal")
            result = self._send_to_arduino(actuator_properties.id, actuator_properties.type, param)
            if result >= 0:
                with self._lock:
                    self._active_goals[goal_id] = result
                to_return = True
        elif actuator_properties.family.lower() == 'ax12':
            rospy.loginfo("Received an ax12 goal")
            goal_handle = self._send_to_ax12(actuator_properties.id, goal.order, param)
            if goal_handle is not None:
                with self._lock:
                    self._active_goals[goal_id] = goal_handle
                to_return = True
            else:
                to_return = False
        if to_return:
            timeout = goal.timeout
            if goal.timeout == 0:
                timeout = actuator_properties.default_timeout
                rospy.logwarn("No timeout given to dispatch goal, use default xml value %d ms" % timeout)
                if timeout == 0:
                    rospy.logerr("Goal {} has no timeout nor default timeout, reject it.".format(goal_id))
                    return False
            self._active_goal_timers[goal_id] = rospy.Timer(period=rospy.Duration(timeout / 1000.0),
                                                            callback=partial(self._timer_callback_timeout, goal_id=goal_id),
                                                            oneshot=True)
        return to_return

    def _callback_move_response(self, msg):
        if msg.order_nb in filter(lambda e: isinstance(e, int), self._active_goals.values()):
            received_goal_id = None
            for goal_id in self._active_goals.iterkeys():
                if isinstance(self._active_goals[goal_id], int) and self._active_goals[goal_id] == msg.order_nb:
                    received_goal_id = goal_id
            if received_goal_id is not None:
                self._goal_reached(received_goal_id, msg.success)
        else:
            rospy.logwarn('Unknow id received : {}'.format(msg.order_nb))

    def _callback_ax12_client(self, goal_handle):
        received_goal_id = goal_handle.comm_state_machine.action_goal.goal_id.id
        if goal_handle in filter(lambda e: not isinstance(e, int), self._active_goals.values()):
            for goal_id, goal in self._active_goals.iteritems():
                if not isinstance(goal, int) and goal.comm_state_machine.action_goal.goal_id.id == received_goal_id:
                    goal_status = goal_handle.get_goal_status()

                    if goal_status == GoalStatus.SUCCEEDED:
                        success = True
                    elif goal_status in [GoalStatus.LOST,
                                  GoalStatus.RECALLED,
                                  GoalStatus.REJECTED,
                                  GoalStatus.ABORTED,
                                  GoalStatus.PREEMPTED]:
                        success = False
                    else:
                        return

                    self._goal_reached(goal_id, success)
                    return
        else:
            rospy.logwarn("Unknow ax12 goal received : {}".format(received_goal_id))

    def _send_to_arduino(self, ard_id, ard_type, param):
        msg = drivers_ard_others.msg.Move()
        msg.id = ard_id
        msg.type = {
                'digital': msg.TYPE_DIGITAL,
                'pwm': msg.TYPE_PWM,
                'servo': msg.TYPE_SERVO,
                'stepper': msg.TYPE_STEPPER
            }[ard_type]
        msg.dest_value = int(param)
        msg.order_nb = self._get_order_id()
        self._pub_ard_move.publish(msg)
        return msg.order_nb

    def _send_to_ax12(self, motor_id, order, param):
        try:
            goal_position = int(param)
        except ValueError as ex:
            rospy.logerr("Try to dispatch an ax12 order with invalid position... Reject it.")
            return None
        goal = drivers_ax12.msg.Ax12CommandGoal()
        goal.motor_id = int(motor_id)
        if order.lower() == "joint":
            goal.mode = drivers_ax12.msg.Ax12CommandGoal.JOINT
            goal.speed = 0
            goal.position = goal_position
        elif order.lower() == "wheel":
            goal.mode = drivers_ax12.msg.Ax12CommandGoal.WHEEL
            goal.speed = goal_position
        else:
            rospy.logerr("Bad order: {}, expected joint or wheel".format(order))
            return None
        return self._act_cli_ax12.send_goal(goal, transition_cb=self._callback_ax12_client)

    def _get_order_id(self):
        with self._lock:
            current_id = self._order_id_counter
            self._order_id_counter += 1
        return current_id

    def _timer_callback_timeout(self, event, goal_id):
        rospy.logerr('Timeout triggered for goal %s, cancelling' % goal_id.id)
        self._goal_reached(goal_id, False)

    def _goal_reached(self, goal_id, reached):
        if goal_id in self._active_goals.keys():
            with self._lock:
                # Remove goal from active goals
                del self._active_goals[goal_id]
                # Remove goal timeout
                if goal_id in self._active_goal_timers:
                    if self._active_goal_timers[goal_id]:
                        self._active_goal_timers[goal_id].shutdown()
                    del self._active_goal_timers[goal_id]
            self._action_reached(goal_id, reached, DispatchResult(reached))

    def setHalted(self, isHalted):
        self._isHalted = isHalted
        if isHalted:
            for actuator in self._act_parser._actuators_dictionary.values():
                if "OFF" in actuator.preset.keys():
                    if actuator.family == "arduino":
                        self._send_to_arduino(actuator.id, actuator.type, actuator.preset["OFF"])
                        continue
                    # TODO stop AX12 ?
