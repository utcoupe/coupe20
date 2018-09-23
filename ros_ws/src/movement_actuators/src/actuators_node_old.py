#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy
import time
import random
import threading
import actionlib
import movement_actuators.msg
from src.movement_actuators.src.actuators import actuators_properties
import drivers_ard_others.msg
import drivers_ax12.msg
from ai_game_manager import StatusServices
from ai_game_manager.msg import GameStatus


def current_milli_time(): return int(round(time.time() * 1000))

class ActuatorsNode():
    """Dispatch commands from AI to the correct node"""

    _result = movement_actuators.msg.dispatchResult()

    def __init__(self):
        self.is_halted = False

        rospy.init_node('actuators')
        self._namespace = '/movement/actuators/'
        self._action_name = '{}dispatch'.format(self._namespace)
        self._lock = threading.RLock()
        self._call_stack = {}
        self._action_server = actionlib.SimpleActionServer( self._action_name, movement_actuators.msg.dispatchAction, execute_cb=self.dispatch, auto_start=False)
        self._arduino_move = rospy.Publisher( '/drivers/ard_others/move', drivers_ard_others.msg.Move, queue_size=30)  # TODO check the queue_size
        self._arduino_response = rospy.Subscriber( '/drivers/ard_others/move_response', drivers_ard_others.msg.MoveResponse, self.ard_callback)
        self._ax12_client = actionlib.SimpleActionClient('/drivers/ax12', drivers_ax12.msg.Ax12CommandAction)
        self._action_server.start()


        # Tell ai/game_manager the node initialized successfuly.
        StatusServices("movement", "actuators", None, self.game_status_callback).ready(True)

    def dispatch(self, command):

        if self.is_halted:
            rospy.logerr("Received dispatch command but game_status is halted")
            self._action_server.set_aborted(False)
            return

        #-----Actuator check
        try:
            actuator = actuators_properties.getActuatorsList()[command.name]
        except KeyError as identifier:
            rospy.logwarn('The action dispatch should be called with a name instead of an id.')
            for actuator in actuators_properties.getActuatorsList().values():
                if actuator.id == command.id:
                    break
                actuator = None
            if actuator == None:
                self._action_server.set_succeeded(False)
                rospy.logerr('Couldn\'t find actuators with name "' +
                             command.name + '" or id "' + str(command.id)+'".')
                return
        #-----Param check
        param = None
        if command.param == '':
            try:
                param = actuator.preset[command.preset]
            except KeyError as identifier:
                self._action_server.set_succeeded(False)
                rospy.logerr('Couldn\'t find preset with name "' +
                             command.preset + '" in actuator "' + actuator.name + '"')
                return
        else:
            param = int(command.param) #TODO check param
        #-----Timeout check
        timeout = None
        if command.timeout<=0:
            timeout = actuator.default_timeout
        else:
            timeout = command.timeout
        #-----Time to send !
        if actuator.family.lower() == 'arduino':
            self._result.success = self.sendToArduino( actuator.id, actuator.type, command.order, param, timeout )
            self._action_server.set_succeeded(self._result)
            return
        elif actuator.family.lower() == 'ax12':
            self._result.success = self.sendToAx12( actuator.id, command.order, param, timeout)
            self._action_server.set_succeeded(self._result)
            return

        self._action_server.set_succeeded(False)

    def sendToArduino(self, ard_id, ard_type, order, param, timeout):
        msg = drivers_ard_others.msg.Move()
        msg.id = ard_id
        msg.type = {
                'digital': msg.TYPE_DIGITAL,
                'pwm': msg.TYPE_PWM,
                'servo': msg.TYPE_SERVO
            }[ard_type]
        #Mode is not implemented yet:
        #msg.mode = order
        msg.dest_value = param

        event = threading.Event()
        event.clear()
        msg.order_nb = self.generateId(event)

        self._arduino_move.publish(msg)
        ts = current_milli_time()
        event.wait(timeout / 1000.0)
        ts = current_milli_time() - ts
        if ts >= timeout:
            rospy.loginfo('Timeout reached')
        success = False
        if type(self._call_stack[msg.order_nb])==bool :
            success = self._call_stack[msg.order_nb]
        with self._lock:
            del self._call_stack[msg.order_nb]
        return success

    def sendToAx12(self, id, order, param, timeout):
        goal = drivers_ax12.msg.Ax12CommandGoal()
        goal.motor_id = int(id)
        if order.lower() == "joint":
            goal.mode = drivers_ax12.msg.Ax12CommandGoal.JOINT
            goal.speed = 0
            goal.position = int(param)
        elif order.lower() == "wheel":
            goal.mode = drivers_ax12.msg.Ax12CommandGoal.WHEEL
            goal.speed = int(param)
        else:
            rospy.logerr("Bad order: {}, expected joint or wheel".format(order))
            return False

        self._ax12_client.send_goal(goal)
        if self._ax12_client.wait_for_result(rospy.Duration(int(timeout))):
            success = self._ax12_client.get_result().success
        else:
            rospy.loginfo('Timeout reached')
            success = False

        return success
    
    def generateId(self, event):
        with self._lock:
            ard_id = random.randint(1, 10000)
            while ard_id in self._call_stack:
                ard_id = random.randint(1, 10000)
            self._call_stack[ard_id] = event
        return ard_id

    def ard_callback(self, msg):
        with self._lock:
            if msg.order_nb in self._call_stack:
                event = self._call_stack[msg.order_nb]
                self._call_stack[msg.order_nb] = msg.success
                event.set()
            else:
                rospy.logwarn('Unknow id received : {}'.format(msg.order_nb))

    def game_status_callback(self, msg):
        if not self.is_halted and msg.game_status == GameStatus.STATUS_HALT:
            self.is_halted = True
            for actuator in actuators_properties.getActuatorsList().values():
                try:
                    param = actuator.preset["OFF"]
                    self.sendToArduino( actuator.id, actuator.type, actuator.type, param, actuator.default_timeout )
                    rospy.loginfo("Turned off " + actuator.name)
                except Exception:
                    rospy.logwarn("Can't turn off an actuator!")

        elif self.is_halted and msg.game_status != GameStatus.STATUS_HALT:
            self.is_halted = False

if __name__ == '__main__':
    actuators_properties.initActuatorsList()
    actuators_node = ActuatorsNode()
    rospy.loginfo('Actuators node started')
    rospy.spin()
