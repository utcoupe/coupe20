#!/usr/bin/env python
#-*- coding:utf-8 *-*

import ctypes
import os
import protocol_parser
import math
import numpy as np
import rospy
import rospkg
import time

from asserv_real import *
from geometry_msgs.msg import Pose2D
from ard_asserv.msg import RobotSpeed
from static_map.srv import MapGetContext, MapGetContextRequest

__author__ = "Thomas Fuhrmann & milesial & Mindstan & PaulMConstant"
__date__ = 19/04/2018

# TODO maybe fetch parameters from _shared_parameters.h ?

# Rates 
SEND_POSE_RATE = 0.1  # in s
SEND_SPEED_RATE = 0.1  # in s
STM32ASSERV_RATE = 0.005  # in s # can be fetched from _shared_parameters.h

# From stm32 protocol (must be changed if change in C code):
STM32FifoMaxGoals = 20 # can be fetched from _shared_parameters.h
STM32PAUSE_BIT = ctypes.c_uint8(1) # can be fetched from _shared_parameters.h
STM32NO_GOAL = -1
STM32TYPE_PWM = 3
STM32TYPE_SPD = 4
STM32ERROR_ANGLE = 0.030 # rad

# stm32_asserv data structures - Must be changed if change in C code
class STM32Speeds(ctypes.Structure):
    _fields_ = [("pwm_left", ctypes.c_int),
                ("pwm_right", ctypes.c_int),
                ("angular_speed", ctypes.c_float),
                ("linear_speed", ctypes.c_float)]

class STM32Control(ctypes.Structure):
    _fields_ = [("speeds", STM32Speeds),
                ("max_acc", ctypes.c_float),
                ("max_spd", ctypes.c_float),
                ("rot_spd_ratio", ctypes.c_float),
                ("reset", ctypes.c_uint8),
                ("last_finished_id", ctypes.c_uint16),
                ("order_started", ctypes.c_uint16),
                ("status_bits", ctypes.c_int)]

class STM32Pos(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float),
                ("y", ctypes.c_float),
                ("angle", ctypes.c_float),
                ("modulo_angle", ctypes.c_int)]

class STM32PosData(ctypes.Structure):
    _fields_ = [("x", ctypes.c_int),
                ("y", ctypes.c_int),
                ("d", ctypes.c_int)]

class STM32AngData(ctypes.Structure):
    _fields_ = [("angle", ctypes.c_float),
                ("modulo", ctypes.c_int)]

class STM32PwmData(ctypes.Structure):
    _fields_ = [("time", ctypes.c_float),
                ("pwm_l", ctypes.c_int),
                ("pwm_r", ctypes.c_int),
                ("auto_stop", ctypes.c_int)]

class STM32SpdData(ctypes.Structure):
    _fields_ = [("time", ctypes.c_float),
                ("lin", ctypes.c_int),
                ("ang", ctypes.c_int),
                ("auto_stop", ctypes.c_int)]

class STM32GoalData(ctypes.Union):
    _fields_= [("pos_data", STM32PosData),
               ("ang_data", STM32AngData),
               ("pwm_data", STM32PwmData),
               ("spd_data", STM32SpdData)]

class STM32Goal(ctypes.Structure):
    _fields_ = [("data", STM32GoalData),
                ("type", ctypes.c_int),
                ("ID", ctypes.c_uint16),
                ("is_reached", ctypes.c_int)]

class STM32Fifo(ctypes.Structure):
    _fields_ = [("fifo", STM32Goal*STM32FifoMaxGoals),
                ("nb_goals", ctypes.c_int),
                ("current_goal", ctypes.c_int),
                ("last_goal", ctypes.c_int)]

class AsservSimu(AsservReal):
    """
    Simulation class which calls the functions exported from the stm32_asserv folder.
    Updates the (x, y, theta) position values based on the calculated ideal speed.
    """
    # Begin overloaded functions

    def _start_serial_com_line(self, port):
        """pass : no serial line in simu"""
        pass

    def _data_receiver(self):
        """pass : no data to receive from non-existent arduino"""
        pass

    def _process_received_data(self, data):
        """pass : no data will be received from non-existent arduino"""
        pass

    def _send_serial_data(self, order_type, args_list):
        args_list.insert(0, str(self._goal_counter.id))
        self._goal_counter.id += 1
        self._sending_queue.put(order_type + ";" + ";".join(args_list) + ";\n")

    def _callback_timer_serial_send(self, event):
        if not self._sending_queue.empty():
            data_to_send = self._sending_queue.get()
            rospy.logdebug("Sending data : " + data_to_send)
            self._simu_protocol_parse(data_to_send)
            self._sending_queue.task_done()
    
    # End overloaded functions

    def __init__(self, asserv_node, goal_counter):
        AsservReal.__init__(self, asserv_node, goal_counter, 0)

        # FIXME hardcoded path
        lib_dir = os.environ['UTCOUPE_WORKSPACE'] + '/ros_ws/devel/lib/'

        # STM32 lib stuff
        try:
            print('looking in ' + lib_dir)
            self._stm32lib = ctypes.cdll.LoadLibrary(
                lib_dir + 'libutcoupe_shared_asserv.so'
                )
        except OSError:
            rospy.logerr("Cannot open shared asserv library. Please generate it using catkin_make.")
            rospy.logerr("Exiting simu asserv.")
            exit()

        self._stm32lib.ControlLogicInit()
        self._stm32lib.RobotStateLogicInit()

        self._stm32control = STM32Control.in_dll(self._stm32lib, "control")
        self._stm32fifo = STM32Fifo.in_dll(self._stm32lib, "fifo")
        self._stm32pos = STM32Pos.in_dll(self._stm32lib, "current_pos")

        self._orders_dictionary_reverse = {
            v : k for k, v in self._orders_dictionary.iteritems()}
        self._last_stm32fifo_current_goal = 0
        self._auto_stop_triggered = False

        self._robot_size = 0.25 #m
        # Get map 
        self._map_x = 3.0
        self._map_y = 2.0

        context = self.getMapContext()
        if context:
            self._map_x = context.terrain_shape.width
            self._map_y = context.terrain_shape.height
        else:
            rospy.logwarn("Can't get map context, taking default values.")

        self._sending_queue.put(self._orders_dictionary['START'] + ";0;\n")

        # ROS stuff
        self._tmr_asserv_loop = rospy.Timer(rospy.Duration(STM32ASSERV_RATE), 
            self._asserv_loop)
        self._tmr_speed_send = rospy.Timer(rospy.Duration(SEND_SPEED_RATE), 
            self._callback_timer_speed_send)
        self._tmr_pose_send = rospy.Timer(rospy.Duration(SEND_POSE_RATE), 
            self._callback_timer_pose_send)

    def _simu_protocol_parse(self, data):
        """
        Python implementation of the stm32 protocol.
        Calls c functions exported from the stm32_asserv folder.
        """

        order_char = data[0]        
        index = 2
        order_id = ""
        while data[index] != ";":
            order_id += data[index]
            index += 1

        # Move to first parameter of the order
        data = data[index+1:]

        c_order_id = ctypes.c_int(int(order_id))
        c_data = ctypes.c_char_p(data)

        # Switch case like in parseAndExecuteOrder
        order_type = self._orders_dictionary_reverse[order_char]

        if order_type == "START":
            self._stm32lib.start()

        elif order_type == "HALT":
            self._stm32lib.halt()

        elif order_type == "PINGPING":
            rospy.loginfo("PING")

        elif order_type == "GET_CODER":
            rospy.logerr("GET_CODER cannot be "
                            "implemented in simu...")

        elif order_type == "GOTO":
            self._stm32lib.parseGOTO(c_data, c_order_id)
        
        elif order_type == "GOTOA":
            self._stm32lib.parseGOTOA(c_data, c_order_id)

        elif order_type == "ROT":
            self._stm32lib.parseROT(c_data, c_order_id)

        elif order_type == "ROTNMODULO":
            self._stm32lib.parseROTNMODULO(c_data, c_order_id)

        elif order_type == "PWM":
            rospy.logerr("PWM should not be used in simu. "
                "Please use speed instead.")
            #self._stm32lib.parsePWM(c_data, c_order_id)

        elif order_type == "SPD":
            self._stm32lib.parseSPD(c_data, c_order_id)

        elif order_type == "PIDALL":
            self._stm32lib.parsePIDALL(c_data)

        elif order_type == "PIDRIGHT":
            self._stm32lib.parsePIDRIGHT(c_data)

        elif order_type == "PIDLEFT":
            self._stm32lib.parsePIDLEFT(c_data)

        elif order_type == "KILLG":
            self._stm32lib.FifoNextGoal()
            self._stm32lib.ControlPrepareNewGoal()

        elif order_type == "CLEANG":
            self._stm32lib.FifoInit()
            self._stm32lib.ControlPrepareNewGoal()

        elif order_type == "RESET_ID":
            self._stm32lib.resetID()

        elif order_type == "SET_POS":
            self._stm32lib.parseSETPOS(c_data)

        elif order_type == "GET_POS":
            rospy.loginfo("x = " + str(self._stm32pos.x))
            rospy.loginfo("y = " + str(self._stm32pos.y))
            rospy.loginfo("a = " + str(self._stm32pos.angle))

        elif order_type == "GET_SPD":
            rospy.logerr("GET_SPD is not implemented in simu : "
                "SPD == TARGET_SPD")

        elif order_type == "GET_TARGET_SPD":
            rospy.loginfo("linear speed : " + 
                str(self._stm32control.speeds.linear_speed))
            rospy.loginfo("angular speed : " + 
                str(self._stm32control.speeds.angular_speed))

            rospy.loginfo("left wheel speed : " 
                + str(self._stm32control.speeds.linear_speed
                 - self._stm32control.speeds.angular_speed))
            rospy.loginfo("right wheel speed : "
                + str(self._stm32control.speeds.linear_speed
                 + self._stm32control.speeds.angular_speed))

        elif order_type == "GET_POS_ID":
            rospy.loginfo("x = " + str(self._stm32pos.x))
            rospy.loginfo("y = " + str(self._stm32pos.y))
            rospy.loginfo("a = " + str(self._stm32pos.angle))
            rospy.loginfo("last ID : " + str(self._stm32control.last_finished_id))

        elif order_type == "SPDMAX":
            self._stm32lib.parseSPDMAX(c_data)

        elif order_type == "ACCMAX":
            self._stm32lib.parseACCMAX(c_data)

        elif order_type == "GET_LAST_ID":
            rospy.loginfo("last ID : " + str(self._stm32control.last_finished_id))

        elif order_type == "PAUSE":
            self._stm32lib.ControlSetStop(STM32PAUSE_BIT)

        elif order_type == "RESUME":
            self._stm32lib.ControlUnsetStop(STM32PAUSE_BIT)

        elif order_type == "WHOAMI":
            rospy.logerr("WHOAMI cannot be implemented in simu...")

        elif order_type == "SETEMERGENCYSTOP":
            self._stm32lib.parseSETEMERGENCYSTOP(c_data)
        
        else:
            rospy.logerr("Order " + order_char + " is wrong !")

    def getMapContext(self):
        dest = "/static_map/get_context"
        try: # Handle a timeout in case one node doesn't respond
            server_wait_timeout = 2
            rospy.logdebug("Waiting for service %s for %d seconds"
                                 % (dest, server_wait_timeout))
            rospy.wait_for_service(dest, timeout=server_wait_timeout)
        except rospy.ROSException:
            return False

        rospy.loginfo("Sending service request to '{}'...".format(dest))
        service = rospy.ServiceProxy(dest, MapGetContext)
        response = service(MapGetContextRequest()) # rospy can't handle timeout, solution ?

        if response is not None:
            return response
        else:
            rospy.logerr("Service call response from '{}' is null.".format(dest))
        return False

    def start(self):
        rospy.logdebug("[ASSERV] Node has correctly started in simulation mode.")
        rospy.spin()

    def _STM32FifoCurrentGoal(self):
        return self._stm32fifo.fifo[self._stm32fifo.current_goal 
                    % STM32FifoMaxGoals]

    def _asserv_loop(self, event):
        """
        Main function of the asserv simu.
        Checks for new goals and updates behavior every STM32ASSERV_RATE ms.
        """
        now_micros = int(time.time() * 1000000)

        if self._auto_stop_triggered:
            # consider all goals reached
            while self._STM32FifoCurrentGoal().type != STM32NO_GOAL:
                self._stm32lib.setCurrentGoalReached()
                self._node.goal_reached(
                    self._stm32control.last_finished_id, True)
            self._auto_stop_triggered = False

        self._stm32lib.processCurrentGoal(ctypes.c_long(now_micros))       
        self._update_robot_pose()

        if self._STM32FifoCurrentGoal().is_reached:
            self._stm32lib.setCurrentGoalReached()
            self._node.goal_reached(
                self._stm32control.last_finished_id, True)

    def _update_robot_pose(self):
        """
        Uses the angular and linear speeds to update the robot Pose(x, y, theta).
        """    
        new_theta = (self._stm32pos.angle +
            self._stm32control.speeds.angular_speed / 1000
            * STM32ASSERV_RATE)
        new_theta %= 2 * math.pi

        new_x = (self._stm32pos.x +
                 self._stm32control.speeds.linear_speed *
                 STM32ASSERV_RATE * math.cos(new_theta))
        new_y = (self._stm32pos.y + self._stm32control.speeds.linear_speed *
                STM32ASSERV_RATE * math.sin(new_theta))

        # Table border logic
        max_x = (self._map_x - self._robot_size/2) * 1000
        max_y = (self._map_y - self._robot_size/2) * 1000
        min_xy = self._robot_size/2 * 1000 #mm
        
        goal = self._STM32FifoCurrentGoal()
        pos = self._stm32pos
        linear_speed = self._stm32control.speeds.linear_speed

        push_against_wall = 0

        if new_x > max_x:
            push_against_wall = new_x - max_x
            new_x = max_x
            new_y = pos.y
            if linear_speed > 0 :
                wall_angle = 0
            else:
                wall_angle = math.pi

        elif new_x < min_xy:
            push_against_wall = min_xy - new_x
            new_x = min_xy
            new_y = pos.y
            if linear_speed > 0 :
                wall_angle = math.pi
            else:
                wall_angle = 0

        if new_y > max_y:
            push_against_wall = new_y - max_y
            new_y = max_y
            new_x = pos.x
            if linear_speed > 0 :
                wall_angle = math.pi/2
            else:
                wall_angle = -math.pi/2

        elif new_y < min_xy:
            push_against_wall = min_xy - new_y
            new_y = min_xy
            new_x = pos.x
            if linear_speed > 0 :
                wall_angle = -math.pi/2
            else:
                wall_angle = math.pi/2
        
        if push_against_wall:
            if (abs(new_theta - np.sign(new_theta)*wall_angle)
                    < STM32ERROR_ANGLE or 
                abs(new_theta - np.sign(new_theta)*wall_angle - 
                    np.sign(new_theta)*2*math.pi) < STM32ERROR_ANGLE):

                new_theta = wall_angle
            else:
                if abs(new_theta) > math.pi:
                    new_theta -= np.sign(new_theta)*2*math.pi
                if new_theta > wall_angle:
                    new_theta -= push_against_wall / 300
                else:
                    new_theta += push_against_wall / 300
                
        # Auto stop check
        check_auto_stop = (goal.type == STM32TYPE_SPD and 
                            goal.data.spd_data.auto_stop)
        if (check_auto_stop and
              new_x == pos.x and new_y == pos.y and
              abs(new_theta - pos.angle) < STM32ERROR_ANGLE/100):
            self._auto_stop_triggered = True

        self._stm32lib.RobotStateSetPos(ctypes.c_float(new_x), 
                                        ctypes.c_float(new_y), 
                                        ctypes.c_float(new_theta))

    def _callback_timer_pose_send(self, event):
        self._robot_raw_position = Pose2D(self._stm32pos.x / 1000, 
                                          self._stm32pos.y / 1000, 
                                          self._stm32pos.angle)
        self._node.send_robot_position(self._robot_raw_position)

    def _callback_timer_speed_send(self, event):
        self._node.send_robot_speed(
            RobotSpeed(0, 0, self._stm32control.speeds.linear_speed,
                       0, 0))