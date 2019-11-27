#!/usr/bin/env python

import math
import numpy as np
import rospy
from asserv_abstract import *
from geometry_msgs.msg import Pose2D
from ard_asserv.msg import RobotSpeed
from time import sleep
from static_map.srv import MapGetContext, MapGetContextRequest

__author__ = "Thomas Fuhrmann & milesial & Mindstan & PaulMConstant"
__date__ = 19/04/2018

# Rates
SEND_POSE_RATE = 0.1  # in s
SEND_SPEED_RATE = 0.1  # in s
ASSERV_RATE = 0.05  # in s

# Metric constants
ASSERV_ERROR_POSITION = 0.03  # in meters
ASSERV_ERROR_ANGLE = 0.05  # in radians
ASSERV_MINIMAL_SPEED = 0.05  # in m/s
ASSERV_MAX_SPEED = 0.4 # m/s
SLOW_GO_MAX_SPEED = 0.25 # m/s

# Control classes

class AngularPID():
    P = 0.5
    D = 1
    I = 0.005
    MAX_I = 5
    last_error = 0
    error_sum = 0

class LinearPD():
    # No integral here because we don't want to overshoot
    P = 0.8
    D = 10
    last_error = 0

    ANGLE_SPEED_DIVIDER = 40 # Coefficient to reduce linear speed proportionally to angle error


# Goals
class AsservGoal():
    def __init__(self, goal_id, pose, has_angle, direction, slow_go):
        self.goal_id = goal_id
        self.pose = pose
        self.has_angle = has_angle
        self.direction = direction
        self.slow_go = slow_go

class PwmGoal():
    def __init__(self, goal_id, right_speed, left_speed, duration, auto_stop):
        self.goal_id = goal_id
        self.left_speed = left_speed
        self.right_speed = right_speed
        self.duration = duration
        self.auto_stop = auto_stop
        
class AsservSetPosModes():
    AXY = 0
    A = 1
    Y = 2
    AY = 3
    X = 4
    AX = 5
    XY = 6

class AsservSimu(AsservAbstract):
    def __init__(self, asserv_node):
        
        AsservAbstract.__init__(self, asserv_node)
        # Asserv management stuff
        # The pose is in meters and rad
        self._current_pose = Pose2D(0.18, 1, math.pi/2)
        self._left_wheel_speed = 0 # m/s
        self._right_wheel_speed = 0 # m/s

        self._current_linear_speed = 0 # m/s
        self._current_angular_speed = 0 # m/s
        self._emergency_stop = False

        self._current_goal = None

        # List of Pose2d corresponding to the goals
        self._goals_list = []

        # Parameters
        self._max_acceleration = 0.5 # m/s^2
        self._max_linear_speed = ASSERV_MAX_SPEED # m/s
        self._max_angular_speed = ASSERV_MAX_SPEED

        # ROS stuff
        self._tmr_pose_send = rospy.Timer(rospy.Duration(SEND_POSE_RATE), self._callback_timer_pose_send)
        self._tmr_speed_send = rospy.Timer(rospy.Duration(SEND_SPEED_RATE), self._callback_timer_speed_send)
        self._tmr_asserv_computation = rospy.Timer(rospy.Duration(ASSERV_RATE), self._callback_timer_asserv_computation)

        # Get map & robot size 
        self._map_x = 3.0
        self._map_y = 2.0
        self._robot_wheelbase = 0.2

        context = self.getMapContext()
        if context:
            self._map_x = context.terrain_shape.width
            self._map_y = context.terrain_shape.height
            self._robot_wheelbase = context.robot_shape.wheelbase

        else:
            rospy.logwarn("Can't get map context, taking default values.")

        
        self._robot_up_size = 0.23
        self._robot_down_size = 0.058

    def getMapContext(self):
        dest = "/static_map/get_context"
        try: # Handle a timeout in case one node doesn't respond
            server_wait_timeout = 2
            rospy.logdebug("Waiting for service %s for %d seconds" % (dest, server_wait_timeout))
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

    def goto(self, goal_id, x, y, direction=1, slow_go=False):
        rospy.loginfo("[ASSERV] Accepting goal (x = " + str(x) + ", y = " + str(y) + ").")
        has_angle = False
        self._start_trajectory(goal_id, x, y, 0, direction, has_angle, slow_go)
        return True

    def gotoa(self, goal_id, x, y, a, direction=1, slow_go=False):
        rospy.loginfo("[ASSERV] Accepting goal (x = " + str(x) + ", y = " + str(y) + ", a = " + str(a) + ").")
        has_angle = True
        self._start_trajectory(goal_id, x, y, a, direction, has_angle, slow_go)
        return True

    def rot(self, goal_id, a, no_modulo, slow_go):
        rospy.loginfo("[ASSERV] Accepting goal (a = " + str(a) + ").")
        has_angle = True
        direction = 1
        self._start_trajectory(goal_id, self._current_pose.x, self._current_pose.y, a,
                               direction, has_angle, slow_go)
        return True

    def pwm(self, goal_id, left_255, right_255, duration, auto_stop):
        """
        Converts PWM to speed goal.

        @param left: left wheel speed
        @param right: left right speed
        @param duration: duration in sec
        @param auto_stop: if True, stops when we hit a wall.
        """
        if not auto_stop:
            rospy.logwarn("PWM without auto_stop has not been implemented yet...")
            return False

        rospy.loginfo("[ASSERV] Adding PWM goal of duration %2f" % duration)

        right_wheel_speed, left_wheel_speed = self._limit_wheel_speed(
            right_255/255. * self._max_linear_speed, left_255/255. * self._max_linear_speed)

        self._start_pwm(goal_id, right_wheel_speed, left_wheel_speed, duration, auto_stop)
        return True

    def speed(self, linear, angular, duration, auto_stop):
        """
        Keeps a linear and angular speed for the desired duration.

        @param left: left wheel speed
        @param right: left right speed
        @param duration: duration in sec
        @param auto_stop: if True, stops when we hit a wall.
        """
        if not auto_stop:
            rospy.logwarn("Speed without auto_stop has not been implemented yet...")
            return False

        rospy.loginfo("[ASSERV] Adding speed goal of duration %2f" % duration)

        # Get filtered right and left wheel speeds (prevents unrealistic speed)
        right_wheel_speed, left_wheel_speed = self._get_wheel_speeds(linear, angular)
        # Convert back to lin and angular speed
        lin_spd, ang_spd = self._convert_wheel_to_lin_and_ang_spd(left_wheel_speed, right_wheel_speed)

        self._start_pwm(goal_id, lin_spd, ang_spd, duration, auto_stop)
        return False

    def set_emergency_stop(self, stop):
        rospy.loginfo("[ASSERV] Emergency stop called : " + str(stop))
        self._emergency_stop = stop
        if stop:
            self._stop()

        return True

    def kill_goal(self):
        rospy.logerr("This function has not been implemented yet...")
        return False

    def clean_goals(self):
        self._goals_list = []
        # TODO check to make it proper
        self._current_goal = None
        return True

    def pause(self, pause):
        rospy.logerr("This function has not been implemented yet...")
        return False

    def reset_id(self):
        return True

    def set_max_speed(self, speed):
        self._max_linear_speed = speed
        return True

    def set_max_accel(self, accel):
        self._max_acceleration = accel
        return True

    def set_pid(self, p, i, d):
        return True

    def set_pos(self, x, y, a, mode=AsservSetPosModes.AXY):
        """
        Sets a new position. Can update x, y or a only, if you wish to.

        @param x: new x coord
        @param y: new y coord
        @param a: new angle
        @param mode: choose if you want to change any x, y, a combination.
        See AsservSetPosModes class for modes info.
        """
        if (x < 0 or x > self._map_x or
            y < 0 or y > self._map_y or
            a < -math.pi or a > math.pi ):
            rospy.logerr("Invalid position !")
            return False

        new_pose = Pose2D(self._current_pose.x, 
                          self._current_pose.y,
                          self._current_pose.theta)

        if mode == AsservSetPosModes.A:
            new_pose.theta = a
        elif mode == AsservSetPosModes.Y:
            new_pose.y = y
        elif mode == AsservSetPosModes.AY:
            new_pose.theta = a
            new_pose.y = y
        elif mode == AsservSetPosModes.X:
            new_pose.x = x
        elif mode == AsservSetPosModes.AX:
            new_pose.theta = a
            new_pose.x = x
        elif mode == AsservSetPosModes.XY:
            new_pose.x = x
            new_pose.y = y
        elif mode == AsservSetPosModes.AXY:
            new_pose.x = x
            new_pose.y = y
            new_pose.theta = a

        self._current_pose = new_pose
        
        return True

    def _callback_timer_asserv_computation(self, event):
        """
        Main function of the asserv simu.
        Checks for new goals and updates behavior every ASSERV_RATE ms.
        """
        # Check for emergency stop
        if self._emergency_stop:
            self._stop()
            return

        # Check if a goal has to be got from the list
        self._check_new_current_goal()
        
        if self._current_goal is None: return


        # Check the goal type and take adapted actions
        if isinstance(self._current_goal, PwmGoal):
            direction = 1 if self._current_goal.left_speed > 0 \
                and self._current_goal.right_speed > 0 else 0

            self._apply_wheel_acceleration(self._current_goal.left_speed, self._current_goal.right_speed)
            # Translate wheel speeds to linear and angular speed
            self._current_linear_speed, self._current_angular_speed = self._get_speeds_from_wheels()

            self._update_current_pose_pos()

            self._update_current_pose_pos()
            if self._wallhit_stop(direction):
                return
            
            self._current_goal.duration -= ASSERV_RATE
            if self._current_goal.duration <= 0:
                rospy.loginfo("[ASSERV] Spd/PWM Goal reached end of its duration !")
                self._node.goal_reached(self._current_goal.goal_id, True)
                self._stop()
                self._current_goal = None

            return

        if isinstance(self._current_goal, AsservGoal):  
            self._apply_slow_go(self._current_goal.slow_go)
            lin_error = self._get_linear_error()
            angle_error = self._get_angle_error()

            if self._current_goal.has_angle: # gotoa
                if lin_error < ASSERV_ERROR_POSITION:
                    # Position OK. Just needs to rotate.
                    self._current_goal.pose.x = self._current_pose.x
                    self._current_goal.pose.y = self._current_pose.y
                    lin_error = 0
                    LinearPD.last_error = 0
                    angle_error = self._current_pose.theta - self._current_goal.pose.theta
                    angle_error = self._clean_angle_error(angle_error)

                    if abs(angle_error) < ASSERV_ERROR_ANGLE:
                        # Position and angle OK.
                        rospy.loginfo('[ASSERV] Goal position has been reached !')
                        self._node.goal_reached(self._current_goal.goal_id, True)
                        self._stop()
                        self._current_goal = None
                        return

            elif  lin_error < ASSERV_ERROR_POSITION: # goto
                # Position OK, no need to rotate.
                rospy.loginfo('[ASSERV] Goal position has been reached !')
                self._node.goal_reached(self._current_goal.goal_id, True)
                self._stop()
                self._current_goal = None
                return

            if abs(angle_error) > math.pi :
                # It is counter-productive to have a linear speed 
                # if we are facing away from our goal. For now, only rotate.
                wanted_lin_speed = 0
            else:
                wanted_lin_speed = (lin_error * LinearPD.P
                    + (lin_error - LinearPD.last_error) * LinearPD.D)

                # Reduce linear speed proportionnaly to angle error
                wanted_lin_speed /= (1 + LinearPD.ANGLE_SPEED_DIVIDER * abs(angle_error) * abs(lin_error))

                if not self._current_goal.direction:
                    # Go backwards
                    wanted_lin_speed = -wanted_lin_speed

            wanted_ang_speed = (angle_error * AngularPID.P 
                + (angle_error - AngularPID.last_error) * AngularPID.D
                + AngularPID.error_sum * AngularPID.I)

            wanted_right_speed, wanted_left_speed = \
                self._get_wheel_speeds(wanted_lin_speed, 
                                       wanted_ang_speed)

            # Only allow for a realistic change in wheel speed
            self._apply_wheel_acceleration(wanted_right_speed, wanted_left_speed)

            # Translate wheel speeds to linear and angular speed
            self._current_linear_speed, self._current_angular_speed = self._get_speeds_from_wheels()
            
            self._update_current_pose_pos()
            self._update_PID_errors(lin_error, angle_error)

            return

    def _check_new_current_goal(self):
        """
        Checks if a new goal has to be got. 
        Initializes PID parameters if necessary.
        """
        if self._current_goal is not None or not len(self._goals_list):
            return

        self._current_goal = self._goals_list.pop(0)
        if isinstance(self._current_goal, PwmGoal):
            rospy.loginfo("[ASSERV] Starting new Spd/PWM goal with duration = %.2fs"
                            % self._current_goal.duration )
            
        elif isinstance(self._current_goal, AsservGoal):
            rospy.loginfo('[ASSERV] Starting a new goal : x = %.2f y = %.2f a = %.2f' %
                (self._current_goal.pose.x, self._current_goal.pose.y, self._current_goal.pose.theta))
            
            AngularPID.last_error = self._get_angle_error()
            AngularPID.error_sum = 0

            LinearPD.last_error = self._get_linear_error()

    def _apply_slow_go(self, slow_go):
        """
        If True, reduces max speed.
        Else, sets max speed to default.

        @param slow_go : True to reduce max spd else False
        """
        if slow_go : 
            self.set_max_speed(SLOW_GO_MAX_SPEED)
        else :
            self.set_max_speed(ASSERV_MAX_SPEED)

    def _wallhit_stop(self, direction):
        """
        Checks if the robot is near a wall.
        If it is, stops it and returns True.

        @param direction: (current direction of the robot) 1 forward, 0 backwards
        @return: True if the robot is hitting a wall, else False
        """
        # TODO avoid robot teleporting when he comes from an angle
        # TODO fix it, sometimes it doesnt work as intended
        traj_angle = self._current_pose.theta

        # Depending on the orientation of the robot, 
        # adjust the position of the map borders
        if abs(traj_angle) > math.pi:
            traj_angle -= np.sign(traj_angle) * 2 * math.pi

        # Set y border
        if traj_angle < 0 : # Facing down
            high_y_edge = self._map_y - self._robot_down_size
            low_y_wall = self._robot_up_size
        else: # Facing up
            high_y_edge = self._map_y - self._robot_up_size
            low_y_wall = self._robot_down_size

        # Set x border
        if abs(traj_angle) < math.pi/2: # Facing right
            high_x_edge = self._map_x - self._robot_up_size
            low_x_wall = self._robot_down_size
        else: # Facing left
            high_x_edge = self._map_x - self._robot_down_size
            low_x_wall = self._robot_up_size

        new_theta = None

        if self._current_pose.x >= high_x_edge :
            new_theta = 0
        elif self._current_pose.x <= low_x_wall :
            new_theta = -math.pi

        elif self._current_pose.y >= high_y_edge :
            new_theta = math.pi/2
        elif self._current_pose.y <= low_y_wall :
            new_theta = -math.pi/2
        
        if new_theta is None:
            return False

        if direction == 0 : # Semiturn if we were going backwards
            new_theta += math.pi
        new_theta %= 2 * math.pi

        self._current_pose = Pose2D(self._current_pose.x, 
                                    self._current_pose.y, 
                                    new_theta)
        rospy.loginfo('[ASSERV] A wall has been hit !')
        self._node.goal_reached(self._current_goal.goal_id, True)
        self._stop()
        self._current_goal = None

        return True

    def _stop(self):
        """
        Sets wheel speeds and linear / angular speeds to 0.
        Sets the state of the robot to IDLE.
        """
        #TODO Make the robot spd go down as fast as possible => more realistic
        self._current_linear_speed = 0
        self._current_angular_speed = 0
        self._left_wheel_speed = 0
        self._right_wheel_speed = 0

    def _get_angle_error(self):
        """
        Returns the error between the trajectory and the robot current angle.

        @return: angle error in radian
        """
        slope_angle = self._get_wanted_angle_to_align_traj()
        angle_error = self._current_pose.theta - slope_angle
        
        return self._clean_angle_error(angle_error)

    def _get_wanted_angle_to_align_traj(self):
        """
        Gets the angle we want the robot to have to be aligned on the 
        current trajectory.

        @return: angle we wish the robot to have
        """
        wanted_angle = math.atan2(self._current_goal.pose.y - self._current_pose.y,
                                  self._current_goal.pose.x - self._current_pose.x)

        # If going backwards turn 180 degrees
        if self._current_goal.direction == 0: wanted_angle += math.pi

        wanted_angle %= 2* math.pi
        return wanted_angle

    def _clean_angle_error(self, angle_error):
        """
        Returns an angle error between pi and -pi.

        @param angle_error: old angle error
        @return: filtered angle error
        """
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2* math.pi

        # Avoid error turn if angle ~= 6.28 #TODO find a better way
        if abs(abs(angle_error) - 2 * math.pi) < 0.1:
            angle_error = 0
        return angle_error

    def _get_linear_error(self):
        """
        Returns the linear error between our current goal 
        and our current position.

        @return: linear error in m
        """
        return math.sqrt((self._current_goal.pose.x - self._current_pose.x) ** 2 
                + (self._current_goal.pose.y - self._current_pose.y) ** 2)

    def _get_wheel_speeds(self, lin_spd, ang_spd):
        """
        Converts linear and angular speed to right wheel and left wheel speeds.
        Also prevents the wheels from going over the robot max speed.

        @param lin_spd: desired linear speed
        @param ang_spd: desirde angular speed

        @return: filtered right wheel speed, filtered left wheel speed
        """
        right_speed = 2 * lin_spd + ang_spd * self._robot_wheelbase
        left_speed = 2 * lin_spd - ang_spd * self._robot_wheelbase

        return self._limit_wheel_speed(right_speed, left_speed)

    def _limit_wheel_speed(self, right_speed, left_speed):
        """
        Filters wheel speeds so they don't go faster than the robot max speed.
        @param right_speed: right wheel speed
        @param left_speed: left wheel speed

        @return: filtered right wheel speed, filtered left wheel speed
        """
        if abs(right_speed) > self._max_linear_speed:
            right_speed = np.sign(right_speed) * self._max_linear_speed
        
        elif abs(right_speed) < ASSERV_MINIMAL_SPEED:
            right_speed = np.sign(right_speed) * ASSERV_MINIMAL_SPEED

        if abs(left_speed) > self._max_linear_speed:
            left_speed = np.sign(left_speed) * self._max_linear_speed
        
        elif abs(left_speed) < ASSERV_MINIMAL_SPEED:
            left_speed = np.sign(left_speed) * ASSERV_MINIMAL_SPEED

        return right_speed, left_speed

    def _apply_wheel_acceleration(self, wanted_right_speed, wanted_left_speed):
        """
        Applies the wanted right speed and wanted left speed to the wheels.
        Respects the max acceleration threshold to get a more realistic trajectory.

        @param wanted_right_speed: ideal speed of the right wheel
        @param wanted_left_speed: ideal speed of the left wheel
        """

        if (abs(self._right_wheel_speed - wanted_right_speed)
                > self._max_acceleration * ASSERV_RATE):
            self._right_wheel_speed += (
                np.sign(wanted_right_speed - self._right_wheel_speed) 
                * self._max_acceleration * ASSERV_RATE)
        else:
            self._right_wheel_speed = wanted_right_speed

        if (abs(self._left_wheel_speed - wanted_left_speed)
                > self._max_acceleration * ASSERV_RATE):
            self._left_wheel_speed += (
                np.sign(wanted_left_speed - self._left_wheel_speed)
                * self._max_acceleration * ASSERV_RATE)
        else:
            self._left_wheel_speed = wanted_left_speed

    def _get_speeds_from_wheels(self):
        """
        Takes in the speed of both wheels and returns linear and angular speeds.

        @return linear speed, angular speed
        """
        
        return self._convert_wheel_to_lin_and_ang_spd(
            self._right_wheel_speed, self._left_wheel_speed
        )
        
    def _convert_wheel_to_lin_and_ang_spd(self, right_wheel_speed, left_wheel_speed):
        """
        Converts wheel speeds to linear and angular speeds.

        @param left_wheel_speed: speed of the left wheel
        @param right_wheel_speed: speed of the right wheel

        @return linear_speed, angular_speed
        """

        lin_spd = (left_wheel_speed + right_wheel_speed) / 2
        ang_spd = (left_wheel_speed - right_wheel_speed) / self._robot_wheelbase
        if abs(ang_spd) > self._max_angular_speed:
            ang_spd = np.sign(ang_spd) * self._max_angular_speed

        return lin_spd, ang_spd

    def _update_current_pose_pos(self):
        """
        Uses the angular and linear speeds to update the robot Pose(x, y, theta).
        """
        new_theta = self._current_pose.theta + self._current_angular_speed * ASSERV_RATE
        new_theta %= 2 * math.pi

        new_x = self._current_pose.x + self._current_linear_speed * ASSERV_RATE * math.cos(
            new_theta)
        new_y = self._current_pose.y + self._current_linear_speed * ASSERV_RATE * math.sin(
            new_theta)
        
        self._current_pose = Pose2D(new_x, new_y, new_theta)

    def _update_PID_errors(self, lin_error, angle_error):
        """
        Updates AngularPID and LinearPD error memory.

        @param lin_error: last linear error
        @param angle_error: last angle error
        """
        AngularPID.last_error = angle_error

        AngularPID.error_sum += angle_error
        if AngularPID.error_sum >= AngularPID.MAX_I:
            AngularPID.error_sum = AngularPID.MAX_I

        LinearPD.last_error = lin_error

    def _rotate(self, speed_mult):
        self._current_angular_speed = speed_mult * self._max_angular_speed
        self._current_pose.theta += self._current_angular_speed * ASSERV_RATE

    def _callback_timer_pose_send(self, event):
        self._node.send_robot_position(self._current_pose)

    def _callback_timer_speed_send(self, event):
        self._node.send_robot_speed(RobotSpeed(0, 0, self._current_linear_speed, 0, 0))

    def _start_trajectory(self, goal_id, x, y, a=0, direction=1, has_angle=False, slow_go=False): #direction=1 forward, 0 backward
        self._goals_list.append(AsservGoal(goal_id, Pose2D(x, y, a), has_angle, direction, slow_go))

    def _start_pwm(self, goal_id, right_speed, left_speed, duration, auto_stop):
        self._goals_list.append(PwmGoal(goal_id, right_speed, left_speed, duration, auto_stop))