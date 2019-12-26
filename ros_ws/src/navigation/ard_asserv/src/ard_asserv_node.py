#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Pose2D, TransformStamped, Pose
import actionlib
from ard_asserv.srv import *
from ard_asserv.msg import *
from port_finder.srv import *
import asserv
from game_manager import StatusServices
from game_manager.msg import GameStatus
from static_map.srv import MapGetWaypoint
from static_map.msg import Waypoint
import tf
import tf2_ros
import tf2_geometry_msgs  # DO NOT REMOVE, declares geometry_msgs conversions (yes that's ugly)

__author__ = "Thomas Fuhrmann"
__date__ = 21 / 10 / 2017

NODE_NAME = "ard_asserv"
GET_PORT_SERVICE_NAME = "drivers/port_finder/get_port"
GET_MAP_SERVICE_NAME = "static_map/get_waypoint"
GET_PORT_SERVICE_TIMEOUT = 25  # in seconds
GET_MAP_SERVICE_TIMEOUT = 15  # in seconds
TF_ASSERV = "robot"

class goalCounter:
    """
    Class used to store the goal id counter.
    This makes the id be passed by reference.
    This way, the asserv_real class can increase the counter.
    """
    def __init__(self):
        self.id = 0

class Asserv:
    """
    The Asserv class manages the driver node for communication with asserv (real using the Arduino or in simulation).
    As this node is used as an interface, it gets orders to send on ROS services. The state of the robot is published on ROS topics.
    This node handles an action (see actionlib) for the Goto movement.
    The selection of asserv type (real or simu) is made using serial port data. If the Arduino asserv is found, the real asserv is used.
    If no Arduino asserv is found, the node will launch a simulated asserv.
    """

    def __init__(self):
        rospy.logdebug("[ASSERV] Starting asserv_node.")
        # This dictionary stores the goals received by the DoGoto action and which are currently in processing
        self._goals_dictionary = {}
        # This dictionary stores the goals received by the Goto service and which are currently in processing
        self._goto_srv_dictionary = {}
        # Unique ID given to the received goal to manage it
        self._goal_counter = goalCounter()
        # Instance of the asserv object (simu or real)
        self._asserv_instance = None
        # Flag to know if the system has been halted (end of game)
        self._is_halted = False
        # Init ROS stuff
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        self._pub_robot_pose = rospy.Publisher(
            "drivers/" + NODE_NAME + "/pose2d", Pose2D, queue_size=5
        )
        self._pub_robot_speed = rospy.Publisher(
            "drivers/" + NODE_NAME + "/speed", RobotSpeed, queue_size=5
        )
        self._srv_goto = rospy.Service(
            "drivers/" + NODE_NAME + "/goto", Goto, self._callback_goto
        )

        self._srv_pwm = rospy.Service(
            "drivers/" + NODE_NAME + "/pwm", Pwm, self._callback_pwm
        )
        self._srv_speed = rospy.Service(
            "drivers/" + NODE_NAME + "/speed", Speed, self._callback_speed
        )
        self._srv_set_pos = rospy.Service(
            "drivers/" + NODE_NAME + "/set_pos", SetPos, self._callback_set_pos
        )
        self._srv_emergency_stop = rospy.Service(
            "drivers/" + NODE_NAME + "/emergency_stop",
            EmergencyStop,
            self._callback_emergency_stop,
        )
        self._srv_params = rospy.Service(
            "drivers/" + NODE_NAME + "/parameters",
            Parameters,
            self._callback_asserv_param,
        )
        self._srv_management = rospy.Service(
            "drivers/" + NODE_NAME + "/management",
            Management,
            self._callback_management,
        )
        self._act_goto = actionlib.ActionServer(
            "drivers/" + NODE_NAME + "/goto_action",
            DoGotoAction,
            self._callback_action_goto,
            auto_start=False,
        )
        self._pub_tf_odom = tf2_ros.TransformBroadcaster()
        self._act_goto.start()
        self._srv_client_map_fill_waypoints = None
        self._buffer_tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._buffer_tf)
        try:
            rospy.wait_for_service(GET_PORT_SERVICE_NAME, GET_PORT_SERVICE_TIMEOUT)
            srv_client_get_port = rospy.ServiceProxy(GET_PORT_SERVICE_NAME, GetPort)
            arduino_port = srv_client_get_port("ard_asserv").port
        except rospy.ROSException as exc:
            rospy.loginfo("Port_finder has not been launched...")
            arduino_port = ""
        rospy.loginfo("Port_finder returns value : " + arduino_port)

        is_simu = False
        if arduino_port == "":
            rospy.logwarn("[ASSERV] Creation of the simu asserv.")
            self._asserv_instance = asserv.AsservSimu(self, self._goal_counter)
            is_simu = True
        else:
            rospy.loginfo("[ASSERV] Creation of the real asserv.")
            self._asserv_instance = asserv.AsservReal(self, self._goal_counter, arduino_port)
        try:
            rospy.wait_for_service(GET_MAP_SERVICE_NAME, GET_MAP_SERVICE_TIMEOUT)
            self._srv_client_map_fill_waypoints = rospy.ServiceProxy(
                GET_MAP_SERVICE_NAME, MapGetWaypoint
            )
            rospy.logdebug("static_map has been found.")
        except rospy.ROSException as exc:
            rospy.logwarn("static_map has not been launched...")
        # Tell ai/game_manager the node initialized successfuly.
        StatusServices("drivers", "ard_asserv", None, self._callback_game_status).ready(
            not is_simu
        )

        self._asserv_instance.start()

    # goal_id generated by node, reached to know if the gial as been reached
    def goal_reached(self, goal_id, reached):
        if reached:
            rospy.loginfo("[ASSERV] Goal id {}, has been reached.".format(goal_id))
        else:
            rospy.logwarn("[ASSERV] Goal id {}, has NOT been reached.".format(goal_id))
        result = DoGotoResult(reached)

        if goal_id in self._goals_dictionary:
            self._goals_dictionary[goal_id].set_succeeded(result)
            del self._goals_dictionary[goal_id]
        elif goal_id in self._goto_srv_dictionary:
            del self._goto_srv_dictionary[goal_id]

    # robot_pose is a Pose2d structure
    def send_robot_position(self, robot_pose):
        self._pub_robot_pose.publish(robot_pose)
        # Send the position using tf
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = robot_pose.x
        t.transform.translation.y = robot_pose.y
        t.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, robot_pose.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._pub_tf_odom.sendTransform(t)

    # robot_speed is a RobotSpeed structure
    def send_robot_speed(self, robot_speed):
        self._pub_robot_speed.publish(robot_speed)

    def _callback_goto(self, request):
        """
        Callback of the Goto service.
        @param request: Service request
        @type request:  GotoRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         GotoResponse
        """
        rospy.logdebug("[ASSERV] Received a request (goto service).")

        response = self._process_goto_order(
            self._goal_counter.id,
            request.mode,
            request.position.x,
            request.position.y,
            request.position.theta,
            request.direction,
            int(request.slow_go),
        )
        if response:
            self._goto_srv_dictionary[self._goal_counter.id] = ""
        else:
            rospy.logerr(
                "[ASSERV] Service GOTO has failed... Mode probably does not exist."
            )
        return GotoResponse(response)

    def _callback_set_pos(self, request):
        """
        Callback of the SetPos service.
        @param request: Service request
        @type request:  SetPosRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         SetPosResponse
        """
        set_position = Pose2D(0, 0, 0)
        if request.position_waypoint == "":
            rospy.logdebug("[ASSERV] Received a request (set_pos service).")
            set_position = request.position
        else:
            rospy.logdebug(
                "[ASSERV] Received a request (set_pos service), using waypoint"
            )
            if self._srv_client_map_fill_waypoints is not None:
                wpt = Waypoint(name=request.position_waypoint)
                wpt.has_angle = True
                set_position = self._srv_client_map_fill_waypoints.call(
                    wpt
                ).filled_waypoint.pose
            else:
                rospy.logwarn(
                    "[ASSERV] Received a waypoint request but static_map seems not to be launched..."
                )
        if self._asserv_instance:
            ret_value = self._asserv_instance.set_pos(
                set_position.x, set_position.y, set_position.theta, request.mode
            )
        else:
            ret_value = False
        return SetPosResponse(ret_value)

    def _callback_pwm(self, request):
        """
        Callback of the Pwm service.
        @param request: Service request
        @type request:  PwmRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         PwmResponse
        """
        rospy.loginfo("[ASSERV] Received a request (pwm service).")
        if self._asserv_instance:
            ret_value = self._asserv_instance.pwm(
                self._goal_counter.id,
                request.left,
                request.right,
                request.duration,
                request.auto_stop
            )
        else:
            ret_value = False

        if ret_value:
            self._goto_srv_dictionary[self._goal_counter.id] = ""
        return PwmResponse(ret_value)

    def _callback_speed(self, request):
        """
        Callback of the Speed service.
        @param request: Service request
        @type request:  SpeedRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         SpeedResponse
        """
        rospy.logdebug("[ASSERV] Received a request (speed service).")
        if self._asserv_instance:
            ret_value = self._asserv_instance.speed(
                request.linear, request.angular, request.duration, request.auto_stop
            )
        else:
            ret_value = False
        return SpeedResponse(ret_value)

    def _callback_emergency_stop(self, request):
        """
        Callback of the EmergencyStop service.
        @param request: Service request
        @type request:  EmergencyStopRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         EmergencyStopResponse
        """
        rospy.logdebug("[ASSERV] Received a request (emergency_stop service).")
        if self._asserv_instance:
            ret_value = self._asserv_instance.set_emergency_stop(request.enable)
        else:
            ret_value = False
        return EmergencyStopResponse(ret_value)

    def _callback_asserv_param(self, request):
        """
        Callback of the Parameters service.
        @param request: Service request
        @type request:  ParametersRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         ParametersResponse
        """
        rospy.logdebug("[ASSERV] Received a request (parameters service).")
        response = True
        if self._asserv_instance:
            if request.mode == request.SPDMAX:
                response = self._asserv_instance.set_max_speed(
                    request.spd, request.spd_ratio
                )
            elif request.mode == request.ACCMAX:
                response = self._asserv_instance.set_max_accel(request.acc)
            elif request.mode == request.PIDRIGHT:
                # TODO manage left and right
                response = self._asserv_instance.set_pid(
                    request.p, request.i, request.d
                )
            elif request.mode == request.PIDLEFT:
                # TODO manage left and right
                response = self._asserv_instance.set_pid(
                    request.p, request.i, request.d
                )
            elif request.mode == request.PIDALL:
                response = self._asserv_instance.set_pid(
                    request.p, request.i, request.d
                )
            else:
                response = False
                rospy.logerr(
                    "[ASSERV] Parameter mode %d does not exists...", request.mode
                )
        else:
            response = False
        return ParametersResponse(response)

    def _callback_management(self, request):
        """
        Callback of the Management service.
        @param request: Service request
        @type request:  ManagementRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         ManagementResponse
        """
        rospy.logdebug("[ASSERV] Received a request (management service).")
        response = True
        if self._asserv_instance:
            if request.mode == request.KILLG:
                response = self._asserv_instance.kill_goal()
            elif request.mode == request.CLEANG:
                response = self._asserv_instance.clean_goals()
                # Delete all internal goals
                for goal in self._goals_dictionary.values():
                    goal.set_canceled()
                self._goals_dictionary.clear()
            elif request.mode == request.PAUSE:
                response = self._asserv_instance.pause(True)
            elif request.mode == request.RESUME:
                response = self._asserv_instance.pause(False)
            elif request.mode == request.RESET_ID:
                response = self._asserv_instance.reset_id()
            else:
                response = False
                rospy.logerr(
                    "[ASSERV] Management mode %d does not exists...", request.mode
                )
        else:
            response = False
        return ManagementResponse(response)

    def _callback_action_goto(self, goal_handled):
        """
        Callback of the DoGoto action.
        @param goal_handled:    Goal handler corresponding to the received action
        @type goal_handled:     ServerGoalHandle
        """
        if not self._is_halted:
            pos = Pose2D(0, 0, 0)
            if goal_handled.get_goal().position_waypoint == "":
                rospy.logdebug("[ASSERV] Received a request (dogoto action).")
                pose_stamped = goal_handled.get_goal().position
                target_pose = pose_stamped.pose
                if pose_stamped.header.frame_id != TF_ASSERV:
                    try:
                        target_pose = self._buffer_tf.transform(
                            pose_stamped, TF_ASSERV
                        ).pose
                    except (tf2_ros.TypeException, tf2_ros.LookupException) as e:
                        rospy.logerr("Failed to convert tf: " + repr(e))
                        rospy.logerr("=> Will not convert tf.")
                pos = Asserv._pose_to_pose2D(target_pose)
            else:
                rospy.logdebug(
                    "[ASSERV] Received a request (dogoto action), using waypoint"
                )
                if self._srv_client_map_fill_waypoints is not None:
                    wpt = Waypoint(name=goal_handled.get_goal().position_waypoint)
                    wpt.has_angle = True
                    pos = self._srv_client_map_fill_waypoints.call(
                        wpt
                    ).filled_waypoint.pose
                else:
                    rospy.logwarn(
                        "[ASSERV] Received a waypoint request but static_map seems not to be launched..."
                    )
                    goal_handled.set_rejected()
                    return

            if self._process_goto_order(
                    self._goal_counter.id,
                    goal_handled.get_goal().mode,
                    pos.x,
                    pos.y,
                    pos.theta,
                    goal_handled.get_goal().direction,
                    goal_handled.get_goal().slow_go,
            ):
                goal_handled.set_accepted()
                self._goals_dictionary[self._goal_counter.id - 1] = goal_handled
            else:
                rospy.logerr(
                    "[ASSERV] Action GOTO has failed... Mode probably does not exist."
                )
        else:
            goal_handled.set_rejected()
            rospy.logwarn(
                "[ASSERV] Action GOTO can not be accepted, asserv has been halted."
            )

    def _process_goto_order(self, goal_id, mode, x, y, a, direction, slow_go):
        """
        Processes the goto order, coming from service or action.
        @param mode:    Mode of the Goto order (see Goto.srv or DoGoto.action files)
        @type mode:     string
        @param x:       X coordinate (in meters)
        @type x:        float64
        @param y:       Y coordinate (in meters)
        @type y:        float64
        @param a:       Angle (in radians)
        @type a:        float64
        @param slow_go:  true to set SLOW_GO_BIT (control.max_spd *= EMERGENCY_SLOW_GO_RATIO)
        @type slow_go:   bool 
        @return:        True if order sent, false otherwise
        @rtype:         bool
        """
        to_return = True
        if self._asserv_instance:
            if mode == GotoRequest.GOTO:
                rospy.loginfo(
                    "[ASSERV] Accepting goal GOTO (id = {}, x = {}, y = {}).".format(
                        goal_id, x, y
                    )
                )
                to_return = self._asserv_instance.goto(
                    goal_id, x, y, direction, slow_go
                )
            elif mode == GotoRequest.GOTOA:
                rospy.loginfo(
                    "[ASSERV] Accepting goal GOTOA (id = {}, x = {}, y = {}, a = {}).".format(
                        goal_id, x, y, a
                    )
                )
                to_return = self._asserv_instance.gotoa(
                    goal_id, x, y, a, direction, slow_go
                )
            elif mode == GotoRequest.ROT:
                rospy.loginfo(
                    "[ASSERV] Accepting goal ROT (id = {}, a = {}).".format(goal_id, a)
                )
                to_return = self._asserv_instance.rot(goal_id, a, False)
            elif mode == GotoRequest.ROTNOMODULO:
                rospy.loginfo(
                    "[ASSERV] Accepting goal ROT NOMODULO (id = {}, a = {}).".format(
                        goal_id, a
                    )
                )
                to_return = self._asserv_instance.rot(goal_id, a, True)
            else:
                to_return = False
                rospy.logerr("[ASSERV] GOTO mode %d does not exists...", mode)
        else:
            to_return = False
        return to_return

    def _callback_game_status(self, msg):
        if not self._is_halted and msg.game_status == GameStatus.STATUS_HALT:
            self._is_halted = True
            # Halt the system, emergency stop + clean all goals
            if self._asserv_instance:
                self._asserv_instance.set_emergency_stop(True)
            management_msg = ManagementRequest()
            management_msg.mode = ManagementRequest.CLEANG
            self._callback_management(management_msg)
            rospy.loginfo("Asserv successfuly stopped")
        elif self._is_halted and msg.game_status != GameStatus.STATUS_HALT:
            self._is_halted = False
            if self._asserv_instance:
                self._asserv_instance.set_emergency_stop(False)

    @staticmethod
    def _pose_to_pose2D(pose):
        _, _, theta = Asserv._quaternion_to_euler_angle(pose.orientation)
        return Pose2D(pose.position.x, pose.position.y, theta)

    @staticmethod
    def _quaternion_to_euler_angle(quaternion):
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y ** 2)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y ** 2 + z * z)
        Z = math.atan2(t3, t4)
        return (X, Y, Z)


if __name__ == "__main__":
    Asserv()
