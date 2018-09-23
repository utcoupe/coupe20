
import actionlib
import rospy
from actuators_abstract import ActuatorsAbstract
from movement_actuators.msg import BarrelAction, BarrelResult, DispatchAction, DispatchGoal
import drivers_ard_others.msg

from actionlib.action_client import CommState

import matplotlib.pyplot as plt

class Color:
    UNKNOWN = 0
    ORANGE = 1
    GREEN = 2

class ActuatorsBarrel(ActuatorsAbstract):
    def __init__(self):

        self.COLOR_SENSOR_TOPIC = '/drivers/ard_others/color'
        self.BARREL_NAME = 'barrel' # name in the dispatcher
        self.PRESET_BIN = 'BIN'
        self.PRESET_NORMAL = 'HIGH'
        self.PRESET_CANON = 'CANON'

        self.ORANGE_HUE = 15
        self.GREEN_HUE = 120
        self.HUE_MARGIN = 15

        self.SATURATION_TRESH = 90

        self.BALL_COUNT = 8

        self._client = actionlib.SimpleActionClient('/movement/actuators/dispatch', DispatchAction)
        self._client.wait_for_server(rospy.Duration(10))

        self._curr_color = Color.UNKNOWN

        self._color_client = rospy.Subscriber(self.COLOR_SENSOR_TOPIC, drivers_ard_others.msg.Color, self._color_callback)

        self._team_color = Color.ORANGE if rospy.get_param('/current_team', 'orange') == 'orange' else Color.GREEN

        ActuatorsAbstract.__init__(self,
                                   action_name='barrel',
                                   action_type=BarrelAction)

        self._is_running = False
        self._doing_back = False
        self._doing_forth = False

        self._curr_goal_id = None
        self._curr_pause = 0

        self._pause_timer = None

        self._timer = None
        self._curr_timeout = 0

    def _process_action(self, goal, goal_id):
        if self._isHalted:
            rospy.logerr("Barrel actuator is turned off!")
            return False
        if self._is_running:
            rospy.logerr("Received a goal but another one is in process !")
            return False

        self._is_running = True
        self._curr_goal_id = goal_id
        self._curr_pause = goal.pause

        if goal.timeout > 0:
            self._curr_timeout = goal.timeout*1000
        else:
            self._curr_timeout = 5000
            rospy.logwarn('No timeout is set, setting dispatcher timeout to 5s')

        if not goal.sort:
            rospy.logdebug('Starting goal chain with no sort')
            self._start_goal_chain(self.PRESET_CANON)
        else:
            rospy.logdebug('Starting goal chain for the bin')
            self._start_goal_chain(self.PRESET_BIN)

        return True

    def _start_back(self, e=None):
        self._timer = rospy.Timer(rospy.Duration(self._curr_timeout), lambda e: self._finish_action(False), oneshot=True)

        g = DispatchGoal()
        g.name = self.BARREL_NAME
        g.order = 'JOINT'
        g.preset = self.PRESET_NORMAL
        g.timeout = self._curr_timeout

        self._doing_back = True
        self._doing_forth = False
        self._client.send_goal(g, done_cb=self._back_done_cb)


    def _forth_done_cb(self, state, result):
        rospy.logdebug('forth_done, waiting %d seconds' % self._curr_pause)
        if self._timer:
            self._timer.shutdown()
            self._timer = None

        if self._curr_pause > 0:
            self._pause_timer = rospy.Timer(rospy.Duration(self._curr_pause), self._start_back, oneshot=True)
        else:
            self._start_back()

    def _back_done_cb(self, state, result):
        rospy.logdebug('back_done')
        self._finish_action(result.success)

    def _color_callback(self, msg):

        if msg.hue <= self.ORANGE_HUE + self.HUE_MARGIN \
            and msg.hue >= self.ORANGE_HUE - self.HUE_MARGIN\
                and msg.saturation > self.SATURATION_TRESH:
            self._curr_color = Color.ORANGE

        elif msg.hue <= self.GREEN_HUE + self.HUE_MARGIN \
            and msg.hue >= self.GREEN_HUE - self.HUE_MARGIN \
                and msg.saturation > self.SATURATION_TRESH:
            self._curr_color = Color.GREEN

        else:
            self._curr_color = Color.UNKNOWN

        if self._is_running and \
            not self._doing_back and \
            not self._doing_forth \
            and self._curr_color != Color.UNKNOWN:

            if self._team_color != self._curr_color:
                rospy.logfatal('Starting goal chain for ennemy color ball (h: %d, s: %d, v %d)'
                               % (msg.hue, msg.saturation, msg.lightness))
                self._start_goal_chain(self.PRESET_BIN)
            else:
                rospy.logfatal('Starting goal chain for ally color balll (h: %d, s: %d, v %d)'
                               % (msg.hue, msg.saturation, msg.lightness))
                self._start_goal_chain(self.PRESET_CANON)


    def _start_goal_chain(self, preset):
        self._timer = rospy.Timer(rospy.Duration(self._curr_timeout), self._start_back, oneshot=True)

        g = DispatchGoal()
        g.name = self.BARREL_NAME
        g.order = 'JOINT'
        g.preset = preset
        g.timeout = self._curr_timeout

        self._doing_forth = True
        self._client.send_goal(g, done_cb=self._forth_done_cb)

    def _finish_action(self, success):
        self._is_running = False
        self._doing_forth = False
        self._doing_back = False

        self._client.stop_tracking_goal()

        if self._timer:
            self._timer.shutdown()
            self._timer = None

        self._action_reached(self._curr_goal_id, success, BarrelResult(success=success))

        self._curr_goal_id = None
    

    def setHalted(self, isHalted):
        self._isHalted = isHalted
        self._client.cancel_all_goals()
