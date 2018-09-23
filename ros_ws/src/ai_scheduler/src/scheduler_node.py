#!/usr/bin/python
import time
import rospy
from scheduler_communication import AICommunication
from ai import RobotAI, GameProperties

from drivers_ard_hmi.msg import SetStrategies, SetTeams, HMIEvent
from ai_game_manager.srv import SetStatus
from ai_game_manager import StatusServices


class AINode():
    def __init__(self):
        self.DepartmentName, self.PackageName = "ai", "scheduler"

        rospy.init_node(self.PackageName, log_level = rospy.INFO)

        self.AI = RobotAI()
        self.AI.load_game_properties() # fetch available strategies and teams

        self._hmi_init = False
        self._ai_start_request = False

        self._strat_publisher = rospy.Publisher("/feedback/ard_hmi/set_strategies", SetStrategies, queue_size=10)
        self._teams_publisher = rospy.Publisher("/feedback/ard_hmi/set_teams", SetTeams, queue_size=10)
        rospy.Subscriber("/feedback/ard_hmi/hmi_event", HMIEvent, self.on_hmi_event)

        # Sending init status to ai/game_manager, subscribing to game_status status pub.
        status_services = StatusServices(self.DepartmentName, self.PackageName, self.on_arm)
        status_services.ready(True) # Tell ai/game_manager the node initialized successfuly.

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self._hmi_init:
                self.send_game_properties()
            if self._ai_start_request:
                self.AI.start(AICommunication())
                self._ai_start_request = False
            r.sleep()

    def send_game_properties(self): # Happens from init until HMI gets detected.
        self._strat_publisher.publish(GameProperties.AVAILABLE_STRATEGIES)
        self._teams_publisher.publish(GameProperties.AVAILABLE_TEAMS)

    def on_hmi_event(self, req): # Happens immediately after user clicks on 'ARM'on hmi.
        if req.event == req.EVENT_HMI_INITIALIZED:
            time.sleep(0.5)
            self._hmi_init = True
        if req.event == req.EVENT_START:
            GameProperties.CURRENT_STRATEGY = GameProperties.AVAILABLE_STRATEGIES[req.chosen_strategy_id]
            GameProperties.CURRENT_TEAM     = GameProperties.AVAILABLE_TEAMS[req.chosen_team_id]
            rospy.set_param("/current_strategy", GameProperties.CURRENT_STRATEGY)
            rospy.set_param("/current_team",     GameProperties.CURRENT_TEAM)
            rospy.logdebug("Strategy ({}) and team ({}) params set !".format(GameProperties.CURRENT_STRATEGY, GameProperties.CURRENT_TEAM))

    def on_arm(self, req): # Happens when 'ai/game_manager' launches ARM event (when user clicks on ARM on hmi).
        rospy.loginfo("[AI] Starting actions ! Strategy '{}' and team '{}'.".format(GameProperties.CURRENT_STRATEGY,
                                                                                    GameProperties.CURRENT_TEAM))
        self._ai_start_request = True # Start the strategy

'''
PACKAGE STARTING POINT HERE
'''
if __name__ == "__main__":
    node = AINode()
