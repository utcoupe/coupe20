#!/bin/usr/python
import rospy
from timer_client import TimerClient
from game_status_client import GameStatusClient, GameStatusConstants
from ai_loader import AILoader

class RobotAI():
    def __init__(self):
        self.timer = TimerClient() # Timer client.
        self.game_status = GameStatusClient()
        self._loader = AILoader()

    def load_game_properties(self):
        return self._loader.load_game_properties()

    def start(self, communicator):
        strategy = self._loader.load(communicator)
        self.execute(strategy)

    def execute(self, strategy):
        strategy.PrettyPrint()
        # Run the whole AI until there are no orders left to execute
        while not rospy.is_shutdown():
            if self.game_status.status == GameStatusConstants.STATUS_HALT:
                rospy.logwarn("[AI] detected game_status STATUS_HALT, aborting actions.")
                break

            if strategy.canContinue():
                strategy.getNext().execute(strategy.communicator)
            else:
                rospy.loginfo("[AI] In-Game actions finished!")
                break
            strategy.sendReward(strategy.communicator)
        strategy.PrettyPrint()
