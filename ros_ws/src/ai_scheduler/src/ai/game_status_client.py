import rospy
from ai_game_manager.msg import GameStatus

class GameStatusConstants():
    STATUS_INIT   = 0
    STATUS_INGAME = 1
    STATUS_HALT   = 2

class GameStatusClient():
    def __init__(self):
        rospy.Subscriber("/ai/game_manager/status", GameStatus, self.on_new_status)
        self.status = 0
        self.init_status = 0

    def on_new_status(self, msg):
        self.status = msg.game_status
        self.init_status = msg.init_status
