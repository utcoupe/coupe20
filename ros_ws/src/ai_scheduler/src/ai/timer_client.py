import rospy
from ai_game_manager.msg import GameTime

class TimerClient():
    def __init__(self):
        rospy.Subscriber("/ai/game_manager/time", GameTime, self.on_new_time)
        self.is_active = False
        self.game_duration = -1
        self.time_elapsed = -1

    def on_new_time(self, msg):
        self.is_active = msg.is_active
        self.game_duration = msg.game_time_duration
        self.time_elapsed = msg.game_elapsed_time
