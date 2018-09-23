#!/usr/bin/python
import rospy

'''
Imported from ai/game_manager.
'''

from ai_game_manager.msg import GameStatus, ArmRequest
from ai_game_manager.srv import NodeReady


class StatusServices(object):
    READY_SRV = "/ai/game_manager/node_ready" # Service to call when the node has finished its init phase (successful or not).
    ARM_SRV   = "/ai/game_manager/arm"        # Server the node can use if it needs to be calibrated at one point (called by scheduler before jack)
    HALT_SRV  = "/ai/game_manager/status"     # Topic that can be used to know when HALT is activated (if the node needs to be stopped).

    def __init__(self, namespace, packagename, arm_cb = None, status_cb = None):
        self.node_name = "/{}/{}".format(namespace, packagename)
        if arm_cb: rospy.Subscriber(self.ARM_SRV, ArmRequest, arm_cb)
        if status_cb: rospy.Subscriber(self.HALT_SRV, GameStatus, status_cb)

    def ready(self, success):
        try:
            rospy.wait_for_service(self.READY_SRV, timeout = 4.0)
            _ready_pub = rospy.ServiceProxy(self.READY_SRV, NodeReady)
            _ready_pub(self.node_name, success)

            if success: rospy.loginfo("Node '{}' initialized successfully.".format(self.node_name))
            else:       rospy.logerr( "Node '{}' didn't initialize correctly.".format(self.node_name))
        except:
            rospy.logerr("status_services couldn't contact ai/game_manager to send init notification.")


'''
EXAMPLE USAGE

def on_arm():
    # ... calibration ...
    return True # True if arm successful, False otherwise.

def on_status(req):
    if req.game_status == req.STATUS_HALT:
        pass # ... e.g. stop updating TFs ...

def start():
    rospy.init_node("nodename", log_level=rospy.INFO)
    self.status_services = StatusServices("recognition", "localizer", on_arm, on_status)
    # ... initialization ...
    self.status_services.ready(True) # True if init successful, False otherwise.

if __name__ == "__main__":
    start()
'''
