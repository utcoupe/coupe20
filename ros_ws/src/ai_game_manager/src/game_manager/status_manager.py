import time, rospy

from ai_game_manager.msg import GameStatus, NodesStatus, ArmRequest
from ai_game_manager.srv import SetStatus, SetStatusResponse, NodeReady, NodeReadyResponse
from drivers_ard_hmi.msg import HMIEvent

from status_config import Status

class StatusManager():
    INIT_TIMEOUT = 40 # seconds to wait for the nodes to send their init response before timeout.

    def __init__(self):
        self._node_ready_notif = rospy.Service("/ai/game_manager/node_ready", NodeReady, self.on_node_ready)
        self._set_status_srv   = rospy.Service("/ai/game_manager/set_status", SetStatus, self.on_set_status)
        self._game_status_pub  = rospy.Publisher("/ai/game_manager/status", GameStatus,        queue_size = 10)
        self._arm_pub          = rospy.Publisher("/ai/game_manager/arm",    ArmRequest,        queue_size = 1)
        self._nodes_status_pub = rospy.Publisher("/ai/game_manager/nodes_status", NodesStatus, queue_size = 10)

        rospy.Subscriber("/feedback/ard_hmi/hmi_event", HMIEvent, self.on_hmi_event)

        self.game_status = Status.STATUS_INIT
        self.init_status = Status.INIT_INITIALIZING

        self._init_start_time = time.time()
    
    def update(self):
        if self.init_status == Status.INIT_INITIALIZING:
            self.check_init_checklist()
            if time.time() - self._init_start_time > self.INIT_TIMEOUT:
                rospy.logdebug("Waited %d seconds" % self.INIT_TIMEOUT)
                if len([n for n in Status.INIT_CHECKLIST if Status.INIT_CHECKLIST[n] in [None, False]]) > 0:
                    self.set_init_status(Status.INIT_FAILED)
                else:
                    self.set_init_status(Status.INIT_INITIALIZED)
        self.publish_statuses() # publish game status at 5Hz.

    def set_init_status(self, new_status):
        if new_status == Status.INIT_INITIALIZING:
            rospy.loginfo("game_status set to INITIALIZING.")
        elif new_status == Status.INIT_INITIALIZED:
            rospy.loginfo("All nodes initialized successfully, ready to start !")
        elif new_status == Status.INIT_FAILED:
            rospy.logerr("Init timeout reached, certain nodes failed ({}) or didn't respond ({}) ! "
                         "System will continue, but be careful..".format(
                         ', '.join([n for n in Status.INIT_CHECKLIST if Status.INIT_CHECKLIST[n] == False]),
                         ', '.join([n for n in Status.INIT_CHECKLIST if Status.INIT_CHECKLIST[n] == None])))
        self.init_status = new_status

    def publish_statuses(self):
        m = GameStatus()
        m.game_status = self.game_status
        m.init_status = self.init_status
        self._game_status_pub.publish(m)

        m = NodesStatus()
        for node in Status.INIT_CHECKLIST:
            if Status.INIT_CHECKLIST[node] is None:
                m.pending_nodes.append(node)
            elif Status.INIT_CHECKLIST[node] is True:
                m.ready_nodes.append(node)
            elif Status.INIT_CHECKLIST[node] is False:
                m.failed_nodes.append(node)
        self._nodes_status_pub.publish(m)

    def check_init_checklist(self):
        for node in Status.INIT_CHECKLIST:
            if Status.INIT_CHECKLIST[node] in [None, False]:
                return
        if self.init_status == Status.INIT_INITIALIZING:
            self.set_init_status(Status.INIT_INITIALIZED)
        else:
            rospy.logerr("Unexpected behaviour : init_status checklist got full but status is not INITIALIZING.")

    def on_node_ready(self, msg):
        if msg.node_name in Status.INIT_CHECKLIST:
            Status.INIT_CHECKLIST[msg.node_name] = msg.success
            return NodeReadyResponse()
        else:
            rospy.logwarn("Node name '{}' not in ai/game_manager init checklist, passing.".format(msg.node_name))
            return NodeReadyResponse()

    def on_set_status(self, req):
        self.game_status = req.new_game_status
        return SetStatusResponse(True)

    def on_hmi_event(self, req):
        if req.event == req.EVENT_START:
            rospy.loginfo("Received HMI event, publishing arm request")
            self._arm_pub.publish(ArmRequest())
        elif req.event == req.EVENT_GAME_CANCEL:
            self.game_status = GameStatus.STATUS_HALT