# -*- coding: utf-8 -*-
import rospy
from definitions import *
from tasklist import TaskList
from task import Task


class Strategy(Task):
    def __init__(self, xml, actions, orders, communicator):
        super(Strategy, self).__init__(xml)
        self.Name = xml.attrib["name"]
        self.communicator = communicator
        self.loadxml(xml, actions, orders)

    def loadxml(self, xml, actions, orders):
        self.TASKS = TaskList(xml, actions, orders)

    def canContinue(self):
        return self.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING, TaskStatus.PAUSED]

    def getNext(self): # Returns the next free task (TaskList, Action or Order).
        return self.TASKS.getNext()

    def sendReward(self, communicator):
        communicator.SendRequest("/ai/scheduler/score", {"score": self.TASKS.getActiveReward()})

    def getStatus(self, text_version=False):
        return self.TASKS.getStatus(text_version)

    def PrettyPrint(self):
        rospy.loginfo("[STRATEGY {}⚡ ACTIVE] {}".format(self.TASKS.getActiveReward(), self.__repr__()))
        self.TASKS.prettyprint(1)

    def __repr__(self):
        return self.Name
