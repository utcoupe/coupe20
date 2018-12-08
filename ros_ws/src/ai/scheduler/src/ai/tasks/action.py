# -*- coding: utf-8 -*-
from definitions import *
from task import Task
from tasklist import TaskList


class Action(TaskList):
    def __init__(self, xml, actions, orders):
        super(Action, self).__init__(xml.find("actions"), actions, orders)
        self.Ref = xml.attrib["ref"]
        self.Name = self.Ref if not self.Name else self.Name
        self.fetchBoundParams(xml)

    def getParamForBind(self, bind):
        for task in self.TASKS:
            if task.getParamForBind(bind):
                return task.getParamForBind(bind)

    def fetchBoundParams(self, xml):
        if "params" not in [node.tag for node in xml]:
            return
        boundParamsList = []
        for param in xml.find("params"):
            if "name" not in param.attrib:
                raise KeyError("Parameters need a 'name' attribute ! (action '{}')".format(self.Name))
            name = param.attrib["name"]
            finalBind = self.getParamForBind(name)

            if not finalBind:
                raise KeyError("No parameter bound with '{}' !".format(name))
            else:
                boundParamsList.append(finalBind)
        self.BoundParams = {p.bind: p for p in boundParamsList}

    def setParameters(self, orderref_xml):
        for child in orderref_xml:
            name = child.tag
            if name not in self.BoundParams:
                raise KeyError("No bind found for '{}' !".format(name))
            self.BoundParams[name].parseValue(child)

        for p in self.BoundParams:
            if self.BoundParams[p].bind == p:
                self.BoundParams[p].checkValues()

    def __repr__(self):
        c = Console();c.setstyle(Colors.BOLD);c.setstyle(Colors.BLUE)
        c.addtext("[{} Action]".format(self.getStatusEmoji()))
        c.endstyle();c.setstyle(Colors.BLUE);c.addtext(" {0} ".format(self.Name));c.endstyle();c.setstyle(Colors.GRAY)

        c.addtext("[{}{}{}{}{}]".format(ExecutionMode.toEmoji(self.executionMode),
                                       " " + ExecutionOrder.toEmoji(self.executionOrder),
                                       " {}/{}".format(str(self._repeats), str(self._repeats_max)) \
                                            + RepeatMode.toEmoji(self.repeatMode) if self.repeatMode != RepeatMode.ONCE else "",
                                       ", {}⚡".format(self.getReward()) if self.getReward() else "",
                                       ", ~{}⌛".format(int(self.getDuration())) if self.getDuration() else ""))
        return c.getText()
