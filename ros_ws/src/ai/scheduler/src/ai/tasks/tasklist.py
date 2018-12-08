# -*- coding: utf-8 -*-
import copy
import random
import rospy
from definitions import *
from task import Task
from order import Order


class TaskList(Task):
    MAX_REPEATS = 50 # If repeat mode is 'while', this will be the repeat limit.

    def __init__(self, xml, actions, orders):
        super(TaskList, self).__init__(xml)
        self.executionMode  = ExecutionMode.fromText( xml.attrib["exec"])   if "exec"  in xml.attrib else ExecutionMode.ALL
        self.repeatMode     = RepeatMode.fromText(    xml.attrib["repeat"]) if "repeat"in xml.attrib else RepeatMode.ONCE
        self._repeats = 0 # used to track how many times the list already repeated
        self._successful_repeats = 0
        self._repeats_max = TaskList.MAX_REPEATS
        if self.repeatMode == RepeatMode.ONCE: self._repeats_max = 1
        if self.repeatMode == RepeatMode.FOR:  self._repeats_max = int(xml.attrib["repeat"])

        self.executionOrder = ExecutionOrder.fromText(xml.attrib["order"])  if "order" in xml.attrib else ExecutionOrder.LINEAR
        self.TASKS = self.loadxml(xml, actions, orders)

    def loadxml(self, xml, actions, orders):
        tasks = []
        for node_xml in xml:
            tag = node_xml.tag
            if tag == "tasklist":
                i = TaskList(node_xml, actions, orders)
                i.setParent(self)
                if "needsprevious" in node_xml.attrib and node_xml.attrib["needsprevious"] == 'true':
                    i.Status = TaskStatus.NEEDSPREVIOUS
                tasks.append(i)
            elif tag == "actionref" or tag == "orderref":
                instances = [action for action in actions if action.Ref == node_xml.attrib["ref"]] if tag == "actionref" else \
                            [order  for order  in orders  if order.Ref  == node_xml.attrib["ref"]]
                if len(instances) != 1:
                    raise KeyError, "{} action or order instance(s) found with the name '{}'.".format(len(instances), node_xml.attrib["ref"])
                i = copy.deepcopy(instances[0])
                i.setParent(self)
                i.Reward = int(node_xml.attrib["reward"]) if "reward" in node_xml.attrib else i.Reward
                i.setParameters(node_xml)
                if "needsprevious" in node_xml.attrib and node_xml.attrib["needsprevious"] == 'true':
                    i.Status = TaskStatus.NEEDSPREVIOUS
                if "name" in node_xml.attrib:
                    i.Name = node_xml.attrib["name"]
                if tag == "orderref":
                    i.Message.Timeout = float(node_xml.attrib["timeout"]) if "timeout" in node_xml.attrib else i.Message.Timeout
                tasks.append(i)
            elif tag == "team":
                if node_xml.attrib["name"] == GameProperties.CURRENT_TEAM:
                    tasks += self.loadxml(node_xml, actions, orders)
            else:
                rospy.logwarn("WARNING Element skipped at init because '{}' type was not recognized.".format(tag))
        return tasks

    def getReward(self):
        return self.Reward + sum([task.getReward() for task in self.TASKS])

    def getActiveReward(self):
        return self.getReward() if self.getStatus() == TaskStatus.SUCCESS else sum([task.getActiveReward() for task in self.TASKS])

    def getDuration(self):
        return sum([task.getDuration() for task in self.TASKS])

    def getNext(self):
        # Decides the next task(s) to execute based on the childs' statuses and the ExecutionOrder XML setting.
        for task in self.TASKS: # Execute any pending task in any case  #TODO#1 will create problems ?
            if task.getStatus() == TaskStatus.PENDING: return task

        if   self.executionOrder == ExecutionOrder.LINEAR:
            for task in self.TASKS:
                if task.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING]: return task
            for task in self.TASKS:
                if task.getStatus() == TaskStatus.PAUSED: return task


        elif self.executionOrder == ExecutionOrder.RANDOM:
            tasks = [task for task in self.TASKS if task.getStatus() == TaskStatus.FREE]
            if not tasks:
                tasks = [task for task in self.TASKS if task.getStatus() == TaskStatus.PAUSED]
            return tasks[random.randint(0, len(tasks) - 1)]

        elif self.executionOrder == ExecutionOrder.SIMULTANEOUS:
            return [task for task in self.TASKS if task.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING, TaskStatus.PAUSED]] #TODO#1

        elif self.executionOrder == ExecutionOrder.FASTEST:
            record, next_task = 10000000, None
            tasks = [task for task in self.TASKS if task.getStatus() == TaskStatus.FREE]
            if not tasks:
                tasks = [task for task in self.TASKS if task.getStatus() == TaskStatus.PAUSED]
            for task in tasks:
                if task.getDuration() < record:
                    record = task.getDuration()
                    next_task = task
            return next_task

        elif self.executionOrder == ExecutionOrder.MOSTREWARD:
            record, result = -1, None
            tasks = [task for task in self.TASKS if task.getStatus() == TaskStatus.FREE]
            if not tasks:
                tasks = [task for task in self.TASKS if task.getStatus() == TaskStatus.PAUSED]
            for task in tasks:
                if task.getReward() > record:
                    record = task.getReward()
                    result = task
            return result

    def execute(self, communicator):
        if self.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING, TaskStatus.PAUSED]:
            next_tasks = self.getNext()
            if type(next_tasks) is list:
                raise NotImplementedError, "Simultaneous task launches aren't supported yet !" #TODO
            next_tasks.execute(communicator)
        else:
            raise ValueError, "ERROR asked to execute '{}' task that's not free".format(self.__repr__())

    def resetStatus(self, refresh_parent=False): # wipes all progress of this list and all descendent tasks.
        self.setStatus(TaskStatus.FREE, refresh_parent)
        if not refresh_parent: #TODO ~~~~
            self._repeats = 0
        for task in self.TASKS:
            task.resetStatus()

    def _markSuccess(self):
        if self.repeatMode != RepeatMode.ONCE:
            if self.repeatMode == RepeatMode.WHILE or self.repeatMode == RepeatMode.FOR:
                self._repeats += 1
                self._successful_repeats += 1
                if self._repeats < self._repeats_max: # if repeat limit not reached yet, mark everything as free
                    self.resetStatus(refresh_parent=True)
                    return
                elif self._successful_repeats < self._repeats_max:
                    self.setStatus(TaskStatus.ERROR)
                    return

        self.setStatus(TaskStatus.SUCCESS)

    def refreshStatus(self):
        # unblock or block tasks that need previous tasks
        previous_task = self.TASKS[0]
        for task in self.TASKS[1:]:
            if task.getStatus() == TaskStatus.NEEDSPREVIOUS:
                if previous_task.getStatus() == TaskStatus.SUCCESS:
                    task.setStatus(TaskStatus.FREE, refresh_parent = False)
                if previous_task.getStatus() in [TaskStatus.BLOCKED, TaskStatus.ERROR]:
                    task.setStatus(TaskStatus.BLOCKED)
            previous_task = task

        # Decides the status of the list based on the childs' statuses and the ExecutionMode XML setting.
        child_statuses = [task.getStatus() for task in self.TASKS]
        # CAUTION The order of conditions do count!
        if TaskStatus.CRITICAL in child_statuses:
            self.setStatus(TaskStatus.CRITICAL);return

        if self.executionMode == ExecutionMode.ONE:
            if len([1 for c in child_statuses if c == TaskStatus.SUCCESS]) == 1:
                self._markSuccess()
                for task in self.TASKS:
                    if task.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING]:
                        task.setStatus(TaskStatus.BLOCKED)
                return

        if TaskStatus.PENDING in child_statuses:
            self.setStatus(TaskStatus.PENDING);return
        if TaskStatus.FREE in child_statuses:
            self.setStatus(TaskStatus.PENDING);return
        if TaskStatus.PAUSED in child_statuses:
            self.setStatus(TaskStatus.PAUSED);return

        if self.executionMode == ExecutionMode.ALL:
            if len([1 for c in child_statuses if c == TaskStatus.SUCCESS]) == len(child_statuses):
                self._markSuccess();return
        elif self.executionMode == ExecutionMode.ATLEASTONE:
            if len([1 for c in child_statuses if c == TaskStatus.SUCCESS]) >= 1:
                self._markSuccess();return

        if TaskStatus.ERROR in child_statuses:
            if self.repeatMode == RepeatMode.WHILE or self.repeatMode == RepeatMode.FOR:
                self._repeats += 1
                if self._repeats < self._repeats_max: # if repeat limit not reached yet, mark everything as free
                    self.resetStatus(refresh_parent=True)
                    self.setStatus(TaskStatus.PENDING)
                    return
                elif self._successful_repeats < self._repeats_max:
                    self.setStatus(TaskStatus.ERROR)
                    return
            else:
                self.setStatus(TaskStatus.ERROR)
                #TODO Block all dependent nodes
                return

        if TaskStatus.BLOCKED in child_statuses:
            self.setStatus(TaskStatus.BLOCKED)

    def prettyprint(self, indentlevel):
        super(TaskList, self).prettyprint(indentlevel)
        for task in self.TASKS:
            task.prettyprint(indentlevel + 1)

    def __repr__(self):
        c = Console();c.setstyle(Colors.BOLD);c.setstyle(Colors.RED)
        c.addtext("[{} TaskList] {} ".format(self.getStatusEmoji(), self.Name))
        c.endstyle();c.setstyle(Colors.GRAY)
        c.addtext("[{}{}{}{}{}]".format(ExecutionMode.toEmoji(self.executionMode),
                                       " " + ExecutionOrder.toEmoji(self.executionOrder),
                                       " {}/{}".format(str(self._repeats), str(self._repeats_max)) \
                                            + RepeatMode.toEmoji(self.repeatMode) if self.repeatMode != RepeatMode.ONCE else "",
                                       ", {}⚡".format(self.getReward()) if self.getReward() else "",
                                       ", ~{}⌛".format(int(self.getDuration())) if self.getDuration() else ""))
        return c.getText()
