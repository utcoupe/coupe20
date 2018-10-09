# -*- coding: utf-8 -*-

class TaskStatus():
    CRITICAL            = ('CRITICAL'           , '💔')      # Fatal error, system will shutdown.
    NEEDSPREVIOUS       = ('NEEDSPREVIOUS'      , '↳')       # Task can't execute yet because it needs the previous task to be at SUCCESS still.
    PENDING             = ('PENDING'            , '⋯')       # Active when an order (or order child in list) is waiting for response callback.
    FREE                = ('FREE'               , '⬜')      # Free task, not activated yet.
    PAUSED              = ('PAUSED'             , '')      # TODO implement entire pause engine
    ERROR               = ('ERROR'              , '❌')       # Error. Order couldn't be done, AI will try to find an alternative path of orders in the tree.
    BLOCKED             = ('BLOCKED'            , '⛔')      # Node can't execute because conditions aren't fully satisfied.
    SUCCESS             = ('SUCCESS'            , '🆗', 0.0) # Order and lists complete. #
    @staticmethod
    def toEmoji(status):
        return status[1]

class ExecutionMode():
    ALL                 = ('all'                , '⚫')    # Will execute all tasks (except the blocked ones).
    ONE                 = ('one'                , '1')    # Will execute tasks in the list until one is successful.
    ATLEASTONE          = ('+'                  , '+')    # At least one task in the list must be executed, will try to execute all.
    @staticmethod
    def toEmoji(mode):
        return mode[1]
    @staticmethod
    def fromText(text):
        if text == 'all'           : return ExecutionMode.ALL
        elif text == 'one'         : return ExecutionMode.ONE
        elif text == '+'           : return ExecutionMode.ATLEASTONE
        else: raise ValueError, "ExecutionMode '{}' not recognized.".format(text)

class ExecutionOrder():
    LINEAR              = ('linear'             , '⬇')    # Will follow linearly the order given in the XML file.
    RANDOM              = ('random'             , '🌀')    # Will pick a random free task.
    SIMULTANEOUS        = ('simultaneous'       , '⇶')    # Will activate all tasks at once.
    FASTEST             = ('fastest'            , '🕒')    # Will sort the tasks from least Duration to most.
    MOSTREWARD          = ('mostreward'         , '⚡')    # Will sort the tasks from most Reward to least. Execute tasks with same reward amount linearly.
    @staticmethod
    def toEmoji(order):
        return order[1]
    @staticmethod
    def fromText(text):
        if text == 'linear'        : return ExecutionOrder.LINEAR
        elif text == 'random'      : return ExecutionOrder.RANDOM
        elif text == 'simultaneous': return ExecutionOrder.SIMULTANEOUS
        elif text == 'fastest'     : return ExecutionOrder.FASTEST
        elif text == 'mostreward'  : return ExecutionOrder.MOSTREWARD
        else: raise ValueError, "ExecutionOrder '{}' not recognized.".format(text)

class RepeatMode():
    ONCE                = ('once'               , '1')    # Will execute the block one time.
    FOR                 = ('for'                , '')    # Will execute the given amount of times, no matter the fails.
    WHILE               = ('while'              , '∞')    # Will execute tasks in the list until one is successful.
    @staticmethod
    def toEmoji(mode):
        return mode[1]
    @staticmethod
    def fromText(text):
        if text == 'once'          : return RepeatMode.ONCE
        elif text.isdigit()        : return RepeatMode.FOR
        elif text == 'while'       : return RepeatMode.WHILE
        else: raise ValueError, "RepeatMode '{}' not recognized.".format(text)

class GameProperties():
    GAME_DURATION = None
    REWARD_POINTS = 0

    AVAILABLE_STRATEGIES = []
    AVAILABLE_TEAMS      = []

    CURRENT_STRATEGY     = ''
    CURRENT_TEAM         = ''


class Colors():
    BOLD  = "\033[1m"
    GRAY  = "\033[92m"
    BLUE  = "\033[95m"
    RED   = "\033[91m"
    RESET = "\033[0m"


class Console():
    def __init__(self):
        self.Text = Colors.RESET

    def setstyle(self, color):
        self.Text += color

    def addtext(self, text):
        self.Text += text

    def endstyle(self):
        self.Text += Colors.RESET

    def getText(self):
        self.endstyle()
        return self.Text
