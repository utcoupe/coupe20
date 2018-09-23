#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

from libtools import *

class EnemiesData:

    def __init__(self, pos):
        self.pos = pos
        self.pos_history = [[], []]

    def __setattr__(self, name, value):
        super.__setattr__(name, value)
        if name == 'pos':
            if len(self.pos_history[0]) >= self.maxPosHistory:
                del self.pos_history[0][0]
                del self.pos_history[1][0]
            self.pos_history[0].append([value.toXPoint()])
            self.pos_history[1].append([value.toYPoint()])

    def estimateSpeed(self):
        return Point(
            Polynome(self.pos_history[0]).derivative(),
            Polynome(self.pos_history[1]).derivative()
        )
    
    def can_be_same_entity(self, rect, time):
        p = Polynome(self.pos_history[0])
        p.P(time)
        
