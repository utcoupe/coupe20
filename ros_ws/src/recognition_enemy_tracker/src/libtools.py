#!/usr/bin/python2.7
# -*-coding:Utf-8 -*
from numpy import array
from numpy.linalg import solve
from math import sqrt

class Rect:
    def __init__(self,x=0,y=0,w=0,h=0,t=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.t = t

    def toXPoint(self):
        return Point(self.t, self.x + self.w/2)

    def toYPoint(self):
        return Point(self.t, self.y + self.h/2)

    def diagonal(self):
        return sqrt(self.w**2+self.h**2)

    def __repr__(self):
        return 'Rect( x={}, y={}, w={}, h={}, t={} )'.format(self.x, self.y, self.w, self.h, self.t)
class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __repr__(self):
        return '({}, {})'.format(self.x, self.y)
        

class Polynome:
    def __init__(self, point = []):
        n = len(point)
        if n > 0:
            sys = []
            val = []
            for p in point:
                eq = []
                for i in range(0, n):
                    eq.append(p.x**i)
                val.append(p.y)
                sys.append(eq)
            self.coef = solve(array(sys),array(val))
        else:
            self.coef = [0]
    
    def derivative(self):
        p = Polynome()
        del p.coef[0]
        for i in range(1, len(self.coef)):
            p.coef.append(self.coef[i]*i)
        if len(p.coef) == 0:
            p.coef.append(0)
        return p
    
    def P(self,x):
        y = 0
        for i in range(0,len(self.coef)):
            y += self.coef * x**i
        return y

    def __repr__(self):
        return 'P{}'.format(self.coef)
