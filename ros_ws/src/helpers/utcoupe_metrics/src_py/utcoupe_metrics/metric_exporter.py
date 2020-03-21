#!/usr/bin/env python
# -*-coding:Utf-8 -*

__author__ = "Gaëtan Blond"
__date__ = "20/03/20"

import rospy

from metric_types import Chronometer, Counter, Gauge

ROOT_METRICS_TOPIC_NAME="/metrics/"

class MetricExporter(object):
    def __init__(self, node_name, publishMetrics = True):
        rospy.loginfo("Creating metrics exporter for node " + node_name)

        self.m_root_metrics_topic_name = ROOT_METRICS_TOPIC_NAME + node_name + "/"
        self.m_publishMetrics = publishMetrics

        self.m_counters = {}
        self.m_gauges = {}
        self.m_chronometers = {}

    def activateMetrics(self):
        self.m_publishMetrics = True
    
    def deactivateMetrics(self):
        self.m_publishMetrics = False
    
    def createCounter(self, counter_name, start_val = 0.0):
        if self.m_hasMetric(counter_name):
            raise Exception("metric %s already exists" % counter_name)
        self.m_counters[counter_name] = Counter(self.m_root_metrics_topic_name + counter_name)
        if self.m_publishMetrics:
            self.m_counters[counter_name].publish()
    
    def counterIncrement(self, counter_name):
        self.m_counters[counter_name].inc()
        if self.m_publishMetrics:
            self.m_counters[counter_name].publish()
    
    def createGauge(self, gauge_name):
        if self.m_hasMetric(gauge_name):
            raise Exception("metric %s already exists" % gauge_name)
        self.m_gauges[gauge_name] = Gauge(self.m_root_metrics_topic_name + gauge_name)
        if self.m_publishMetrics:
            self.m_counters[gauge_name].publish()
    
    def gaugeIncrement(self, gauge_name, inc_val = 1.0):
        self.m_gauges[gauge_name].inc(inc_val)
        if self.m_publishMetrics:
            self.m_counters[gauge_name].publish()
    
    def gaugeDecrement(self, gauge_name, dec_val = 1.0):
        self.gaugeIncrement(gauge_name, -dec_val)
    
    def gaugeSet(self, gauge_name, val):
        self.m_gauges[gauge_name].set(val)
        if self.m_publishMetrics:
            self.m_counters[gauge_name].publish()
    
    def gaugeSetToCurrentTime(self, gauge_name):
        self.m_gauges[gauge_name].set_to_current_time()
        if self.m_publishMetrics:
            self.m_counters[gauge_name].publish()
        
    def createChronometer(self, chrono_name):
        if self.m_hasMetric(chrono_name):
            raise Exception("metric %s already exists" % chrono_name)
        self.m_chronometers[chrono_name] = Chronometer(self.m_root_metrics_topic_name + chrono_name)
    
    def chronoStart(self, chrono_name):
        self.m_chronometers[chrono_name].start()
    
    def chronoReset(self, chrono_name):
        self.m_chronometers[chrono_name].reset()
    
    def chronoStop(self, chrono_name):
        self.m_chronometers[chrono_name].stop()
        if self.m_publishMetrics:
            self.m_chronometers[chrono_name].publish()

    def m_hasMetric(self, metric_name):
        return self.m_counters.has_key(metric_name) \
            or self.m_gauges.has_key(metric_name) \
            or self.m_chronometers.has_key(metric_name)
    


