#include "utcoupe_metrics/metric_exporter.h"


#include <ros/ros.h>

#include <exception>

using namespace utcoupe_metrics;

std::string ROOT_METRICS_TOPIC_NAME { "/metrics/" };

utcoupe_metrics::MetricExporter::MetricExporter(ros::NodeHandle& nh, const std::string& node_name, bool publishMetrics):
    m_publishMetrics(publishMetrics), m_nodeHandle(nh) {
    m_rootMetricTopicName = ROOT_METRICS_TOPIC_NAME + node_name + "/";
    
    ROS_INFO_STREAM("Creating metric exporter for node " << node_name);
}

void utcoupe_metrics::MetricExporter::chronometerReset(const std::string& chrono_name)
{
    m_chronometers[chrono_name]->reset();
}

void utcoupe_metrics::MetricExporter::chronometerStart(const std::string& chrono_name)
{
    m_chronometers[chrono_name]->start();
}

void utcoupe_metrics::MetricExporter::chronometerStop(const std::string& chrono_name)
{
    m_chronometers[chrono_name]->stop();
    if (m_publishMetrics) {
        m_chronometers[chrono_name]->publish();
    }
}

void utcoupe_metrics::MetricExporter::counterIncrement(const std::string& counter_name, double inc_val)
{
    m_counters[counter_name]->inc(inc_val);
    if (m_publishMetrics) {
        m_counters[counter_name]->publish();
    }
}


void utcoupe_metrics::MetricExporter::createChronometer(const std::string& chrono_name)
{
    m_ensureMetricDoesntExists(chrono_name);
    
    m_chronometers[chrono_name] = std::make_unique<metric_types::Chronometer>(
        m_nodeHandle,
        m_rootMetricTopicName + chrono_name
    );
    if (m_publishMetrics) {
        m_chronometers[chrono_name]->publish();
    }
}

void utcoupe_metrics::MetricExporter::createCounter(const std::string& counter_name, double start_val) {
    m_ensureMetricDoesntExists(counter_name);
    
    m_counters[counter_name] = std::make_unique<metric_types::Counter>(
        m_nodeHandle,
        m_rootMetricTopicName + counter_name,
        start_val
    );
    if (m_publishMetrics) {
        m_counters[counter_name]->publish();
    }
}

void utcoupe_metrics::MetricExporter::createGauge(const std::string& gauge_name, double start_val)
{
    m_ensureMetricDoesntExists(gauge_name);
    
    m_gauges[gauge_name] = std::make_unique<metric_types::Gauge>(
        m_nodeHandle,
        m_rootMetricTopicName + gauge_name,
        start_val
    );
    if (m_publishMetrics) {
        m_gauges[gauge_name]->publish();
    }
}

void utcoupe_metrics::MetricExporter::gaugeDecrement(const std::string& gauge_name, double dec_val)
{
    gaugeIncrement(gauge_name, -dec_val);
}

void utcoupe_metrics::MetricExporter::gaugeIncrement(const std::string& gauge_name, double inc_val)
{
    m_gauges[gauge_name]->inc(inc_val);
    if (m_publishMetrics) {
        m_gauges[gauge_name]->publish();
    }
}

void utcoupe_metrics::MetricExporter::gaugeSet(const std::string& gauge_name, double value)
{
    m_gauges[gauge_name]->set(value);
    if (m_publishMetrics) {
        m_gauges[gauge_name]->publish();
    }
}

void utcoupe_metrics::MetricExporter::gaugeSetToCurrentTime(const std::string& gauge_name)
{
    m_gauges[gauge_name]->setToCurrentTime();
    if (m_publishMetrics) {
        m_gauges[gauge_name]->publish();
    }
}

// ************************* Private member definitions ***********************

void utcoupe_metrics::MetricExporter::m_ensureMetricDoesntExists(const std::string& metricName)
{
    if (m_hasMetric(metricName)) {
        throw std::runtime_error("metric already exists");
    }
}


bool utcoupe_metrics::MetricExporter::m_hasMetric(const std::string metricName)
{
    return m_chronometers.find(metricName) != m_chronometers.end()
        || m_counters.find(metricName) != m_counters.end()
        || m_gauges.find(metricName) != m_gauges.end();
}
