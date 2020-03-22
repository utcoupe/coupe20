#ifndef UTCOUPE_METRICS_METRIC_EXPORTER_H
#define UTCOUPE_METRICS_METRIC_EXPORTER_H

#include "utcoupe_metrics/metric_types/chronometer.h"
#include "utcoupe_metrics/metric_types/counter.h"
#include "utcoupe_metrics/metric_types/gauge.h"

#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace utcoupe_metrics {
    class MetricExporter {
    public:
        MetricExporter(ros::NodeHandle& nh, const std::string& node_name, bool publishMetrics = true);
        
        void activateMetrics() noexcept { m_publishMetrics = true;}
        
        void chronometerReset(const std::string& chrono_name);
        
        void chronometerStart(const std::string& chrono_name);
        
        void chronometerStop(const std::string& chrono_name);
        
        void counterIncrement(const std::string& counter_name, double inc_val = 1.0);
        
        void createChronometer(const std::string& chrono_name);
        
        void createCounter(const std::string& counter_name, double start_val = 0.0);
        
        void createGauge(const std::string& gauge_name, double start_val = 0.0);
        
        void deactivateMetrics() noexcept { m_publishMetrics = false; }
        
        void gaugeDecrement(const std::string& gauge_name, double dec_val = 1.0);
        
        void gaugeIncrement(const std::string& gauge_name, double inc_val = 1.0);
        
        void gaugeSet(const std::string& gauge_name, double value);
        
        void gaugeSetToCurrentTime(const std::string& gauge_name);
        
    
    private:
        bool m_publishMetrics;
        std::string m_rootMetricTopicName;
        ros::NodeHandle& m_nodeHandle;
        
        std::unordered_map<std::string, std::unique_ptr<metric_types::Chronometer>> m_chronometers;
        std::unordered_map<std::string, std::unique_ptr<metric_types::Counter>> m_counters;
        std::unordered_map<std::string, std::unique_ptr<metric_types::Gauge>> m_gauges;
        
        void m_ensureMetricDoesntExists(const std::string& metricName);
        
        bool m_hasMetric(const std::string metricName);
        
    };
} // namespace utcoupe_metrics

#endif // UTCOUPE_METRICS_METRIC_EXPORTER_H
