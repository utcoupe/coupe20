#ifndef UTCOUPE_METRICS_METRIC_TYPES_GAUGE_H
#define UTCOUPE_METRICS_METRIC_TYPES_GAUGE_H

#include <ros/ros.h>

#include <string>

namespace utcoupe_metrics {
    namespace metric_types {
        class Gauge {
        public:
            Gauge(ros::NodeHandle& nh, const std::string& topic_name, double start_value = 0.0);
            
            void dec(double val_dec = 1.0) noexcept { inc(-val_dec); }
            
            void inc(double val_inc = 1.0) noexcept { m_value += val_inc; }
            
            void publish();
            
            void set(double value) noexcept { m_value = value; }
            
            void setToCurrentTime();
            
        private:
            double m_value;
            ros::Publisher m_publisher;
        };
    } // namespace metric_types
} // namespace utcoupe_metrics

#endif // UTCOUPE_METRICS_METRIC_TYPES_GAUGE_H
