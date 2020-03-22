#ifndef UTCOUPE_METRICS_METRIC_TYPES_COUNTER_H
#define UTCOUPE_METRICS_METRIC_TYPES_COUNTER_H

#include <ros/ros.h>

#include <string>

namespace utcoupe_metrics {
    namespace metric_types {
        class Counter {
        public:
            Counter(ros::NodeHandle& nh, const std::string& topic_name, double start_value = 0.0);
            
        
            void inc(double val_inc = 1.0);
            
            void publish();
            
        private:
            double m_value;
            ros::Publisher m_publisher;
        };
    } // namespace metric_types
} // namespace utcoupe_metrics

#endif // UTCOUPE_METRICS_METRIC_TYPES_COUNTER_H
