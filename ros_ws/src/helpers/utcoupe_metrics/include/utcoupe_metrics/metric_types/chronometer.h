#ifndef UTCOUPE_METRICS_METRIC_TYPES_CHRONOMETER_H
#define UTCOUPE_METRICS_METRIC_TYPES_CHRONOMETER_H

#include "utcoupe_metrics/metric_types/gauge.h"

namespace utcoupe_metrics {
    namespace metric_types {
        class Chronometer {
        public:
            Chronometer(ros::NodeHandle& nh, const std::string& topic_name):
                m_gauge({nh, topic_name}), m_started(false) {
                
            }
            
            void publish();
            
            void reset() noexcept { m_started = false; }
            
            void start();
            
            void stop();
            
        private:
            Gauge m_gauge;
            ros::Time m_startTime;
            bool m_started;
        };
    } // namespace metric_types
} // namespace utcoupe_metrics

#endif // UTCOUPE_METRICS_METRIC_TYPES_CHRONOMETER_H
