#include "utcoupe_metrics/metric_types/counter.h"

#include <std_msgs/Float64.h>

#include <exception>

using namespace utcoupe_metrics;

metric_types::Counter::Counter(ros::NodeHandle & nh, const std::string& topic_name, double start_value):
    m_value(start_value) {
    m_publisher = nh.advertise<std_msgs::Float64>(topic_name, 10);
}

void utcoupe_metrics::metric_types::Counter::inc(double val_inc)
{
    if (val_inc < 0) {
        throw std::runtime_error("a counter cannot decrease its value");
    }
    m_value += val_inc;
}

void utcoupe_metrics::metric_types::Counter::publish()
{
    std_msgs::Float64 msg;
    msg.data = m_value;
    m_publisher.publish(msg);
}
