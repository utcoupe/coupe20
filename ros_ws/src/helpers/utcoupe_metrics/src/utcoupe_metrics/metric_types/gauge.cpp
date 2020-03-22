#include "utcoupe_metrics/metric_types/gauge.h"

#include <std_msgs/Float64.h>

utcoupe_metrics::metric_types::Gauge::Gauge(ros::NodeHandle& nh, const std::string& topic_name, double start_value):
    m_value(start_value) {
    m_publisher = nh.advertise<std_msgs::Float64>(topic_name, 10);
}

void utcoupe_metrics::metric_types::Gauge::publish()
{
    std_msgs::Float64 msg;
    msg.data = m_value;
    m_publisher.publish(msg);
}

void utcoupe_metrics::metric_types::Gauge::setToCurrentTime()
{
    auto now = ros::Time::now();
    m_value = static_cast<double>(now.toNSec());
}

