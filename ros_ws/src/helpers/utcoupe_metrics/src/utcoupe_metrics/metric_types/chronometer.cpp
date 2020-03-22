#include "utcoupe_metrics/metric_types/chronometer.h"

#include <exception>

void utcoupe_metrics::metric_types::Chronometer::publish()
{
    m_gauge.publish();
}


void utcoupe_metrics::metric_types::Chronometer::start() {
    m_started = true;
    m_startTime = ros::Time::now();
}

void utcoupe_metrics::metric_types::Chronometer::stop()
{
    if (!m_started) {
        throw std::runtime_error("chronometer hasn't been started");
    }
    auto deltaT = ros::Time::now() - m_startTime;
    m_gauge.set(static_cast<double>(deltaT.toNSec()));
}
