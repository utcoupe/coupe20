#ifndef ABSTRACT_BARRIERS_SUBSCRIBER
#define ABSTRACT_BARRIERS_SUBSCRIBER

#include <ros/ros.h>

#include "geometry_msgs/Pose2D.h"

#include <mutex>

/**
 * Class defining an abstract subscriber.
 */
class AbstractBarriersSubscriber
{
public:
    /**
     * Initialize the safety margin.
     * @param safetyMargin Margin to add to barriers.
     */
    AbstractBarriersSubscriber(const double& safetyMargin) : _safetyMargin(safetyMargin) {};
    
    /**
     * Check if there are any obstacles at the given position.
     * @param pos The position to check.
     */
    virtual bool hasBarrier(const geometry_msgs::Pose2D& pos) = 0;
    
    /**
     * Create internaly the ros subscriber.
     * @param nodeHandle The node handle to use for the subscriber.
     * @param sizeMaxQueue The size of the message queue (optional depending of the subscriber type).
     * @param topic The name of the topic (or service) to subscribe.
     */
    virtual void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) = 0;
    
    /**
     * Update the safety margin.
     * @param safetyMargin The new safety margin.
     */
    void setSafetyMargin(const double& safetyMargin) { _safetyMargin = safetyMargin; }
    
    /**
     * Before starting its algorithm, the pathfinder can ask the subscriber to make a cache of all current barriers.
     * For example, storing them into an array to have a constant access cost O(1).
     * @param widthGrid The width of pathfinder internal map.
     * @param heightGrid The height of pathfinder internal map.
     */
    virtual void fetchOccupancyData(const uint& widthGrid, const uint& heightGrid) {};
    
    /**
     * Indicate if the base coordinates used in the subscriber are different from the pathfinder.
     */
    const virtual bool needConversionBefore() const { return true; };

protected:
    std::mutex g_mutex;
    ros::Subscriber subscriber;
    double _safetyMargin;
};

#endif // ABSTRACT_BARRIERS_SUBSCRIBER
