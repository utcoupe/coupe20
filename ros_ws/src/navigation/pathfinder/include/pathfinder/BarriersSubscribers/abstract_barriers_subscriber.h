#ifndef ABSTRACT_BARRIERS_SUBSCRIBER
#define ABSTRACT_BARRIERS_SUBSCRIBER

#include <ros/ros.h>

#include <mutex>
#include <string>
#include <vector>

class Point;

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
    AbstractBarriersSubscriber(double safetyMargin) : _safetyMargin(safetyMargin){};

    virtual ~AbstractBarriersSubscriber() = default;

    /**
     * Check if there are any obstacles at the given position.
     * @param pos The position to check.
     */
    virtual bool hasBarrier(const Point &pos) const = 0;

    /**
     * Create internaly the ros subscriber.
     * @param nodeHandle The node handle to use for the subscriber.
     * @param sizeMaxQueue The size of the message queue (optional depending of the subscriber type).
     * @param topic The name of the topic (or service) to subscribe.
     */
    virtual void subscribe(ros::NodeHandle &nodeHandle, std::size_t sizeMaxQueue, std::string topic) = 0;

    /**
     * Update the safety margin.
     * @param safetyMargin The new safety margin.
     */
    void setSafetyMargin(double safetyMargin) { _safetyMargin = safetyMargin; }

    /**
     * Before starting its algorithm, the pathfinder can ask the subscriber to make a cache of all current barriers.
     * For example, storing them into an array to have a constant access cost O(1).
     * @param widthGrid The width of pathfinder internal map.
     * @param heightGrid The height of pathfinder internal map.
     * @param ignoredTags Vector of tag names to ignore.
     */
    virtual void fetchOccupancyData(
        const uint &widthGrid,                      /** unused **/
        const uint &heightGrid,                     /** unused **/
        const std::vector<std::string> &ignoredTags /** unused **/
    ){};

    /**
     * Indicate if the base coordinates used in the subscriber are different from the pathfinder.
     */
    virtual bool needConversionBefore() const { return true; };

  protected:
    mutable std::mutex g_mutex;
    ros::Subscriber subscriber;
    double _safetyMargin;
};

#endif // ABSTRACT_BARRIERS_SUBSCRIBER
