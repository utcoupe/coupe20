#ifndef DYNAMIC_BARRIERS_MANAGER_H
#define DYNAMIC_BARRIERS_MANAGER_H

#include "pathfinder/point.h"
#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "pathfinder/pos_convertor.h"

#include "geometry_msgs/Pose2D.h"

#include <memory>
#include <vector>

/**
 * Class managing the barriers subscribers.
 * It is mainly an interface between the pathfinder and the subscribers, converting the position when needed.
 */
class DynamicBarriersManager
{
public:
    using BarriersSubscriber = std::unique_ptr<AbstractBarriersSubscriber>;
    
    DynamicBarriersManager() = default;
    
    /**
     * Check if there is any dynamic obstacle on the given position.
     * @param pos The position to check.
     * @return True if there is at least one obstacle.
     */
    bool hasBarriers(const geometry_msgs::Pose2D& pos);
    /**
     * Check if there is any dynamic obstacle on the given position.
     * @param pos The position to check.
     * @return True if there is at least one obstacle.
     */
    bool hasBarriers(const Point& pos);
    
    /**
     * Add an obstacle subscribers to the manager.
     * The class must be derivated from AbstractBarriersSubscriber, and the object already initialized.
     * @param subscriber The new subscriber to manage.
     */
    void addBarrierSubscriber(BarriersSubscriber&& subscriber);
    
    /**
     * Set the convertor to use with the subscribers. It must be already initialized.
     * @param convertor The new convertor.
     */
    void setConvertor(std::shared_ptr<PosConvertor> convertor);
    
    /**
     * Update the safety margin used in all subscribers.
     * @param newMargin The new margin to apply on barriers.
     */
    void updateSafetyMargin(const double& newMargin);
    void fetchOccupancyDatas(const uint& widthGrid, const uint& heightGrid) const;
    
private:
    std::vector< BarriersSubscriber > subscribers;
    std::shared_ptr<PosConvertor> _convertor;
    
    /**
     * Converts a position from the inside referential and type to the outside ones.
     * @param pos The position in the inside referential and type.
     * @return The position in the outside referential and type.
     */
    geometry_msgs::Pose2D pointToPose2D(const Point& pos) const;
};

#endif // DYNAMIC_BARRIERS_MANAGER_H
