#ifndef DYNAMIC_BARRIERS_MANAGER_H
#define DYNAMIC_BARRIERS_MANAGER_H

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"

#include <memory>
#include <vector>

class PosConvertor;

class Point;

/**
 * Class managing the barriers subscribers.
 * It is mainly an interface between the pathfinder and the subscribers, converting the position when needed.
 */
class DynamicBarriersManager
{
public:
    using BarriersSubscriber = std::unique_ptr<AbstractBarriersSubscriber>;
    
    DynamicBarriersManager(const PosConvertor& convertor):
        _convertor(convertor) {}
    
    /**
     * Check if there is any dynamic obstacle on the given position.
     * @param pos The position to check.
     * @return True if there is at least one obstacle.
     */
    bool hasBarriers(const Point& pos) const;
    
    /**
     * Add an obstacle subscribers to the manager.
     * The class must be derivated from AbstractBarriersSubscriber, and the object already initialized.
     * @param subscriber The new subscriber to manage.
     */
    void addBarrierSubscriber(BarriersSubscriber&& subscriber);
    
    /**
     * Update the safety margin used in all subscribers.
     * @param newMargin The new margin to apply on barriers.
     */
    void updateSafetyMargin(const double& newMargin);
    void fetchOccupancyDatas(const uint& widthGrid, const uint& heightGrid) const;
    
private:
    std::vector< BarriersSubscriber > subscribers;
    const PosConvertor& _convertor;
};

#endif // DYNAMIC_BARRIERS_MANAGER_H
