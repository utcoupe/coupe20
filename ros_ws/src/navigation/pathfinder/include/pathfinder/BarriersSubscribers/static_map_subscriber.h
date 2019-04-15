#ifndef STATIC_MAP_SUBSCRIBER_H
#define STATIC_MAP_SUBSCRIBER_H

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "pathfinder/occupancy_grid.h"

#include <static_map/MapContainer.h>

#include <vector>

class PosConvertor;

namespace Memory {
    class MapSubscriber : public AbstractBarriersSubscriber
    {
    public:
        MapSubscriber(double safetyMargin, const PosConvertor& convertor);
        
        bool hasBarrier(const Point& pos) const override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
        void fetchOccupancyData(const uint& withGrid, const uint& heightGrid) override;
        const bool needConversionBefore() const  override { return false; }
        
    private:
        pathfinder::OccupancyGrid _occupancyGrid;
        ros::ServiceClient _srvGetMapObjects;
    };
}

#endif // STATIC_MAP_SUBSCRIBER_H
