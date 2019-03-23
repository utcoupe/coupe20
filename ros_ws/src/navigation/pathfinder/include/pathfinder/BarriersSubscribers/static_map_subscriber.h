#ifndef STATIC_MAP_SUBSCRIBER_H
#define STATIC_MAP_SUBSCRIBER_H

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "pathfinder/pos_convertor.h"
#include "pathfinder/occupancy_grid.h"

#include <static_map/MapContainer.h>

#include <vector>

namespace Memory {
    class MapSubscriber : public AbstractBarriersSubscriber
    {
    public:
        MapSubscriber(double safetyMargin): MapSubscriber(safetyMargin, nullptr) {}
        MapSubscriber(double safetyMargin, std::shared_ptr<PosConvertor> convertor);
        
        bool hasBarrier(Point pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
        void fetchOccupancyData(const uint& withGrid, const uint& heightGrid) override;
        const bool needConversionBefore() const  override { return false; }
        
        void setConvertor(std::shared_ptr<PosConvertor> convertor) {
            _convertor = convertor;
            _occupancyGrid.setConvertor(convertor);
        };
        
    private:
        pathfinder::OccupancyGrid _occupancyGrid;
        ros::ServiceClient _srvGetMapObjects;
        
        std::shared_ptr<PosConvertor> _convertor;
    };
}

#endif // STATIC_MAP_SUBSCRIBER_H
