#ifndef MEMORY_MAP_SUBSCRIBER_H
#define MEMORY_MAP_SUBSCRIBER_H

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "pathfinder/pos_convertor.h"

#include "nlohmann/json.hpp"

#include <vector>

namespace Memory {
    class MapSubscriber : public AbstractBarriersSubscriber
    {
    public:
        MapSubscriber(const double& safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
        void fetchOccupancyData(const uint& withGrid, const uint& heightGrid) override;
        const bool needConversionBefore() const  override { return false; }
        
        void setConvertor(std::shared_ptr<PosConvertor> convertor) { _convertor = convertor; };
        
    private:
        std::vector<nlohmann::json> _lastReceivedJsons;
        ros::ServiceClient _srvGetMapObjects;
        std::vector< std::vector<bool> > _occupancyGrid;
        
        std::shared_ptr<PosConvertor> _convertor;
        
        void drawRectangle(const nlohmann::json& jsonRect);
        void drawCircle(const nlohmann::json& jsonCircle);
    };
}

#endif // MEMORY_MAP_SUBSCRIBER_H
