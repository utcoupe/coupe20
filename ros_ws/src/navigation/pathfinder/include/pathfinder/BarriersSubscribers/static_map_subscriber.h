#ifndef STATIC_MAP_SUBSCRIBER_H
#define STATIC_MAP_SUBSCRIBER_H

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "pathfinder/pos_convertor.h"

#include "static_map/MapContainer.h"

#include <unordered_set>
#include <vector>

namespace Memory {
    class MapSubscriber : public AbstractBarriersSubscriber
    {
    public:
        MapSubscriber(const double& safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
        void fetchOccupancyData(
            const uint& widthGrid,
            const uint& heightGrid,
            const std::vector<std::string>& ignoredTags
        ) override;
        const bool needConversionBefore() const  override { return false; }
        
        void setConvertor(std::shared_ptr<PosConvertor> convertor) { _convertor = convertor; };
        
    private:
        using TagsList = std::vector<std::string>;
        using TagsSet = std::unordered_set<std::string>;
        
        static_map::MapContainer _lastReceivedContainer;
        ros::ServiceClient _srvGetMapObjects;
        std::vector< std::vector<bool> > _occupancyGrid;
        
        std::shared_ptr<PosConvertor> _convertor;
        
        void drawRectangle(const static_map::MapObject& objectRect);
        void drawCircle(const static_map::MapObject& objectCircle);
        bool objectIgnored(const TagsList& objectTags, const TagsSet& tagsToIgnore);
    };
}

#endif // STATIC_MAP_SUBSCRIBER_H
