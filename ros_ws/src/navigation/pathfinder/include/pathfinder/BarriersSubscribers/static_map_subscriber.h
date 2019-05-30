#ifndef STATIC_MAP_SUBSCRIBER_H
#define STATIC_MAP_SUBSCRIBER_H

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "pathfinder/occupancy_grid.h"

#include <static_map/MapContainer.h>

#include <vector>

class PosConvertor;

namespace Memory
{
class MapSubscriber : public AbstractBarriersSubscriber
{
  public:
    MapSubscriber(double safetyMargin, const PosConvertor &convertor);

    bool hasBarrier(const Point &pos) const override;
    void subscribe(ros::NodeHandle &nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;

    void fetchOccupancyData(
        const uint &widthGrid,
        const uint &heightGrid,
        const std::vector<std::string> &ignoredTags) override;
    bool needConversionBefore() const override { return false; }

  private:
    static_map::MapContainer _lastReceivedContainer;
    ros::ServiceClient _srvGetMapObjects;
    pathfinder::OccupancyGrid _occupancyGrid;
};
} // namespace Memory

#endif // STATIC_MAP_SUBSCRIBER_H
