#include "pathfinder/BarriersSubscribers/static_map_subscriber.h"

#include "pathfinder/pos_convertor.h"

#include <geometry_tools/point.h>
#include <static_map/MapGetContainer.h>

#include <cmath>
#include <utility>

using namespace Memory;
using namespace std;

MapSubscriber::MapSubscriber(double safetyMargin, const PosConvertor &convertor)
    : AbstractBarriersSubscriber(safetyMargin), _occupancyGrid(convertor)
{
}

bool Memory::MapSubscriber::hasBarrier(const Point &pos) const
{
    return !_occupancyGrid.isAllowed(pos);
}

void MapSubscriber::subscribe(ros::NodeHandle &nodeHandle, std::size_t sizeMaxQueue, std::string topic)
{
    _srvGetMapObjects = nodeHandle.serviceClient<static_map::MapGetContainer>(topic);
}

void MapSubscriber::fetchOccupancyData(const uint &widthGrid, const uint &heightGrid, const std::vector<std::string> &ignoredTags)
{
    _occupancyGrid.resize(heightGrid, widthGrid);
    static_map::MapGetContainer srv;
    srv.request.path = "map";
    srv.request.include_subcontainers = true;
    if (!_srvGetMapObjects.call(srv) || !srv.response.success)
    {
        ROS_ERROR("Error when trying to call static_map/get_container");
        return;
    }
    _occupancyGrid.setOccupancyFromMap(srv.response.container.objects, true, _safetyMargin, ignoredTags);
}
