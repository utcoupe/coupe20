#include "pathfinder/BarriersSubscribers/static_map_subscriber.h"

#include "static_map/MapGetContainer.h"

#include <cmath>

using namespace Memory;
using namespace std;

MapSubscriber::MapSubscriber(double safetyMargin, shared_ptr<PosConvertor> convertor)
    : AbstractBarriersSubscriber(safetyMargin), _occupancyGrid(convertor), _convertor(convertor)
{
}

bool MapSubscriber::hasBarrier(Point pos)
{
    return _occupancyGrid.isOccupied(pos);
}

void MapSubscriber::subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic)
{
    _srvGetMapObjects = nodeHandle.serviceClient<static_map::MapGetContainer>(topic);
}

void MapSubscriber::fetchOccupancyData(const uint& widthGrid, const uint& heightGrid)
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
    _occupancyGrid.setOccupancyFromMap(srv.response.container.objects, _safetyMargin);
}

