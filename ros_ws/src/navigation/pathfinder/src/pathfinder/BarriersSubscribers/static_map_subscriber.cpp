#include "pathfinder/BarriersSubscribers/static_map_subscriber.h"

#include "static_map/MapGetContainer.h"

#include <cmath>

using namespace Memory;
using namespace std;

MapSubscriber::MapSubscriber(double safetyMargin, shared_ptr<PosConvertor> convertor)
    : AbstractBarriersSubscriber(safetyMargin), _convertor(convertor)
{
}

bool MapSubscriber::hasBarrier(Point pos)
{
    return _occupancyGrid[pos.getY()][pos.getX()];
}

void MapSubscriber::subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic)
{
    _srvGetMapObjects = nodeHandle.serviceClient<static_map::MapGetContainer>(topic);
}

void MapSubscriber::fetchOccupancyData(const uint& widthGrid, const uint& heightGrid)
{
    if (_occupancyGrid.size() != heightGrid || (heightGrid != 0 && _occupancyGrid.front().size() != widthGrid))
        _occupancyGrid = vector< vector<bool> > (
            heightGrid,
            vector<bool>(widthGrid, false)
        );
    else
        for (unsigned row = 0; row < _occupancyGrid.size(); row++)
            for (unsigned column = 0; column < _occupancyGrid.front().size(); column++)
                _occupancyGrid[row][column] = false;

    static_map::MapGetContainer srv;
    srv.request.path = "map";
    srv.request.include_subcontainers = true;
    if (!_srvGetMapObjects.call(srv) || !srv.response.success)
    {
        ROS_ERROR("Error when trying to call static_map/get_container");
        return;
    }
    _lastReceivedContainer = srv.response.container;
    
    for (auto&& mapObject : _lastReceivedContainer.objects)
    {
        switch (mapObject.shape_type) {
            case static_map::MapObject::SHAPE_RECT:
                drawRectangle(mapObject);
                break;
            case static_map::MapObject::SHAPE_CIRCLE:
                drawCircle(mapObject);
                break;
            
            case static_map::MapObject::SHAPE_POINT:
                ROS_WARN_ONCE("[MapSubscriber::fetchOccupancyData] Point shape not supported!");
                break;
            
            default:
                ROS_ERROR("[MapSubscriber::fetchOccupancyData] Unknown shape!");
        }
    }
}

void Memory::MapSubscriber::drawRectangle(const static_map::MapObject& objectRect)
{
    double w, h;
    w = objectRect.width;
    h = objectRect.height;
    
    auto pos = _convertor->fromRosToMapPos(objectRect.pose);
    w = _convertor->fromRosToMapDistance(w);
    h = _convertor->fromRosToMapDistance(h);
    
    auto safeMarg = _convertor->fromRosToMapDistance(_safetyMargin);
    
    uint yMin = max(pos.getY() - (h/2) - safeMarg, 0.0);
    uint yMax = min(
        static_cast<double>(_occupancyGrid.size()),
        pos.getY() + (h/2) + safeMarg + 0.5
    );
    uint xMin = max(pos.getX() - (w/2) - safeMarg, 0.0);
    uint xMax = min(
        static_cast<double>(_occupancyGrid.front().size()),
        pos.getX() + (w/2) + safeMarg + 0.5
    );
    
    for (uint row = yMin; row < yMax; row++)
        for (uint column = xMin; column < xMax; column++)
            _occupancyGrid[row][column] = true;
}

void Memory::MapSubscriber::drawCircle(const static_map::MapObject& objectCircle)
{
    double r;
    r = objectCircle.radius;
    
    auto pos = _convertor->fromRosToMapPos(objectCircle.pose);
    r = _convertor->fromRosToMapDistance(r);
    
    auto safeMarg = _convertor->fromRosToMapDistance(_safetyMargin);
    
    uint yMin = max(pos.getY() - r - safeMarg, 0.0);
    uint yMax = min(
        static_cast<double>(_occupancyGrid.size()),
        pos.getY() + r + safeMarg + 0.5
    );
    uint xMin = max(pos.getX() - r - safeMarg, 0.0);
    uint xMax = min(
        static_cast<double>(_occupancyGrid.front().size()),
        pos.getX() + r + safeMarg + 0.5
    );
    
    for (uint row = yMin; row < yMax; row++)
        for (uint column = xMin; column < xMax; column++)
            if (pos.norm2Dist({static_cast<double>(column), static_cast<double>(row)}) <= r)
                _occupancyGrid[row][column] = true;
}

