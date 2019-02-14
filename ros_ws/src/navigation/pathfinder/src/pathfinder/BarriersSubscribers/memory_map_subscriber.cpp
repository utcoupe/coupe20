#include "pathfinder/BarriersSubscribers/static_map_subscriber.h"

#include "static_map/MapGetContainer.h"

#include <cmath>

using namespace Memory;
using namespace std;

inline double getNorme2Distance(const double& x1, const double& y1, const double& x2, const double& y2)
{
    double dX = x1 - x2;
    double dY = y1 - y2;
    return sqrt(dX*dX + dY*dY);
}

MapSubscriber::MapSubscriber(const double& safetyMargin)
    : AbstractBarriersSubscriber(safetyMargin)
{
}

bool MapSubscriber::hasBarrier(const geometry_msgs::Pose2D& pos)
{
    return _occupancyGrid[pos.y][pos.x];
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
    double x, y, w, h;
    x = objectRect.pose.x;
    y = objectRect.pose.y;
    w = objectRect.width;
    h = objectRect.height;
    
    auto pos = _convertor->fromRosToMapPos(make_pair(x, y));
    w = _convertor->fromRosToMapDistance(w);
    h = _convertor->fromRosToMapDistance(h);
    
    auto safeMarg = _convertor->fromRosToMapDistance(_safetyMargin);
    
    uint yMin = max(pos.second - (h/2) - safeMarg, 0.0);
    uint yMax = min((double)_occupancyGrid.size(), pos.second + (h/2) + safeMarg + 0.5);
    uint xMin = max(pos.first - (w/2) - safeMarg, 0.0);
    uint xMax = min((double)_occupancyGrid.front().size(), pos.first + (w/2) + safeMarg + 0.5);
    
    for (uint row = yMin; row < yMax; row++)
        for (uint column = xMin; column < xMax; column++)
            _occupancyGrid[row][column] = true;
}

void Memory::MapSubscriber::drawCircle(const static_map::MapObject& objectCircle)
{
    double x, y, r;
    x = objectCircle.pose.x;
    y = objectCircle.pose.y;
    r = objectCircle.radius;
    
    auto pos = _convertor->fromRosToMapPos(make_pair(x, y));
    r = _convertor->fromRosToMapDistance(r);
    
    auto safeMarg = _convertor->fromRosToMapDistance(_safetyMargin);
    
    uint yMin = max(pos.second - r - safeMarg, 0.0);
    uint yMax = min((double)_occupancyGrid.size(), pos.second + r + safeMarg + 0.5);
    uint xMin = max(pos.first - r - safeMarg, 0.0);
    uint xMax = min((double)_occupancyGrid.front().size(), pos.first + r + safeMarg + 0.5);
    
    for (uint row = yMin; row < yMax; row++)
        for (uint column = xMin; column < xMax; column++)
            if (getNorme2Distance(column, row, pos.first, pos.second) <= r)
                _occupancyGrid[row][column] = true;
}

