#include "pathfinder/occupancy_grid.h"

#include "pathfinder/pos_convertor.h"

#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <utility>

using namespace std;

pathfinder::OccupancyGrid::OccupancyGrid(const PosConvertor& convertor, size_t nrow, size_t ncol):
    _convertor(convertor), _grid(nrow, vector<bool>(ncol, false))
{
}

void pathfinder::OccupancyGrid::clear()
{
    for (auto& row : _grid) {
        fill(row.begin(), row.end(), true);
    }
}

void pathfinder::OccupancyGrid::resize(size_t nrows, size_t ncols)
{
    if (!_grid.empty() && _grid.size() == nrows && _grid.front().size() == ncols) {
        return;
    }
    _grid.clear();
    _grid = vector<vector<bool>>(nrows, vector<bool>(ncols, false));
}

void pathfinder::OccupancyGrid::setOccupancyFromMap(const vector<static_map::MapObject>& objects, bool clearBefore, double safetyMargin)
{
    if (_grid.empty() || _grid.front().empty()){
        ROS_ERROR("[OccupancyGrid::setOccupancyFromMap] Trying to fill an empty grid!");
        return;
    }
    
    if (clearBefore) {
        clear();
    }
    
    for (auto&& mapObject : objects)
    {
        switch (mapObject.shape_type) {
            case static_map::MapObject::SHAPE_RECT:
                drawRectangle(mapObject, safetyMargin);
                break;
                
            case static_map::MapObject::SHAPE_CIRCLE:
                drawCircle(mapObject, safetyMargin);
                break;
            
            case static_map::MapObject::SHAPE_POINT:
                ROS_WARN_ONCE("[OccupancyGrid::setOccupancyFromMap] Point shape not supported!");
                break;
            
            default:
                ROS_ERROR("[OccupancyGrid::setOccupancyFromMap] Unknown shape!");
        }
    }
}

void pathfinder::OccupancyGrid::setOccupancyFromGrid(const std::vector<std::vector<bool> >& grid)
{
    if (grid.empty() || grid.front().empty()){
        ROS_WARN("[OccupancyGrid::setOccupancyFromMap] Creating an empty grid!");
    }
    
    _grid = grid;
}


void pathfinder::OccupancyGrid::drawCircle(const static_map::MapObject& objectCircle, double safetyMargin)
{
    double r;
    r = objectCircle.radius;
    
    auto pos = _convertor.fromRosToMapPos(objectCircle.pose);
    r = _convertor.fromRosToMapDistance(r);
    
    auto safeMarg = _convertor.fromRosToMapDistance(safetyMargin);
    
    uint yMin = max(pos.getY() - r - safeMarg, 0.0);
    uint yMax = min(
        static_cast<double>(_grid.size()),
        pos.getY() + r + safeMarg + 0.5
    );
    uint xMin = max(pos.getX() - r - safeMarg, 0.0);
    uint xMax = min(
        static_cast<double>(_grid.front().size()),
        pos.getX() + r + safeMarg + 0.5
    );
    
    for (uint row = yMin; row < yMax; row++)
        for (uint column = xMin; column < xMax; column++)
            if (pos.norm2Dist({static_cast<double>(column), static_cast<double>(row)}) <= r + safeMarg)
                _grid[row][column] = false;
}

void pathfinder::OccupancyGrid::drawRectangle(const static_map::MapObject& objectRect, double safetyMargin)
{
    double w, h;
    w = objectRect.width;
    h = objectRect.height;
    
    auto pos = _convertor.fromRosToMapPos(objectRect.pose);
    w = _convertor.fromRosToMapDistance(w);
    h = _convertor.fromRosToMapDistance(h);
    
    auto safeMarg = _convertor.fromRosToMapDistance(safetyMargin);
    
    uint yMin = max(pos.getY() - (h/2) - safeMarg, 0.0);
    uint yMax = min(
        static_cast<double>(_grid.size()),
        pos.getY() + (h/2) + safeMarg + 0.5
    );
    uint xMin = max(pos.getX() - (w/2) - safeMarg, 0.0);
    uint xMax = min(
        static_cast<double>(_grid.front().size()),
        pos.getX() + (w/2) + safeMarg + 0.5
    );
    
    for (uint row = yMin; row < yMax; row++)
        for (uint column = xMin; column < xMax; column++)
            _grid[row][column] = false;
}



