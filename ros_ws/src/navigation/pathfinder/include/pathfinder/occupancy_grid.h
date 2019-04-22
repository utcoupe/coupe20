#ifndef PATHFINDER_OCCUPATION_GRID
#define PATHFINDER_OCCUPATION_GRID

#include <geometry_tools/point.h>
#include <static_map/MapObject.h>

#include <vector>

class PosConvertor;

namespace pathfinder {
    /**
     * Tool to manipulate an occupancy grid.
     */
    class OccupancyGrid {
    public:
        OccupancyGrid(const PosConvertor& convertor, std::size_t nrows = 0, std::size_t ncols = 0);
        
        std::pair<std::size_t, std::size_t> getSize() const noexcept {
            if (_grid.size() > 0)
                return { _grid.front().size(), _grid.size() };
            else
                return {};
        }
        
        void clear();
        
        void resize(std::size_t nrows, std::size_t ncols);
        
        bool isAllowed(Point pos) const {
            return isAllowed(static_cast<unsigned>(pos.getX()), static_cast<unsigned>(pos.getY()));
        }
        
        bool isAllowed(unsigned x, unsigned y) const {
            if (_grid.size() > 0)
                return _grid[y][x];
            else
                return true; // TODO as editable attribute
        }
        
        bool empty() const noexcept {
            return _grid.empty() || _grid.front().empty();
        }
        
        void setAllowed(Point pos, bool allowed) { _grid[pos.getY()][pos.getX()] = allowed; }
        
        void setOccupancyFromMap(const std::vector<static_map::MapObject>& objects, bool clearBefore = false, double safetyMargin = 0.0);
        
        void setOccupancyFromGrid(const std::vector<std::vector<bool>>& grid);
    
    protected:
        const PosConvertor& _convertor;
        std::vector<std::vector<bool>> _grid;
        
        void drawRectangle(const static_map::MapObject& objectRect, double safetyMargin);
        void drawCircle(const static_map::MapObject& objectCircle, double safetyMargin);
    };
} // namespace Pathfinder

#endif // PATHFINDER_OCCUPATION_GRID
