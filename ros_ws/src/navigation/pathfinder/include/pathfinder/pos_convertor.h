#ifndef POS_CONVERTOR_H
#define POS_CONVERTOR_H

#include <geometry_tools/point.h>

/**
 * @brief This class provide functions to convert coordinates between ROS system and the pathfinding one 
 */
class PosConvertor
{
public:
    /**
     * @brief Initialize the convertor. Do nothing yet.
     */
    PosConvertor() = default;
    
    /**
     * @brief Converts a coodinate from ROS system to pathfinding system using the scales.
     * @param rosPos The coodinate in ROS system
     * @return The coordinate in pathfinder system
     */
    Point fromRosToMapPos (Point rosPos) const;
    
    /**
     * @brief Converts a coodinate from pathfinding system to ROS system using the scales.
     * @param mapPos The coodinate in pathfinder system
     * @return The coordinate in ROS system
     */
    Point fromMapToRosPos (Point mapPos) const;
    
    double fromMapToRosDistance (double dist) const;
    
    double fromRosToMapDistance (double dist) const;
    
    // Getters & Setters
    void setSizes (Point sizeRos, Point sizeMap) noexcept { setRosSize(sizeRos); setMapSize(sizeMap); }
    
    void setRosSize(Point sizeRos) noexcept { _sizeRos = sizeRos; }
    void setMapSize(Point sizeMap) noexcept { _sizeMap = sizeMap; }
    
    void setInvertedY(bool invertedY) noexcept { _invertedY = invertedY; }
    
private:
    bool _invertedY;
    Point _sizeRos, _sizeMap;
};

#endif // POS_CONVERTOR_H
