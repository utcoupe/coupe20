#ifndef POS_CONVERTOR_H
#define POS_CONVERTOR_H

#include <algorithm>

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
    std::pair<double,double> fromRosToMapPos (const std::pair<double,double>& rosPos) const;
    
    /**
     * @brief Converts a coodinate from pathfinding system to ROS system using the scales.
     * @param mapPos The coodinate in pathfinder system
     * @return The coordinate in ROS system
     */
    std::pair<double,double> fromMapToRosPos (const std::pair<double,double>& mapPos) const;
    
    double fromMapToRosDistance (const double& dist) const;
    
    double fromRosToMapDistance (const double& dist) const;
    
    // Getters & Setters
    void setSizes (std::pair<double,double> sizeRos, std::pair<double,double> sizeMap);
    
    void setInvertedY(bool invertedY) { _invertedY = invertedY; }
    
private:
    bool _invertedY;
    std::pair<double,double> _sizeRos, _sizeMap;
};

#endif // POS_CONVERTOR_H
