#ifndef COLLISIONS_MARKERS_PUBLISHER_H
#define COLLISIONS_MARKERS_PUBLISHER_H

#include "collisions/obstacle.h"
#include "collisions/robot.h"

#include <ros/node_handle.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>

class MarkersPublisher {
public:
    using RobotPtr = std::shared_ptr<Robot>;
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    using ShapePtr = std::unique_ptr<CollisionsShapes::AbstractShape>;
    
    MarkersPublisher(ros::NodeHandle& nhandle);
    
    void publishCheckZones(RobotPtr robot);
    void publishObstacles(const std::vector<ObstaclePtr>& obstacles);
    
private:
    ros::Publisher m_markersPubl;
    
    bool m_isConnected() const;
    void m_publishMarker(std::string ns, int index, const CollisionsShapes::AbstractShape& shape, double z_scale, double z_height, std_msgs::ColorRGBA color);
    
    void m_addSegmentInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const;
    void m_addRectangleInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const;
    void m_addCircleInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const;
};

#endif // COLLISIONS_MARKERS_PUBLISHER_H
