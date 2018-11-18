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
    using ShapePtr = std::shared_ptr<CollisionsShapes::AbstractShape>;
    
    MarkersPublisher(ros::NodeHandle& nhandle);
    
    void publishCheckZones(RobotPtr robot);
    void publishObstacles(const std::vector<ObstaclePtr>& obstacles);
    
private:
    ros::Publisher markersPubl_;
    
    bool isConnected() const;
    void publishMarker(std::string ns, unsigned index, ShapePtr shape, double z_scale, double z_height, std_msgs::ColorRGBA color);
    
    void addSegmentInfoToMarker(ShapePtr shape, visualization_msgs::Marker& marker);
    void addRectangleInfoToMarker(ShapePtr shape, visualization_msgs::Marker& marker);
    void addCircleInfoToMarker(ShapePtr shape, visualization_msgs::Marker& marker);
};

#endif // COLLISIONS_MARKERS_PUBLISHER_H
