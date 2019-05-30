#ifndef COLLISIONS_MARKERS_PUBLISHER_H
#define COLLISIONS_MARKERS_PUBLISHER_H

#include <ros/publisher.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>

namespace ros {
    class NodeHandle;
} // namespace ros

namespace CollisionsShapes {
class AbstractShape;
} // namespace CollisionsShapes
class Obstacle;
class Robot;

class MarkersPublisher {
public:
    MarkersPublisher(ros::NodeHandle& nhandle);
    
    void publishCheckZones(const Robot& robot);
    void publishObstacles(const std::vector<const Obstacle*>& obstacles);
    
private:
    ros::Publisher m_markersPubl;
    
    bool m_isConnected() const;
    void m_publishMarker(std::string ns, int index, const CollisionsShapes::AbstractShape& shape, double z_scale, double z_height, std_msgs::ColorRGBA color);
    
    void m_addSegmentInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const;
    void m_addRectangleInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const;
    void m_addCircleInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const;
};

#endif // COLLISIONS_MARKERS_PUBLISHER_H
