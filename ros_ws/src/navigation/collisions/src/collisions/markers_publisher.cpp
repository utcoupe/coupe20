#include "collisions/markers_publisher.h"

#include "geometry_tools/position.h"
#include "collisions/shapes/segment.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/circle.h"

#include <geometry_msgs/Quaternion.h>
#include <ros/console.h>

#include <cmath>

const std::string MARKERS_TOPIC = "/visualization_markers/navigation";
const std::size_t QUEUE_SIZE    = 10;

const std::string NS_MARKERS_MAIN       = "collisions_main";
const std::string NS_MARKERS_PATH       = "collisions_path";
const std::string NS_MARKERS_OBSTACLES  = "collisions_obstacles";

const std_msgs::ColorRGBA constructColorRGBA(float r, float g, float b, float a) noexcept;

const std_msgs::ColorRGBA COLOR_ROBOT_MAIN_SHAPES           = constructColorRGBA(1.0, 0.0, 0.0, 0.8);
const std_msgs::ColorRGBA COLOR_ROBOT_PATH_SHAPES           = constructColorRGBA(1.0, 0.5, 0.0, 0.8);
const std_msgs::ColorRGBA COLOR_OBSTACLES_SHAPES            = constructColorRGBA(1.0, 0.8, 0.3, 0.8);
const std_msgs::ColorRGBA COLOR_OBSTACLE_VELOCITY_SHAPES    = constructColorRGBA(1.0, 0.0, 0.0, 0.8);

const std::string MARKER_FRAME_ID = "/map";

geometry_msgs::Quaternion eulerToQuaternion(Position pos) noexcept;

MarkersPublisher::MarkersPublisher(ros::NodeHandle& nhandle) {
    m_markersPubl = nhandle.advertise<visualization_msgs::Marker>(MARKERS_TOPIC, QUEUE_SIZE);
}

void MarkersPublisher::publishCheckZones(RobotPtr robot) {
    if (!m_isConnected())
        return;
    int index = 0;
    for (const auto& shape: robot->getMainShapes()) {
        m_publishMarker(NS_MARKERS_MAIN, index, *shape, 0.02, 0.01, COLOR_ROBOT_MAIN_SHAPES);
        index++;
    }
    ROS_DEBUG_STREAM_THROTTLE(1, "Published " << index << " robot shapes.");
    index = 0;
    for (const auto& shape: robot->getPathShapes()) {
        m_publishMarker(NS_MARKERS_PATH, index, *shape, 0.02, 0.01, COLOR_ROBOT_PATH_SHAPES);
        index++;
    }
    ROS_DEBUG_STREAM_THROTTLE(1, "Published " << index << " path shapes.");
}

void MarkersPublisher::publishObstacles(const std::vector<ObstaclePtr>& obstacles) {
    if (!m_isConnected())
        return;
    int index = 0;
    for (auto obstacle: obstacles) {
        m_publishMarker(NS_MARKERS_OBSTACLES, index, obstacle->getShape(), 0.35, 0.35 / 2.0, COLOR_OBSTACLES_SHAPES);
        index++;
        
        for (const auto& velShape: obstacle->getVelocityShapes()) {
            m_publishMarker(NS_MARKERS_OBSTACLES, index, *velShape, 0.02, 0.01, COLOR_OBSTACLE_VELOCITY_SHAPES);
            index++;
        }
    }
    ROS_DEBUG_STREAM_THROTTLE(1, "Published " << index << " obstacle shapes.");
}

bool MarkersPublisher::m_isConnected() const {
    return m_markersPubl.getNumSubscribers() > 0;
}

void MarkersPublisher::m_publishMarker(std::string ns, int index, const CollisionsShapes::AbstractShape& shape, double z_scale, double z_height, std_msgs::ColorRGBA color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = MARKER_FRAME_ID;
    marker.ns = ns;
    marker.id = index;
    
    switch (shape.getShapeType()) {
    case CollisionsShapes::ShapeType::SEGMENT:
        m_addSegmentInfoToMarker(shape, marker);
        break;
    case CollisionsShapes::ShapeType::RECTANGLE:
        m_addRectangleInfoToMarker(shape, marker);
        break;
    case CollisionsShapes::ShapeType::CIRCLE:
        m_addCircleInfoToMarker(shape, marker);
        break;
    default:
        ROS_WARN_ONCE("Tried to publish marker for an unknown shape. This message will be printed once.");
    }
    
    marker.scale.z = z_scale;
    marker.color = color;
    marker.pose.position = shape.getPos().toGeoPoint();
    marker.pose.position.z = z_height;
    marker.pose.orientation = eulerToQuaternion({0, 0, shape.getPos().getAngle()});
    marker.lifetime = ros::Duration(0.1);
    
    m_markersPubl.publish(marker);
}

void MarkersPublisher::m_addSegmentInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const {
    const auto& seg = dynamic_cast<const CollisionsShapes::Segment&>(shape);
    
    marker.type = marker.CUBE;
    marker.scale.x = seg.getLength();
    marker.scale.y = 0.02;
}

void MarkersPublisher::m_addRectangleInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const {
    const auto& rect = dynamic_cast<const CollisionsShapes::Rectangle&>(shape);
    
    marker.type = marker.CUBE;
    marker.scale.x = rect.getWidth();
    marker.scale.y = rect.getHeight();
}

void MarkersPublisher::m_addCircleInfoToMarker(const CollisionsShapes::AbstractShape& shape, visualization_msgs::Marker& marker) const {
    const auto& circ = dynamic_cast<const CollisionsShapes::Circle&>(shape);
    
    marker.type = marker.CYLINDER;
    marker.scale.x = marker.scale.y = circ.getRadius() * 2.0;
}

const std_msgs::ColorRGBA constructColorRGBA(float r, float g, float b, float a) noexcept {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}


geometry_msgs::Quaternion eulerToQuaternion(Position pos) noexcept {
    double cr = std::cos(pos.getX() * 0.5);
    double sr = std::sin(pos.getX() * 0.5);
    double cp = std::cos(pos.getY() * 0.5);
    double sp = std::sin(pos.getY() * 0.5);
    double cy = std::cos(pos.getAngle() * 0.5);
    double sy = std::sin(pos.getAngle() * 0.5);
    
    geometry_msgs::Quaternion quaternion;
    quaternion.x = cy * sr * cp - sy * cr * sp;
    quaternion.y = cy * cr * sp + sy * sr * cp;
    quaternion.z = sy * cr * cp - cy * sr * sp;
    quaternion.w = cy * cr * cp + sy * sr * sp;
    
    return quaternion;
}
