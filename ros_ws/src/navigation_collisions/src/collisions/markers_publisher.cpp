#include "collisions/markers_publisher.h"

#include "collisions/position.h"
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
    markersPubl_ = nhandle.advertise<visualization_msgs::Marker>(MARKERS_TOPIC, QUEUE_SIZE);
}

void MarkersPublisher::publishCheckZones(RobotPtr robot) {
    if (!isConnected())
        return;
    int index = 0;
    for (auto shape: robot->getMainShapes()) {
        publishMarker(NS_MARKERS_MAIN, index, shape, 0.02, 0.01, COLOR_ROBOT_MAIN_SHAPES);
        index++;
    }
    ROS_DEBUG_STREAM_THROTTLE(1, "Published " << index << " robot shapes.");
    index = 0;
    for (auto shape: robot->getPathShapes()) {
        publishMarker(NS_MARKERS_PATH, index, shape, 0.02, 0.01, COLOR_ROBOT_PATH_SHAPES);
        index++;
    }
    ROS_DEBUG_STREAM_THROTTLE(1, "Published " << index << " path shapes.");
}

void MarkersPublisher::publishObstacles(const std::vector<ObstaclePtr>& obstacles) {
    if (!isConnected())
        return;
    int index = 0;
    for (auto obstacle: obstacles) {
        publishMarker(NS_MARKERS_OBSTACLES, index, obstacle->getShape(), 0.35, 0.35 / 2.0, COLOR_OBSTACLES_SHAPES);
        index++;
        
        for (auto velShape: obstacle->getVelocityShapes()) {
            publishMarker(NS_MARKERS_OBSTACLES, index, velShape, 0.02, 0.01, COLOR_OBSTACLE_VELOCITY_SHAPES);
            index++;
        }
    }
    ROS_DEBUG_STREAM_THROTTLE(1, "Published " << index << " obstacle shapes.");
}

bool MarkersPublisher::isConnected() const {
    return markersPubl_.getNumSubscribers() > 0;
}

void MarkersPublisher::publishMarker(std::string ns, int index, ShapePtr shape, double z_scale, double z_height, std_msgs::ColorRGBA color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = MARKER_FRAME_ID;
    marker.ns = ns;
    marker.id = index;
    
    switch (shape->getShapeType()) {
    case CollisionsShapes::ShapeType::SEGMENT:
        addSegmentInfoToMarker(shape, marker);
        break;
    case CollisionsShapes::ShapeType::RECTANGLE:
        addRectangleInfoToMarker(shape, marker);
        break;
    case CollisionsShapes::ShapeType::CIRCLE:
        addCircleInfoToMarker(shape, marker);
        break;
    default:
        ROS_WARN_ONCE("Tried to publish marker for an unknown shape. This message will be printed once.");
    }
    
    marker.scale.z = z_scale;
    marker.color = color;
    marker.pose.position.x = shape->getPos().getX();
    marker.pose.position.y = shape->getPos().getY();
    marker.pose.position.z = z_height;
    marker.pose.orientation = eulerToQuaternion({0, 0, shape->getPos().getAngle()});
    marker.lifetime = ros::Duration(0.1);
    
    markersPubl_.publish(marker);
}

void MarkersPublisher::addSegmentInfoToMarker(MarkersPublisher::ShapePtr shape, visualization_msgs::Marker& marker)
{
    auto* seg = dynamic_cast<const CollisionsShapes::Segment* const>(shape.get());
    
    marker.type = marker.CUBE;
    marker.scale.x = seg->getLength();
    marker.scale.y = 0.02;
}

void MarkersPublisher::addRectangleInfoToMarker(MarkersPublisher::ShapePtr shape, visualization_msgs::Marker& marker)
{
    auto* rect = dynamic_cast<const CollisionsShapes::Rectangle* const>(shape.get());
    
    marker.type = marker.CUBE;
    marker.scale.x = rect->getWidth();
    marker.scale.y = rect->getHeight();
}

void MarkersPublisher::addCircleInfoToMarker(MarkersPublisher::ShapePtr shape, visualization_msgs::Marker& marker)
{
    auto* circ = dynamic_cast<const CollisionsShapes::Circle* const>(shape.get());
    
    marker.type = marker.CYLINDER;
    marker.scale.x = circ->getRadius() * 2.0;
    marker.scale.y = circ->getRadius() * 2.0;
}

const std_msgs::ColorRGBA constructColorRGBA(float r, float g, float b, float a) noexcept
{
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
