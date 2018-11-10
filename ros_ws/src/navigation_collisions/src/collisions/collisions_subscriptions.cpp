#include "collisions/collisions_subscriptions.h"

#include "collisions/shapes/circle.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/segment.h"
#include "collisions/obstacle_velocity.h"

#include <ros/duration.h>

#include <cmath>
#include <string>

const double CACHE_TIME_TF2_BUFFER = 5.0;
const unsigned SIZE_MAX_QUEUE = 10;

const std::string NAVIGATOR_STATUS_TOPIC = "navigation/navigator/status";
const std::string OBJECTS_TOPIC          = "recognition/objects_classifier/objects";
const std::string ASSERV_SPEED_TOPIC     = "drivers/ard_asserv/speed";

const std::string NODE_NAME      = "collisions";
const std::string NAMESPACE_NAME = "navigation";

const std::string MAP_TF_FRAME  = "map";

Position quaternionToEuler(geometry_msgs::Quaternion quaternion);

inline constexpr double radToDegrees(double angle) {
    return angle * M_PI / 180;
}

CollisionsSubscriptions::CollisionsSubscriptions(ros::NodeHandle& nhandle, RobotPtr robot):
robot_(robot),
    tf2PosBuffer_(ros::Duration(CACHE_TIME_TF2_BUFFER)),
    tf2PosListener_(tf2PosBuffer_)
{
    asservSpeedSubscriber_ = nhandle.subscribe<drivers_ard_asserv::RobotSpeed>(
        ASSERV_SPEED_TOPIC,
        SIZE_MAX_QUEUE,
        &CollisionsSubscriptions::onAsservSpeed,
        this
    );
    navigatorStatusSubscriber_ = nhandle.subscribe<navigation_navigator::Status>(
        NAVIGATOR_STATUS_TOPIC,
        SIZE_MAX_QUEUE,
        &CollisionsSubscriptions::onNavStatus,
        this
    );
    
    objectsSubscriber_ = nhandle.subscribe<recognition_objects_classifier::ClassifiedObjects>(
        OBJECTS_TOPIC,
        SIZE_MAX_QUEUE,
        &CollisionsSubscriptions::onObjects,
        this
    );
    
    gameStatus_ = std::make_unique<StatusServices>(NAMESPACE_NAME, NODE_NAME, nullptr, [this](const ai_game_manager::GameStatus::ConstPtr& status) {
        this->onGameStatus(status);
    });
}

void CollisionsSubscriptions::onAsservSpeed(const drivers_ard_asserv::RobotSpeed::ConstPtr& speed)
{
    velLinear_ = speed->linear_speed;
    velAngular_ = 0.0;
}


void CollisionsSubscriptions::onGameStatus(const ai_game_manager::GameStatus::ConstPtr& status) {
    // Nothing ?
}

void CollisionsSubscriptions::onNavStatus(const navigation_navigator::Status::ConstPtr& status)
{
    if (status->status == status->NAV_IDLE)
        navStatus_ = Robot::NavStatus::IDLE;
    else if (status->status == status->NAV_NAVIGATING)
        navStatus_ = Robot::NavStatus::NAVIGATING;
    
    robotPathWaypoints_.clear();
    for (auto point: status->currentPath)
        robotPathWaypoints_.emplace_back(point.x, point.y, point.theta);
}

void CollisionsSubscriptions::onObjects(const recognition_objects_classifier::ClassifiedObjects::ConstPtr& objects)
{
    std::vector<std::shared_ptr<Obstacle>> newBelt, newLidar;
    
    for (auto rect: objects->unknown_rects) {
        if (rect.header.frame_id != MAP_TF_FRAME && rect.header.frame_id != "/" + MAP_TF_FRAME) {
            ROS_WARN_STREAM("Belt rect not in /" << MAP_TF_FRAME << " tf frame, skipping.");
            continue;
        }
        auto rectShape = std::make_shared<CollisionsShapes::Rectangle>(
            Position(rect.x, rect.y, rect.a),
            rect.w,
            rect.h
        );
        newBelt.emplace_back(std::make_shared<Obstacle>(rectShape));
    }
    if (!newBelt.empty()) {
        obstacleStack.updateBeltPoints(newBelt);
    }
    
    for (auto seg: objects->unknown_segments) {
        if (seg.header.frame_id != MAP_TF_FRAME && seg.header.frame_id != "/" + MAP_TF_FRAME) {
            ROS_WARN_STREAM("Lidar segment not in /" << MAP_TF_FRAME << " tf frame, skipping.");
            continue;
        }
        auto segShape = std::make_shared<CollisionsShapes::Segment>(
            Point(seg.segment.first_point.x, seg.segment.first_point.y),
            Point(seg.segment.last_point.x, seg.segment.last_point.y)
        );
        newLidar.emplace_back(std::make_shared<Obstacle>(segShape));
    }
    for (auto circ: objects->unknown_circles) {
        if (circ.header.frame_id != MAP_TF_FRAME && circ.header.frame_id != "/" + MAP_TF_FRAME) {
            ROS_WARN_STREAM("Lidar circle not in /" << MAP_TF_FRAME << " tf frame, skipping.");
            continue;
        }
        double velDist = std::hypot(circ.circle.velocity.x, circ.circle.velocity.y);
        double velAngle = std::atan2(circ.circle.velocity.y, circ.circle.velocity.x);
        auto circShape = std::make_shared<CollisionsShapes::Circle>(
            Position(circ.circle.center.x, circ.circle.center.y, velAngle),
            circ.circle.radius
        );
        auto velocity = std::make_shared<ObstacleVelocity>(
            0.0,
            0.0,
            velAngle,
            velDist
        );
        newLidar.emplace_back(std::make_shared<Obstacle>(circShape, velocity));
    }
    if (!newLidar.empty()) {
        obstacleStack.updateLidarObjects(newLidar);
    }
}

Position quaternionToEuler(geometry_msgs::Quaternion quaternion)
{
    double qw, qx, qy, qz;
    qw = quaternion.w;
    qx = quaternion.x;
    qy = quaternion.y;
    qz = quaternion.z;
    
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
    double ex = std::atan2(t0, t1);
    
    double t2 = +2.0 * (qw * qy - qz * qx);
    if (t2 < -1.0)
        t2 = -1.0;
    else if (t2 > 1.0)
        t2 = 1.0;
    double ey = std::asin(t2);
    
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
    double ez = std::atan2(t3, t4);
    
    return {ex, ey, ez};
}
