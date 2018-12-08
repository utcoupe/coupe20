#include "collisions/collisions_node_class.h"

#include "collisions/obstacle.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/circle.h"
#include "collisions/engine/constants.h"

#include <ros/console.h>

#include <chrono>
#include <functional>
#include <string>

const std::string SET_ACTIVE_SERVICE    = "navigation/collisions/set_active";
const std::string WARNER_TOPIC          = "navigation/collisions/warner";

const double RATE_RUN_HZ = 20.0;

CollisionsNode::CollisionsNode(ros::NodeHandle& nhandle):
    subscriptions_(nhandle),
    markersPublisher_(nhandle)
{
    ROS_INFO("Collisions node is starting. Please wait...");
    obstacleStack_ = subscriptions_.getObstaclesStack();
    setActiveService_ = nhandle.advertiseService(
        SET_ACTIVE_SERVICE,
        &CollisionsNode::onSetActive,
        this
    );
    warnerPublisher_ = nhandle.advertise<collisions::PredictedCollision>(WARNER_TOPIC, 1);
    
    robot_ = subscriptions_.createRobot(nhandle);
    
    subscriptions_.sendInit(true);
    ROS_INFO("navigation/collisions ready, waiting for activation.");
    timerRun = nhandle.createTimer(
        ros::Duration(1 / RATE_RUN_HZ),
        &CollisionsNode::run,
        this
    );
}


void CollisionsNode::run(const ros::TimerEvent&)
{
    std::chrono::system_clock::time_point startTime;
    startTime = std::chrono::system_clock::now();
    subscriptions_.updateRobot();
    if (active_) {
        for (const auto& collision: robot_->checkCollisions(obstacleStack_->toList())) {
            publishCollision(collision);
        }
        markersPublisher_.publishCheckZones(robot_);
    }
    
    markersPublisher_.publishObstacles(obstacleStack_->toList());
    obstacleStack_->garbageCollect();
    
    auto spentTime = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now() - startTime);
    
    ROS_DEBUG_STREAM_THROTTLE(1, "Cycle done in " << spentTime.count() << "ms");
}

void CollisionsNode::publishCollision(const Collision& collision)
{
    collisions::PredictedCollision msg;
    msg.danger_level = static_cast<unsigned char>(collision.getLevel());
    
    switch(collision.getLevel()) {
    case CollisionLevel::LEVEL_STOP:
        ROS_WARN_THROTTLE(0.2, "Found freaking close collision, please stop!!!");
        break;
    case CollisionLevel::LEVEL_DANGER:
        ROS_WARN_THROTTLE(0.5, "Found close collision intersecting with the path.");
        break;
    case CollisionLevel::LEVEL_POTENTIAL:
        ROS_INFO_THROTTLE(1, "Found far-off collision intersecting with the path.");
        break;
    }
    auto obst = collision.getObstacle();
    msg.obstacle_pos.x = obst->getPos().getX();
    msg.obstacle_pos.y = obst->getPos().getY();
    msg.obstacle_pos.theta = obst->getPos().getAngle();
    
    using ShapeType = CollisionsShapes::ShapeType;
    switch (obst->getShape()->getShapeType()) {
    case ShapeType::RECTANGLE:
        addRectInfosToPredictedCollision(msg, obst->getShape().get());
        break;
    case ShapeType::CIRCLE:
        addCircInfosToPredictedCollision(msg, obst->getShape().get());
        break;
    default:
        ROS_WARN_ONCE("Found collision has a special shape that cannot be reported.");
    }
    
    warnerPublisher_.publish(msg);
}

bool CollisionsNode::onSetActive(collisions::ActivateCollisions::Request& req, collisions::ActivateCollisions::Response& res)
{
    active_ = req.active;
    std::string msg = "Starting";
    if (!active_)
        msg = "Stopping";
    ROS_INFO_STREAM(msg << " collisions check.");
    res.success = true;
    return true;
}

void CollisionsNode::addRectInfosToPredictedCollision(collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape* const shape) const
{
    msg.obstacle_type = msg.TYPE_RECT;
    auto* rect = dynamic_cast<const CollisionsShapes::Rectangle* const>(shape);
    msg.obstacle_width = static_cast<float>(rect->getWidth());
    msg.obstacle_height = static_cast<float>(rect->getHeight());
}

void CollisionsNode::addCircInfosToPredictedCollision(collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape* const shape) const
{
    msg.obstacle_type = msg.TYPE_CIRCLE;
    auto* circle = dynamic_cast<const CollisionsShapes::Circle* const>(shape);
    msg.obstacle_radius = static_cast<float>(circle->getRadius());
}
