#include "collisions/collisions_node_class.h"

#include "collisions/obstacle.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/circle.h"
#include "collisions/engine/constants.h"

#include "navigation_collisions/PredictedCollision.h"

#include <chrono>
#include <functional>
#include <string>

const std::string SET_ACTIVE_SERVICE    = "navigation/collisions/set_active";
const std::string WARNER_TOPIC          = "navigation/collisions/warner";

const double RATE_RUN_HZ = 20.0;

CollisionsNode::CollisionsNode():
    subscriptions_(nhandle_)
{
    ROS_INFO("Collisions node is starting. Please wait...");
    obstacleStack_ = subscriptions_.getObstaclesStack();
    setActiveService_ = nhandle_.advertiseService(
        SET_ACTIVE_SERVICE,
        &CollisionsNode::onSetActive,
        this
    );
    warnerPublisher_ = nhandle_.advertise<navigation_collisions::PredictedCollision>(WARNER_TOPIC, 1);
    
    robot_ = subscriptions_.createRobot(nhandle_);
    
    subscriptions_.sendInit(true);
    ROS_INFO("navigation/collisions ready, waiting for activation.");
    runThread_ = std::thread([&](){ this->run(); });
    runThread_.detach();
}

CollisionsNode::~CollisionsNode()
{
    runThread_.join();
}


void CollisionsNode::run()
{
    auto rate = ros::Rate(RATE_RUN_HZ);
    std::chrono::system_clock::time_point startTime, endTime;
    
    while(!ros::isShuttingDown()) {
        startTime = std::chrono::system_clock::now();
        subscriptions_.updateRobot();
        if (active_) {
            for (const auto& collision: robot_->checkCollisions(obstacleStack_->toList())) {
                publishCollision(collision);
            }
        }
        
        obstacleStack_->garbageCollect();
        
        endTime = std::chrono::system_clock::now();
        ROS_DEBUG_STREAM("Cycle done in " << (std::chrono::duration<double, std::milli>(endTime - startTime)).count() << "ms");
        rate.sleep();
    }
}

void CollisionsNode::publishCollision(const Collision& collision)
{
    navigation_collisions::PredictedCollision msg;
    msg.danger_level = static_cast<unsigned>(collision.getLevel());
    
    switch(collision.getLevel()) {
    case CollisionLevel::LEVEL_STOP:
        ROS_WARN_THROTTLE(0.2, "Found freaking clos collision, please stop!!!");
        break;
    case CollisionLevel::LEVEL_DANGER:
        ROS_WARN_THROTTLE(0.5, "Found close collision intersecting with the path.");
        break;
    case CollisionLevel::LEVEL_POTENTIAL:
        ROS_INFO_THROTTLE(1, "Foudn far-off collision intersecting with the path.");
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
}

bool CollisionsNode::onSetActive(navigation_collisions::ActivateCollisions::Request& req, navigation_collisions::ActivateCollisions::Response& res)
{
    active_ = req.active;
    std::string msg = "Starting";
    if (!active_)
        msg = "Stopping";
    ROS_INFO_STREAM(msg << " collisions check.");
    return true;
}

void CollisionsNode::addRectInfosToPredictedCollision(navigation_collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape* const shape) const
{
    msg.obstacle_type = msg.TYPE_RECT;
    auto* rect = dynamic_cast<const CollisionsShapes::Rectangle* const>(shape);
    msg.obstacle_width = rect->getWidth();
    msg.obstacle_height = rect->getHeight();
}

void CollisionsNode::addCircInfosToPredictedCollision(navigation_collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape* const shape) const
{
    msg.obstacle_type = msg.TYPE_CIRCLE;
    auto* circle = dynamic_cast<const CollisionsShapes::Circle* const>(shape);
    msg.obstacle_radius = circle->getRadius();
}
