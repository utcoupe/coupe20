#include "collisions/collisions_node_class.h"

#include "collisions/obstacle.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/circle.h"
#include "collisions/engine/constants.h"
#include "collisions/engine/collision.h"

#include <ros/console.h>
#include <ros/node_handle.h>

#include <chrono>
#include <functional>
#include <string>
#include <utility>

const std::string SET_ACTIVE_SERVICE    = "navigation/collisions/set_active";
const std::string WARNER_TOPIC          = "navigation/collisions/warner";

const double RATE_RUN_HZ = 20.0;

CollisionsNode::CollisionsNode(ros::NodeHandle& nhandle):
    m_subscriptions(nhandle),
    m_markersPublisher(nhandle)
{
    ROS_INFO("Collisions node is starting. Please wait...");
    m_obstacleStack = m_subscriptions.getObstaclesStack();
    m_setActiveService = nhandle.advertiseService(
        SET_ACTIVE_SERVICE,
        &CollisionsNode::m_onSetActive,
        this
    );
    m_warnerPublisher = nhandle.advertise<collisions::PredictedCollision>(WARNER_TOPIC, 1);
    
    m_robot = m_subscriptions.createRobot(nhandle);
    
    m_subscriptions.sendInit(true);
    ROS_INFO("navigation/collisions ready, waiting for activation.");
    m_timerRun = nhandle.createTimer(
        ros::Duration(1 / RATE_RUN_HZ),
        &CollisionsNode::m_run,
        this
    );
}


void CollisionsNode::m_run(const ros::TimerEvent&) {
    std::chrono::system_clock::time_point startTime;
    startTime = std::chrono::system_clock::now();
    m_subscriptions.updateRobot();
    if (m_active) {
        bool firstRun = true;
        Collision worstCollsision(CollisionLevel::SAFE, nullptr, 0.0);
        for (auto&& collision: m_robot->checkCollisions(m_obstacleStack->toList())) {
            if (firstRun || worstCollsision.getLevel() < collision.getLevel()) {
                worstCollsision = std::move(collision);
            }
        }
        m_publishCollision(worstCollsision);
        m_markersPublisher.publishCheckZones(*m_robot);
    }
    
    m_markersPublisher.publishObstacles(m_obstacleStack->toList());
    m_obstacleStack->garbageCollect();
    
    auto spentTime = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now() - startTime);
    
    ROS_DEBUG_STREAM_THROTTLE(1, "Cycle done in " << spentTime.count() << "ms");
}

void CollisionsNode::m_publishCollision(const Collision& collision) {
    collisions::PredictedCollision msg;
    msg.danger_level = static_cast<unsigned char>(collision.getLevel());
    
    switch(collision.getLevel()) {
    case CollisionLevel::STOP:
        ROS_WARN_THROTTLE(0.2, "Found freaking close collision, please stop!!!");
        break;
    case CollisionLevel::DANGER:
        ROS_WARN_THROTTLE(0.5, "Found close collision intersecting with the path.");
        break;
    case CollisionLevel::POTENTIAL:
        ROS_INFO_THROTTLE(1, "Found far-off collision intersecting with the path.");
        break;
    case CollisionLevel::SAFE:
        ROS_INFO_THROTTLE(1, "Found no collisions intersecting with the path.");
        break;
    }
    // obstacle can be null
    if (auto obst = collision.getObstacle()) {
        msg.obstacle_pos = obst->getPos().toPose2D();
        
        using ShapeType = CollisionsShapes::ShapeType;
        switch (obst->getShape().getShapeType()) {
        case ShapeType::RECTANGLE:
            m_addRectInfosToPredictedCollision(msg, obst->getShape());
            break;
        case ShapeType::CIRCLE:
            m_addCircInfosToPredictedCollision(msg, obst->getShape());
            break;
        default:
            ROS_WARN_ONCE("Found collision has a special shape that cannot be reported. This message will print once.");
        }
    }
    
    m_warnerPublisher.publish(msg);
}

bool CollisionsNode::m_onSetActive(
    collisions::ActivateCollisions::Request& req,
    collisions::ActivateCollisions::Response& res
) {
    m_active = req.active;
    std::string msg = "Starting";
    if (!m_active)
        msg = "Stopping";
    ROS_INFO_STREAM(msg << " collisions check.");
    res.success = true;
    return true;
}

void CollisionsNode::m_addRectInfosToPredictedCollision(
    collisions::PredictedCollision& msg,
    const CollisionsShapes::AbstractShape& shape
) const {
    msg.obstacle_type = msg.TYPE_RECT;
    const auto& rect = dynamic_cast<const CollisionsShapes::Rectangle&>(shape);
    msg.obstacle_width = static_cast<float>(rect.getWidth());
    msg.obstacle_height = static_cast<float>(rect.getHeight());
}

void CollisionsNode::m_addCircInfosToPredictedCollision(
    collisions::PredictedCollision& msg,
    const CollisionsShapes::AbstractShape& shape
) const {
    msg.obstacle_type = msg.TYPE_CIRCLE;
    const auto& circle = dynamic_cast<const CollisionsShapes::Circle&>(shape);
    msg.obstacle_radius = static_cast<float>(circle.getRadius());
}
