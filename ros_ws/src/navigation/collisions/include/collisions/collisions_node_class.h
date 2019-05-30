#ifndef COLLISIONS_NODE_CLASS_H
#define COLLISIONS_NODE_CLASS_H

#include "collisions/collisions_subscriptions.h"
#include "collisions/markers_publisher.h"
#include "collisions/obstacles_stack.h"
#include "collisions/robot.h"
#include "collisions/shapes/abstract_shape.h"

#include "collisions/ActivateCollisions.h"
#include "collisions/PredictedCollision.h"

#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/timer.h>

#include <atomic>

namespace ros {
    class NodeHandle;
} // namespace ros

class Collision;

/**
 * Main class of the node.
 */
class CollisionsNode {
public:
    /** Represents a shared pointer on a Robot object. */
    using RobotPtr = std::shared_ptr<Robot>;
    /** Represents a shared pointer on an ObstacleStack object. */
    using ObstaclesStackPtr = std::shared_ptr<ObstaclesStack>;
    
    /**
     * Initialize the services and topics owned by the node.
     * @param nhandle A valid ros::NodeHandle that will be used to create or connect to services and topics.
     */
    CollisionsNode(ros::NodeHandle& nhandle);
    
    /** Default destructor. */
    ~CollisionsNode() = default;

private:
    /** Activate collision checks in the main loop. It is edited through the service set_active */
    std::atomic_bool m_active { false };
    /**
     * Timer that will invoke at a regular rate the main loop.
     */
    ros::Timer m_timerRun;
    
    /** Contains all informations about the robot. */
    RobotPtr m_robot;
    /** Contains all informations about the current obstacles. */
    ObstaclesStackPtr m_obstacleStack;
    /** Object managing all subscriptions to other nodes. */
    CollisionsSubscriptions m_subscriptions;
    /** Server for the set_active service. */
    ros::ServiceServer m_setActiveService;
    /** Publisher on the warner topic. */
    ros::Publisher m_warnerPublisher;
    /** Manager that publishes dangerous objects and check zones in rviz */
    MarkersPublisher m_markersPublisher;
    
    /**
     * Main loop
     */
    void m_run(const ros::TimerEvent&);
    /**
     * Publishes a collision on the topic warner
     * @param collision The collision to publish in the topic.
     */
    void m_publishCollision(const Collision& collision);
    /**
     * Callback for the set_status service.
     * @param req The received request
     * @param res The response (res.success = true)
     * @return always true
     */
    bool m_onSetActive(collisions::ActivateCollisions::Request& req, collisions::ActivateCollisions::Response& res);
    
    /**
     * Adds CollisionShapes::Rectangle properties on the warner topic's message.
     * @param msg Pre-constructed message that will be completed
     * @param shape A pointer on a CollisionShapes::Rectangle (will throw if it isn't one)
     */
    void m_addRectInfosToPredictedCollision(collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape& shape) const;
    /**
     * Adds CollisionShapes::Circle properties on the warner topic's message.
     * @param msg Pre-constructed message that will be completed
     * @param shape A pointer on a CollisionShapes::Circle (will throw if it isn't one)
     */
    void m_addCircInfosToPredictedCollision(collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape& shape) const;
};

#endif // COLLISIONS_NODE_CLASS_H
