#ifndef COLLISIONS_NODE_CLASS_H
#define COLLISIONS_NODE_CLASS_H

#include "collisions/collisions_subscriptions.h"
#include "collisions/markers_publisher.h"
#include "collisions/obstacles_stack.h"
#include "collisions/robot.h"
#include "collisions/engine/collision.h"
#include "collisions/shapes/abstract_shape.h"

#include "collisions/ActivateCollisions.h"
#include "collisions/PredictedCollision.h"

#include <ros/ros.h>

#include <atomic>
#include <thread>

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
    std::atomic_bool active_ { false };
    /**
     * Timer that will invoke at a regular rate the main loop.
     */
    ros::Timer timerRun;
    
    /** Contains all informations about the robot. */
    RobotPtr robot_;
    /** Contains all informations about the current obstacles. */
    ObstaclesStackPtr obstacleStack_;
    /** Object managing all subscriptions to other nodes. */
    CollisionsSubscriptions subscriptions_;
    /** Server for the set_active service. */
    ros::ServiceServer setActiveService_;
    /** Publisher on the warner topic. */
    ros::Publisher warnerPublisher_;
    /** Manager that publishes dangerous objects and check zones in rviz */
    MarkersPublisher markersPublisher_;
    
    /**
     * Main loop
     */
    void run(const ros::TimerEvent&);
    /**
     * Publishes a collision on the topic warner
     * @param collision The collision to publish in the topic.
     */
    void publishCollision(const Collision& collision);
    /**
     * Callback for the set_status service.
     * @param req The received request
     * @param res The response (res.success = true)
     * @return always true
     */
    bool onSetActive(collisions::ActivateCollisions::Request& req, collisions::ActivateCollisions::Response& res);
    
    /**
     * Adds CollisionShapes::Rectangle properties on the warner topic's message.
     * @param msg Pre-constructed message that will be completed
     * @param shape A pointer on a CollisionShapes::Rectangle (will throw if it isn't one)
     */
    void addRectInfosToPredictedCollision(collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape* const shape) const;
    /**
     * Adds CollisionShapes::Circle properties on the warner topic's message.
     * @param msg Pre-constructed message that will be completed
     * @param shape A pointer on a CollisionShapes::Circle (will throw if it isn't one)
     */
    void addCircInfosToPredictedCollision(collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape* const shape) const;
};

#endif // COLLISIONS_NODE_CLASS_H
