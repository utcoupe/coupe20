#ifndef COLLISIONS_NODE_CLASS_H
#define COLLISIONS_NODE_CLASS_H

#include "collisions/collisions_subscriptions.h"
#include "collisions/robot.h"
#include "collisions/obstacles_stack.h"
#include "collisions/engine/collision.h"
#include "collisions/shapes/abstract_shape.h"

#include "navigation_collisions/ActivateCollisions.h"
#include "navigation_collisions/PredictedCollision.h"

#include <ros/ros.h>

class CollisionsNode {
public:
    using RobotPtr = std::shared_ptr<Robot>;
    using ObstaclesStackPtr = std::shared_ptr<ObstacleStack>;
    CollisionsNode();

private:
    RobotPtr robot_;
    ObstaclesStackPtr obstacleStack_;
    bool active_ = false; // navigation/navigator activates this node through a service.
    CollisionsSubscriptions subscriptions_;
    ros::NodeHandle nhandle_;
    ros::ServiceServer setActiveService_;
    ros::Publisher warnerPublisher_;
    
    void run();
    void publishCollision(const Collision& collision);
    bool onSetActive(navigation_collisions::ActivateCollisions::Request& req, navigation_collisions::ActivateCollisions::Response& res);
    
    void addRectInfosToPredictedCollision(navigation_collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape* const shape) const;
    void addCircInfosToPredictedCollision(navigation_collisions::PredictedCollision& msg, const CollisionsShapes::AbstractShape* const shape) const;
};

#endif // COLLISIONS_NODE_CLASS_H
