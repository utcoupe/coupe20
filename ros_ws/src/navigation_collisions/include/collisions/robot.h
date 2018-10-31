#ifndef COLLISIONS_ROBOT_H
#define COLLISIONS_ROBOT_H

#include "collisions/position.h"
#include "collisions/obstacle.h"
#include "collisions/engine/velocity.h"
#include "collisions/engine/constants.h"
#include "collisions/engine/path_check_zone.h"
#include "collisions/engine/collision.h"
#include "collisions/shapes/abstract_shape.h"

#include <memory>
#include <utility>
#include <vector>

class Robot {
public:
    using ShapePtr = std::shared_ptr<CollisionsShapes::AbstractShape>;
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    enum class NavStatus { IDLE, NAVIGATING };
    
    Robot(double width, double height);
    
    void setPos(Position pos) { pos_ = pos; }
    void updateVelocity(double velLinear, double velAngular);
    void updateStatus(NavStatus status) { navStatus_ = status; }
    void updateWaypoints(std::vector<Position>&& newWaypoints) { pathCheckZone_.updateWaypoints(std::move(newWaypoints); }
    
    std::vector<ShapePtr> getMainShapes();
    std::vector<ShapePtr> getPathShapes();
    std::vector<Collision> checkCollisions(std::vector<ObstaclePtr> obstacles);
    
private:
    double width_, height_;
    Position pos_;
    Velocity velocity_;
    NavStatus navStatus_ = NavStatus::IDLE;
    PathCheckZone pathCheckZone_;
    
    double getMaxMainDist() const;
};

#endif // COLLISIONS_ROBOT_H
