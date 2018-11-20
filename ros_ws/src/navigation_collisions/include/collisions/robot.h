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
    
    constexpr void setPos(Position pos) noexcept { pos_ = pos; }
    constexpr Position getPos() const   noexcept { return pos_; }
    
    constexpr void updateVelocity(double velLinear, double velAngular) noexcept {
        velocity_.setVelLinear(velLinear);
        velocity_.setVelAngular(velAngular);
    }
    
    constexpr void updateStatus(NavStatus status) noexcept { navStatus_ = status; }
    void updateWaypoints(const std::vector<Position>& newWaypoints) { pathCheckZone_.updateWaypoints(newWaypoints); }
    
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
