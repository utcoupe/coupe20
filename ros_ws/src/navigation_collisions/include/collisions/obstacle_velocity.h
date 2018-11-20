#ifndef COLLISIONS_OBSTACLE_VELOCITY_H
#define COLLISIONS_OBSTACLE_VELOCITY_H

#include "collisions/shapes/abstract_shape.h"

#include <memory>
#include <vector>

class ObstacleVelocity {
public:
    using ShapePtr = std::shared_ptr<CollisionsShapes::AbstractShape>;
    
    ObstacleVelocity(double width, double height, double velLinear = 0.0, double velAngular = 0.0, Position objectPos = {}) noexcept;
    
    std::vector<ShapePtr> getShapes(double maxDist = -1.0);
    
    constexpr void setObjectPos(Position pos) noexcept {
        objectPos_ = pos;
        needUpdate_ = true;
    }
    
private:
    bool needUpdate_ = true;
    double width_, height_, velLinear_, velAngular_;
    Position objectPos_;
    std::vector<ShapePtr> velShapes_;
    double lastMaxDist_ = -1.0;
    
    void generateVelShapes(double maxDist);
};

#endif // COLLISIONS_OBSTACLE_VELOCITY_H
