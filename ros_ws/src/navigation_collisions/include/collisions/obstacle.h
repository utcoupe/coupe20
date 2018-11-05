#ifndef COLLISIONS_OBSTACLE_H
#define COLLISIONS_OBSTACLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/obstacle_velocity.h"

#include <chrono>
#include <memory>

class Obstacle {
public:
    using ShapePtr = std::shared_ptr<CollisionsShapes::AbstractShape>;
    using VelocityPtr = std::shared_ptr<ObstacleVelocity>;
    
    Obstacle(ShapePtr shape, VelocityPtr velocity = nullptr);
    
    ShapePtr getShape() const { return shape_; }
    
    std::vector<ShapePtr> getVelocityShapes (double maxDist = -1.0);
    
    Position getPos() const { return shape_->getPos(); }
    
    std::chrono::duration<double> getAge() const;
    
protected:
    ShapePtr shape_;
    VelocityPtr velocity_;
    std::chrono::system_clock::time_point spawnTime_ = std::chrono::system_clock::now();
};

#endif // COLLISIONS_OBSTACLE_H
