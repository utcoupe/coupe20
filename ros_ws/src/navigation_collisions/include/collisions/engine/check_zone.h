#ifndef COLLISIONS_CHECK_ZONE_H
#define COLLISIONS_CHECK_ZONE_H

#include "collisions/engine/constants.h"
#include "collisions/position.h"
#include "collisions/engine/engine.h"
#include "collisions/obstacle.h"
#include "collisions/engine/collision.h"
#include "collisions/shapes/abstract_shape.h"

#include <memory>
#include <vector>

class CheckZone {
public:
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    using ShapePtr = std::shared_ptr<CollisionsShapes::AbstractShape>;
    
    CheckZone(double width, double height, CollisionLevel level):
        width_(width), height_(height), level_(level)
    {}
    
    virtual std::vector<ShapePtr> getShapes(Position robotPos) = 0;
    virtual std::vector<Collision> checkCollisions(Position robotPos, std::vector<ObstaclePtr> obstacles) = 0;
    
    CollisionLevel  getLevel()  const { return level_; }
    double          getHeight() const { return height_; }
    double          getWidth()  const { return width_; }
    
protected:
    double width_, height_;
    CollisionLevel level_;
};

#endif // COLLISIONS_CHECK_ZONE_H
