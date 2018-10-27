#ifndef COLLISIONS_CHECK_ZONE_H
#define COLLISIONS_CHECK_ZONE_H

#include "collisions/engine/constants.h"
#include "collisions/position.h"
#include "collisions/engine/engine.h"

#include <memory>
#include <vector>

class MapObstacle;
class Collision;
using PtrObstacle = std::shared_ptr<MapObstacle>;

class CheckZone {
public:
    CheckZone(double width, double height, CollisionLevel level):
        width_(width), height_(height), level_(level)
    {}
    
    virtual std::vector<PtrObstacle> getShapes(Position robotPos) = 0;
    virtual std::vector<Collision> checkCollisions(Position robotPos, std::vector<PtrObstacle> obstacles);
    
    CollisionLevel  getLevel()  const { return level_; }
    double          getHeight() const { return height_; }
    double          getWidth()  const { return width_; }
    
protected:
    double width_, height_;
    CollisionLevel level_;
};

#endif // COLLISIONS_CHECK_ZONE_H
