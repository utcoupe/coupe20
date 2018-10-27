#ifndef COLLISIONS_ENGINE_ENGINE_H
#define COLLISIONS_ENGINE_ENGINE_H

#include "collisions/engine/constants.h"
#include "collisions/engine/shapes.h"

#include <vector>

class MapObstacle;
using PtrObstacle = std::shared_ptr<MapObstacle>;

class Collision {
public:
    Collision(CollisionLevel level, PtrObstacle obstacle, double approxDistance):
        _level(level), _obstacle(obstacle), _approxDistance(approxDistance)
    {}
    
    // Getters & Setters
    CollisionLevel getLevel() const { return _level; }
    
    PtrObstacle getObstacle() const { return _obstacle; }
    
    double getDistance() const { return _approxDistance; }
    
private:
    CollisionLevel _level;
    PtrObstacle _obstacle;
    double _approxDistance;
};

namespace CollisionResolver {
    std::vector<PtrObstacle> findCollisions(const std::vector<PtrObstacle>& robotShapes, const std::vector<PtrObstacle>& obstacleShapes);
    bool intersect(PtrObstacle obst1, PtrObstacle obst2);
} // namespace CollisionResolver

#endif // COLLISIONS_ENGINE_ENGINE_H
