#ifndef COLLISIONS_ENGINE_ENGINE_H
#define COLLISIONS_ENGINE_ENGINE_H

#include "collisions/engine/constants.h"
#include "collisions/engine/shapes.h"

#include <vector>

class Collision {
public:
    Collision(CollisionLevel level, MapObstacle* obstacle, double approxDistance):
        _level(level), _obstacle(obstacle), _approxDistance(approxDistance)
    {}
    
    // Getters & Setters
    CollisionLevel getLevel() const { return _level; }
    
    MapObstacle* getObstacle() const { return _obstacle; }
    
    double getDistance() const { return _approxDistance; }
    
private:
    CollisionLevel _level;
    MapObstacle* _obstacle;
    double _approxDistance;
}

namespace CollisionResolver {
    std::vector<Collision> findCollisions(const std::vector<MapObstacle>& robotShapes, const std::vector<MapObstacle>& obstacleShapes);
    bool intersect(const MapObstacle& obst1, const MapObstacle& obst2); // TODO * ou const &
    
    
} // namespace CollisionResolver

#endif // COLLISIONS_ENGINE_ENGINE_H
