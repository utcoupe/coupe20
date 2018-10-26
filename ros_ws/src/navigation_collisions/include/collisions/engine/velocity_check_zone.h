#ifndef COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H
#define COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H

#include "collisions/engine/check_zone.h"

class VelocityCheckZone: public CheckZone {
public:
    VelocityCheckZone(double width, double height, CollisionLevel level):
        CheckZone(width, height, level)
    {}
    
    std::vector<PtrObstacle> getShapes(Position robotPos) override { return getShapes(robotPos, 0.0, 0.0); }
    std::vector<PtrObstacle> getShapes(Position robotPos, double velLinear, double velAngular, double maxDist = -1.0);
    
    std::vector<Collision> checkCollisions(Position robotPos, std::vector<PtrObstacle> obstacles) override {
        return checkCollisions(robotPos, obstacles, 0.0, 0.0);
    }
    std::vector<Collision> checkCollisions(Position robotPos, std::vector<PtrObstacle> obstacles, double velLinear, double velAngular);
};

#endif // COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H
