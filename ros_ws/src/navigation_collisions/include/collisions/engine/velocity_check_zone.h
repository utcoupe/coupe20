#ifndef COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H
#define COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H

#include "collisions/engine/check_zone.h"

class VelocityCheckZone: public CheckZone {
public:
    VelocityCheckZone(double width, double height, CollisionLevel level) noexcept:
        CheckZone(width, height, level)
    {}
    ~VelocityCheckZone() override = default;
    
    std::vector<ShapePtr> getShapes(Position robotPos) override { return getShapes(robotPos, 0.0, 0.0); }
    std::vector<ShapePtr> getShapes(Position robotPos, double velLinear, double velAngular, double maxDist = -1.0);
    
    std::vector<Collision> checkCollisions(Position robotPos, std::vector<ObstaclePtr> obstacles) override;
    std::vector<Collision> checkCollisions(Position robotPos, std::vector<ObstaclePtr> obstacles, double velLinear, double velAngular);
};

#endif // COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H
