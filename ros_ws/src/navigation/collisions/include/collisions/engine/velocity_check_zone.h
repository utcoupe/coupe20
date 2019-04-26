#ifndef COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H
#define COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H

#include "collisions/engine/check_zone.h"

class ObstacleVelocity;

class VelocityCheckZone: public CheckZone {
public:
    VelocityCheckZone(CollisionLevel level, const Position& robotPos, const ObstacleVelocity& robotVelocity) noexcept:
        CheckZone(level, robotPos), m_robotVelocity(robotVelocity) {}
    
    ~VelocityCheckZone() override = default;
    
    const std::vector<ShapePtr>& getShapes() const override { return m_robotVelocity.getShapes(); }
    const std::vector<ShapePtr>& getShapes(double maxDist) const {
        return m_robotVelocity.getShapes(maxDist);
    }
    
    std::vector<Collision> checkCollisions(const std::vector<ObstaclePtr>& obstacles) const override;

private:
    const ObstacleVelocity& m_robotVelocity;
};

#endif // COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H
