#ifndef COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H
#define COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H

#include "collisions/engine/check_zone.h"
#include "collisions/obstacle_velocity.h"

class Obstacle;

class VelocityCheckZone : public CheckZone {
public:
    /**
     * Initialize a collision checker based on robot velocity shapes.
     *
     * @param level The default collision level to apply on found collisions.
     * @param robotPos A reference on the robot position that must be updated outside.
     * @param robotVelocity A const reference on robot velocity shapes that must be updated externally
     */
    VelocityCheckZone(CollisionLevel level, const Position &robotPos, const ObstacleVelocity &robotVelocity) noexcept:
            CheckZone(level, robotPos), m_robotVelocity(robotVelocity) {}

    /** We leave the default destructor. **/
    ~VelocityCheckZone() override = default;

    /**
     * Returns a shape list corresponding to the area checked for collisions.
     *
     * @return The shape list.
     */
    const std::vector<ShapePtr> &getShapes() const override { return m_robotVelocity.getShapes(); }

    /**
     * Returns a shape list corresponding to the area checked for collisions.
     *
     * @param maxDist The maximum distance to compute the velocity projection.
     * @return The shape list.
     */
    const std::vector<ShapePtr> &getShapes(double maxDist) const {
        return m_robotVelocity.getShapes(maxDist);
    }

    /**
     * Updates obstacles' collision data when they are dangerous.
     *
     * @param obstacles An obstacle list to check.
     */
    void checkCollisions(const ObstacleRefList &obstacles) const override;

private:
    /** Stores the updated robot's velocity shapes. **/
    const ObstacleVelocity &m_robotVelocity;
};

#endif // COLLISIONS_ENGINE_VELOCITY_CHECK_ZONE_H
