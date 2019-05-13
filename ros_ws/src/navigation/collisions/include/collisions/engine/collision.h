#ifndef COLLISIONS_ENGINE_COLLISION
#define COLLISIONS_ENGINE_COLLISION

#include "collisions/engine/constants.h"

#include <limits>

/**
 * Stores information about a found collision.
 */
class Collision {
public:
    /**
     * Main constructor of Collision.
     * 
     * @param level The danger level of collision
     * @param approxDistance Distance between the robot and the obstacle
     */
    constexpr Collision(
            CollisionLevel level = CollisionLevel::SAFE,
            double approxDistance = std::numeric_limits<double>::max());
    
    /**
     * Returns the danger level of collision
     * 
     * @return The level of collision
     */
    constexpr CollisionLevel getLevel() const noexcept { return m_level; }

    /**
     * Sets the danger level of collision.
     * @param level
     */
    constexpr void setLevel(CollisionLevel level) noexcept { m_level = level; }
    
    /**
     * Returns the approximative distance between the robot and the obstacle.
     * 
     * @return The distance
     */
    constexpr double getDistance() const noexcept { return m_approxDistance; }

    /**
     *  Sets the distance between the robots and the obstacle.
     * @param distance
     */
    constexpr void setDistance(double distance) noexcept { m_approxDistance = distance; }
    
private:
    /** The danger level of collision **/
    CollisionLevel m_level;
    
    /** The distance between the robot and the obstacle **/
    double m_approxDistance;
};

constexpr Collision::Collision(CollisionLevel level, double approxDistance) :
        m_level(level), m_approxDistance(approxDistance) {}

#endif // COLLISIONS_ENGINE_COLLISION
