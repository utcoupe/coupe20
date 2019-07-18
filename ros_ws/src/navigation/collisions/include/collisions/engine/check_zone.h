#ifndef COLLISIONS_CHECK_ZONE_H
#define COLLISIONS_CHECK_ZONE_H

#include "collisions/engine/constants.h"
#include "collisions/shapes/abstract_shape.h"

#include <memory>
#include <vector>

class Position;

class Obstacle;

/**
 * Abstract collision checker.
 */
class CheckZone {
public:
    /** Alias to represent a shape pointer. **/
    using ShapePtr = std::unique_ptr<CollisionsShapes::AbstractShape>;

    /**
     * Initializes the checker.
     * 
     * @param level The default collision level to apply on found collisions.
     * @param robotPos A reference on the robot position that must be updated outside.
     */
    CheckZone(CollisionLevel level, const Position &robotPos) noexcept:
            m_level(level), m_robotPos(robotPos) {}

    /**
     * Default destructor.
     */
    virtual ~CheckZone() = default;

    /**
     * Returns a shape list corresponding to the area checked for collisions.
     * 
     * @return The shape list.
     */
    virtual const std::vector<ShapePtr> &getShapes() const = 0;

    /**
     * Updates obstacles' collision data if they are dangerous.
     * 
     * @param obstacles An obstacle list to check.
     */
    virtual void checkCollisions(const std::vector<Obstacle *> &obstacles) const = 0;

    /**
     * Returns the default collision level applied on found collisions.
     *
     * It can differ from some collision values.
     *
     * @return The danger level.
     */
    CollisionLevel getLevel() const noexcept { return m_level; }

protected:
    /** The default collision level **/
    CollisionLevel m_level;
    /** A reference on robot position **/
    const Position &m_robotPos;
};

#endif // COLLISIONS_CHECK_ZONE_H
