#ifndef COLLISIONS_CHECK_ZONE_H
#define COLLISIONS_CHECK_ZONE_H

#include "collisions/engine/constants.h"
#include "collisions/engine/engine.h"
#include "collisions/obstacle.h"
#include "collisions/engine/collision.h"
#include "collisions/shapes/abstract_shape.h"

#include <memory>
#include <vector>

class Position;

/**
 * Abstract collision checker.
 */
class CheckZone {
public:
    /** Alias to represent an obstacle pointer. **/
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    /** Alias to represent a shape pointer. **/
    using ShapePtr = std::unique_ptr<CollisionsShapes::AbstractShape>;
    
    /**
     * Initiazes the checker.
     * 
     * @param level The default collision level to apply on found collisions.
     * @param robotPos A reference on the robot position that must be updated outside.
     */
    CheckZone(CollisionLevel level, const Position& robotPos) noexcept:
        m_level(level), m_robotPos(robotPos){}
    
    /**
     * Default destructor.
     */
    virtual ~CheckZone() = default;
    
    /**
     * Returns a shape list corresponding to the area checked for collisions.
     * 
     * @return The shape list.
     */
    virtual const std::vector<ShapePtr>& getShapes() const = 0;
    
    /**
     * Returns a list of found collisions according to the obstacle list passed by parameter.
     * 
     * @param obstacles A obstacle list to check.
     * @return A collision list.
     */
    virtual std::vector<Collision> checkCollisions(const std::vector<ObstaclePtr>& obstacles) const = 0;
    
    /**
     * Returns the default collision level applied on found collisions. It can differ from some collision values.
     */
    CollisionLevel  getLevel()  const noexcept { return m_level; }
    
protected:
    /** The default collision level **/
    CollisionLevel m_level;
    /** A reference on robot position **/
    const Position& m_robotPos;
};

#endif // COLLISIONS_CHECK_ZONE_H
