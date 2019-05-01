#ifndef COLLISIONS_OBSTACLE_VELOCITY_H
#define COLLISIONS_OBSTACLE_VELOCITY_H

#include "collisions/shapes/abstract_shape.h"

#include <memory>
#include <vector>

/**
 * Class representing the velocity of an obstacle.
 */
class ObstacleVelocity {
public:
    /** Alias to manipulate a shape pointer **/
    using ShapePtr = std::unique_ptr<CollisionsShapes::AbstractShape>;
    
    /**
     * Construct a velocity collision box according to linear and angular velocities, and the position of corresponding obstacle.
     * 
     * For now, only rectangle shape works, and must be describe with its width and its height.
     * 
     * @param width The width of the rectangle collision box
     * @param height The height of the rectangle collision box
     * @param velLinear The linear velocity (in polar coordinates)
     * @param velAngular The angular velocity (in polar coordinates)
     * @param objectPos The position of corresponding object.
     */
    ObstacleVelocity(double width, double height, double velLinear = 0.0, double velAngular = 0.0, Position objectPos = {}) noexcept;
    
    constexpr static double EXPANSION_DIST_DISABLED = -1.0;
    
    /**
     * Returns a shape list describing the collison box according to its velocities.
     * 
     * A maximum expansion distance can be passed by parameter. It is disbled when the value is EXPANSION_DIST_DISABLED. This function uses internal cache updated only when needed.
     * 
     * @param maxDist The maximum expansion distance of collision box.
     * @return The shape list corresponding to the collision box.
     */
    const std::vector<ShapePtr>& getShapes(double maxDist = EXPANSION_DIST_DISABLED) const;
    
    /**
     * Sets obstacle position and invalidates caches.
     * 
     * @param pos The new position.
     */
    void setObjectPos(const Position& pos) noexcept {
        m_objectPos = pos;
        m_needUpdate = true;
    }
    
    /**
     * Sets obstacle velocities (in polar coordinates) and invalidates cache.
     * @param velLinear The linear velocity
     * @param velAngular The angular velocity
     */
    void setVelocity(double velLinear, double velAngular) {
        m_velLinear = velLinear;
        m_velAngular = velAngular;
        m_needUpdate = true;
    }
    
private:
    /** Says if caches need update */
    mutable bool m_needUpdate = true;
    
    /** The collision box width */
    double m_width;
    /** The collision box height */
    double m_height;
    
    /** The linear velocity */
    double m_velLinear;
    /** The angular velocity */
    double m_velAngular;
    
    /** The obstacle current position */
    Position m_objectPos;
    
    /** Cache for all velocity shapes */
    mutable std::vector<ShapePtr> m_velShapes;
    /** Maximum expansion distance used to generate the cache for velocity shapes **/
    mutable double m_lastMaxDist = EXPANSION_DIST_DISABLED;
    
    /**
     * Stores in m_velShapes computed velocity collision box.
     * 
     * @param maxDist The maximum expansion distance.
     */
    void m_generateVelShapes(double maxDist) const;
};

#endif // COLLISIONS_OBSTACLE_VELOCITY_H
