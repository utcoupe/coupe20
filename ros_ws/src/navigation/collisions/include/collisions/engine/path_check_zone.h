#ifndef COLLISIONS_ENGINE_PATH_CHECK_ZONE_H
#define COLLISIONS_ENGINE_PATH_CHECK_ZONE_H

#include "collisions/engine/check_zone.h"

/**
 * Checks collision on a path.
 */
class PathCheckZone: public CheckZone
{
public:
    /**
     * Initializes the checker.
     * 
     * @param collisionLevel The level to apply by default on found collisions.
     * @param robotPos A reference on robot position that must be updated outside.
     * @param width The width of the collision box, likely robot width.
     * @param height The hight of the collision box, likely robot height.
     */
    PathCheckZone(CollisionLevel collisionLevel, const Position& robotPos, double width, double height):
        CheckZone(collisionLevel, robotPos),
        m_width(width), m_height(height)
    {}
    
    /**
     * Default destructor.
     */
    ~PathCheckZone() override = default;
    
    /**
     * Returns a list of collision box shapes.
     * 
     * @return The shape list.
     */
    const std::vector<ShapePtr>& getShapes() const override;
    
    /**
     * Returns a list of found collision according to the obstacle list passed by argument.
     * 
     * @param obstacles An obstacle list to check collision.
     * @return The collision list.
     */
    std::vector<Collision> checkCollisions(const std::vector<const Obstacle*>& obstacles) const override;
    
    /**
     * Updates internal list of waypoints constituting the current path.
     * 
     * @param newWaypoints The current path
     */
    void updateWaypoints(const std::vector<Position>& newWaypoints) { m_waypoints = newWaypoints; }
    
    /**
     * Returns true if there is at least one waypoint in the path, false else.
     * 
     * @return True if path not empty.
     */
    bool hasWaypoints() const noexcept { return !m_waypoints.empty(); }
    
    /**
     * Returns the first waypoint of the path. It may crash if the path is empty.
     * 
     * @return The first waypoint of the path.
     */
    const Position& getFirstWaypoint() const { return m_waypoints.front(); }
    
    /**
     * Returns the height of the collision box.
     * 
     * @return The height.
     */
    double getHeight() const noexcept { return m_height; }
    
    /**
     * Returns the width of the collision box.
     * 
     * @return The width.
     */
    double getWidth()  const noexcept { return m_width; }
    
private:
    /** The width of the collision box **/
    double m_width;
    /** The height of the collision box **/
    double m_height;
    /** The current path **/
    std::vector<Position> m_waypoints;
    /** Generated shapes TODO clean **/
    mutable std::vector<ShapePtr> m_shapes;
};

#endif // COLLISIONS_ENGINE_PATH_CHECK_ZONE_H
