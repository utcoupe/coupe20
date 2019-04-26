#ifndef COLLISIONS_ENGINE_PATH_CHECK_ZONE_H
#define COLLISIONS_ENGINE_PATH_CHECK_ZONE_H

#include "collisions/engine/check_zone.h"

class PathCheckZone: public CheckZone
{
public:
    PathCheckZone(CollisionLevel collisionLevel, const Position& robotPos, double width, double height):
        CheckZone(collisionLevel, robotPos),
        m_width(width), m_height(height)
    {}
    ~PathCheckZone() override = default;
    
    const std::vector<ShapePtr>& getShapes() const override;
    std::vector<Collision> checkCollisions(const std::vector<ObstaclePtr>& obstacles) const override;
    
    void updateWaypoints(const std::vector<Position>& newWaypoints) { m_waypoints = newWaypoints; }
    
    bool hasWaypoints() const noexcept { return !m_waypoints.empty(); }
    const Position& getFirstWaypoint() const { return m_waypoints.front(); }
    
    double getHeight() const noexcept { return m_height; }
    double getWidth()  const noexcept { return m_width; }
    
private:
    double m_width, m_height;
    std::vector<Position> m_waypoints;
    mutable std::vector<ShapePtr> m_shapes;
};

#endif // COLLISIONS_ENGINE_PATH_CHECK_ZONE_H
