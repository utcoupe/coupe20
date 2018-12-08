#ifndef COLLISIONS_ENGINE_PATH_CHECK_ZONE_H
#define COLLISIONS_ENGINE_PATH_CHECK_ZONE_H

#include "collisions/engine/check_zone.h"

class PathCheckZone: public CheckZone
{
public:
    PathCheckZone(double width, double height, CollisionLevel collisionLevel):
        CheckZone(width, height, collisionLevel)
    {}
    ~PathCheckZone() override = default;
    
    std::vector<ShapePtr> getShapes(Position robotPos) override;
    std::vector<Collision> checkCollisions(Position robotPos, std::vector<ObstaclePtr> obstacles) override;
    
    void updateWaypoints(const std::vector<Position>& newWaypoints) { waypoints_ = newWaypoints; }
    
    bool hasWaypoints() const noexcept { return !waypoints_.empty(); }
    Position getFirstWaypoint() const { return waypoints_.front(); }
    
private:
    std::vector<Position> waypoints_;
    
    std::vector<Position> getFullWaypoints(Position robotPos) const;
};

#endif // COLLISIONS_ENGINE_PATH_CHECK_ZONE_H
