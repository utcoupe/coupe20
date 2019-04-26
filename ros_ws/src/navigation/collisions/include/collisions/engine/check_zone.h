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

class CheckZone {
public:
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    using ShapePtr = std::unique_ptr<CollisionsShapes::AbstractShape>;
    
    CheckZone(CollisionLevel level, const Position& robotPos) noexcept:
        m_level(level), m_robotPos(robotPos){}
    
    virtual ~CheckZone() = default;
    
    virtual const std::vector<ShapePtr>& getShapes() const = 0;
    virtual std::vector<Collision> checkCollisions(const std::vector<ObstaclePtr>& obstacles) const = 0;
    
    CollisionLevel  getLevel()  const noexcept { return m_level; }
    
protected:
    CollisionLevel m_level;
    const Position& m_robotPos;
};

#endif // COLLISIONS_CHECK_ZONE_H
