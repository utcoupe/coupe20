#include "collisions/engine/velocity_check_zone.h"

#include "collisions/engine/constants.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/obstacle_velocity.h"


#include <geometry_tools/position.h>

using namespace CollisionsShapes;

std::vector<Collision> VelocityCheckZone::checkCollisions(const std::vector<ObstaclePtr>& obstacles) const {
    std::vector<Collision> collisions;
    for (auto obst: CollisionResolver::findCollisions(m_robotVelocity.getShapes(), obstacles)) {
        double approxDist = m_robotPos.norm2Dist(obst->getPos());
        collisions.emplace_back(CollisionLevel::LEVEL_STOP, obst, approxDist);
    }
    return collisions;
}
