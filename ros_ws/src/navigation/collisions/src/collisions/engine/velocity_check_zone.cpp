#include "collisions/engine/velocity_check_zone.h"

#include "collisions/engine/constants.h"
#include "collisions/engine/engine.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/obstacle.h"

#include <geometry_tools/position.h>

using namespace CollisionsShapes;

std::vector<Collision> VelocityCheckZone::checkCollisions(const std::vector<const Obstacle*>& obstacles) const {
    std::vector<Collision> collisions;
    for (auto obst: CollisionResolver::findCollisions(m_robotVelocity.getShapes(), obstacles)) {
        double approxDist = m_robotPos.norm2Dist(obst->getPos());
        collisions.emplace_back(m_level, obst, approxDist);
    }
    return collisions;
}
