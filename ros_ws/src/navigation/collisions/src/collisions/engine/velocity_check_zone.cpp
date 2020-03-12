#include "collisions/engine/velocity_check_zone.h"

#include "collisions/engine/constants.h"
#include "collisions/engine/engine.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/obstacle.h"

#include <geometry_tools/position.h>

using namespace CollisionsShapes;

void VelocityCheckZone::checkCollisions(const ObstacleRefList& obstacles) const
{
    for (auto obstacle: CollisionResolver::findCollisions(m_robotVelocity.getShapes(), obstacles)) {
        if (obstacle.get().getCollisionData().getLevel() > m_level) {
            obstacle.get().setCollisionData(Collision(
                    m_level,
                    m_robotPos.norm2Dist(obstacle.get().getPos().toPoint())
            ));
        }
    }
}
