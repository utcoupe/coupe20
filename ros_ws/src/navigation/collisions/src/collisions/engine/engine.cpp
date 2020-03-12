#include "collisions/engine/engine.h"

#include "collisions/obstacle.h"

#include <cmath>

using namespace CollisionResolver;

ObstacleRefList CollisionResolver::findCollisions(
        const std::vector<PtrShape> &robotShapes,
        const ObstacleRefList &obstacles) {
    ObstacleRefList collisions;
    for (const auto &robotShape: robotShapes) {
        for (auto obstacle: obstacles) {
            if (robotShape->isCollidingWith(obstacle.get().getShape())) {
                collisions.push_back(obstacle);
            } else {
                for (const auto &velShape: obstacle.get().getVelocityShapes()) {
                    if (robotShape->isCollidingWith(*velShape)) {
                        collisions.push_back(obstacle);
                    }
                }
            }
        }
    }
    return collisions;
}
