#include "collisions/engine/engine.h"

#include "collisions/obstacle.h"

#include <cmath>

using namespace CollisionResolver;

std::vector<Obstacle *> CollisionResolver::findCollisions(
        const std::vector<PtrShape> &robotShapes,
        const std::vector<Obstacle *> &obstacles) {
    std::vector<Obstacle *> collisions;
    for (const auto &robotShape: robotShapes) {
        for (auto *obstacle: obstacles) {
            if (robotShape->isCollidingWith(obstacle->getShape())) {
                collisions.push_back(obstacle);
            } else {
                for (const auto &velShape: obstacle->getVelocityShapes()) {
                    if (robotShape->isCollidingWith(*velShape)) {
                        collisions.push_back(obstacle);
                    }
                }
            }
        }
    }
    return collisions;
}
