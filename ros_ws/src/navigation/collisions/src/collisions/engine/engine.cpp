#include "collisions/engine/engine.h"

#include "collisions/obstacle.h"

#include <cmath>

using namespace CollisionResolver;

std::vector<const Obstacle*> CollisionResolver::findCollisions(const std::vector<PtrShape>& robotShapes, const std::vector<const Obstacle*>& obstacleShapes) {
    std::vector<const Obstacle*> collisions;
    for (const auto& robotShape: robotShapes) {
        for (const auto* obstShape: obstacleShapes) {
            if (robotShape->isCollidingWith(obstShape->getShape())) {
                collisions.push_back(obstShape);
            } else {
                for (const auto& velShape: obstShape->getVelocityShapes()) {
                    if (robotShape->isCollidingWith(*velShape)) {
                        collisions.push_back(obstShape);
                    }
                }
            }
        }
    }
    return collisions;
}
