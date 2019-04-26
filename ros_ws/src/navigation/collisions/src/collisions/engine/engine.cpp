#include "collisions/engine/engine.h"

#include <ros/console.h>

#include <cmath>

using namespace CollisionResolver;

std::vector<PtrObstacle> CollisionResolver::findCollisions(const std::vector<PtrShape>& robotShapes, const std::vector<PtrObstacle>& obstacleShapes) {
    std::vector<PtrObstacle> collisions;
    for (const auto& robotShape: robotShapes) {
        for (const auto& obstShape: obstacleShapes) {
            bool intersecting = robotShape->isCollidingWith(& obstShape->getShape());
            
            if (intersecting) {
                collisions.push_back(obstShape);
            } else {
                for (const auto& velShape: obstShape->getVelocityShapes()) {
                    if (robotShape->isCollidingWith(velShape.get())) {
                        collisions.push_back(obstShape);
                    }
                }
            }
        }
    }
    return collisions;
}
