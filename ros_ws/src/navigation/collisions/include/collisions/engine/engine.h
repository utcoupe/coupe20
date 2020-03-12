#ifndef COLLISIONS_ENGINE_ENGINE_H
#define COLLISIONS_ENGINE_ENGINE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/obstacle.h"

#include <memory>

namespace CollisionResolver {
    using PtrShape = std::unique_ptr<CollisionsShapes::AbstractShape>;

    /**
     * Computes and returns all obstacles colliding with at least one shape of the robot.
     *
     * @param robotShapes The shapes of the robot
     * @param obstacleShapes A list of obstacles.
     * @return A list of colliding obstacles.
     */
    ObstacleRefList findCollisions(
            const std::vector<PtrShape> &robotShapes,
            const ObstacleRefList &obstacleShapes);
} // namespace CollisionResolver

#endif // COLLISIONS_ENGINE_ENGINE_H
