#ifndef COLLISIONS_ENGINE_ENGINE_H
#define COLLISIONS_ENGINE_ENGINE_H

#include "collisions/engine/constants.h"
#include "collisions/obstacle.h"
#include "collisions/shapes/abstract_shape.h"

#include <memory>

namespace CollisionResolver {
    using PtrShape = std::unique_ptr<CollisionsShapes::AbstractShape>;
    using PtrObstacle = std::shared_ptr<Obstacle>;
    std::vector<PtrObstacle> findCollisions(const std::vector<PtrShape>& robotShapes, const std::vector<PtrObstacle>& obstacleShapes);
} // namespace CollisionResolver

#endif // COLLISIONS_ENGINE_ENGINE_H
