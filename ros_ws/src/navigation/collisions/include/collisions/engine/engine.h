#ifndef COLLISIONS_ENGINE_ENGINE_H
#define COLLISIONS_ENGINE_ENGINE_H

#include "collisions/shapes/abstract_shape.h"

#include <memory>

class Obstacle;

namespace CollisionResolver {
    using PtrShape = std::unique_ptr<CollisionsShapes::AbstractShape>;
    std::vector<const Obstacle*> findCollisions(const std::vector<PtrShape>& robotShapes, const std::vector<const Obstacle*>& obstacleShapes);
} // namespace CollisionResolver

#endif // COLLISIONS_ENGINE_ENGINE_H
