#include "collisions/engine/velocity.h"

std::vector<Velocity::ShapePtr> Velocity::getShapes(Position objectPos, double maxDist)
{
    return checkZone_.getShapes(objectPos, velLinear_, velAngular_, maxDist);
}

std::vector<Collision> Velocity::checkCollisions(Position objectPos, std::vector<ObstaclePtr> obstacles)
{
    return checkZone_.checkCollisions(objectPos, obstacles, velLinear_, velAngular_);
}
