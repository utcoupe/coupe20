#include "collisions/engine/velocity.h"

std::vector<PtrObstacle> Velocity::getShapes(Position objectPos, double maxDist)
{
    return checkZone_.getShapes(objectPos, velLinear_, velAngular_, maxDist);
}

std::vector<Collision> Velocity::checkCollisions(Position objectPos, std::vector<PtrObstacle> obstacles)
{
    return checkZone_.checkCollisions(objectPos, obstacles, velLinear_, velAngular_)
}
