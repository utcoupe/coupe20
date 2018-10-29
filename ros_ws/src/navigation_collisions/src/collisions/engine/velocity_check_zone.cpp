#include "collisions/engine/velocity_check_zone.h"

#include "collisions/engine/check_zone.h"

#include "collisions/engine/constants.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/obstacle_velocity.h"

using namespace CollisionsShapes;

std::vector<VelocityCheckZone::ShapePtr> VelocityCheckZone::getShapes(Position robotPos, double velLinear, double velAngular, double maxDist) {
    ObstacleVelocity vel(width_, height_, velLinear, velAngular, robotPos);
    return vel.getShapes();
}

std::vector<Collision> VelocityCheckZone::checkCollisions(Position robotPos, std::vector<ObstaclePtr> obstacles) {
    return checkCollisions(robotPos, obstacles, 0.0, 0.0);
}

std::vector<Collision> VelocityCheckZone::checkCollisions(Position robotPos, std::vector<ObstaclePtr> obstacles, double velLinear, double velAngular)
{
    std::vector<Collision> collisions;
    auto shapeObstacles = getShapes(robotPos, velLinear, velAngular);
    for (auto obst: CollisionResolver::findCollisions(shapeObstacles, obstacles))
    {
        double approxDist = robotPos.norm2Dist(obst->getPos());
        collisions.emplace_back(CollisionLevel::LEVEL_STOP, obst, approxDist);
    }
    return collisions;
}
