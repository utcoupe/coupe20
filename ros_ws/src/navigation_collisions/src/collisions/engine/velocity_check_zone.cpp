#include "collisions/engine/velocity_check_zone.h"

#include "collisions/engine/constants.h"
#include "collisions/engine/shapes.h"
#include "collisions/engine/engine.h"

std::vector<PtrObstacle> VelocityCheckZone::getShapes(Position robotPos, double velLinear, double velAngular, double maxDist) {
    if (abs(velLinear) < CollisionThresholds::VEL_MIN) {
        return {};
    }
    auto expansionDist = CollisionThresholds::getStopDistance(velLinear);
    if (maxDist != -1) {
        // if set, reduce the expansion to the provided limit
        expansionDist = std::min(expansionDist, maxDist);
    }
    double width = height_ + expansionDist;
    double height = width_;
    double l = width / 2.0 - height_ / 2.0;
    double sideAngle = velLinear < 0 ? M_PI : 0;
    Position pos(
        robotPos.getX() + l * std::cos(robotPos.getAngle() + sideAngle),
        robotPos.getY() + l * std::sin(robotPos.getAngle() + sideAngle),
        robotPos.getAngle()
    );
    return { std::make_shared<RectObstacle>(pos, width, height) };
}

std::vector<Collision> VelocityCheckZone::checkCollisions(Position robotPos, std::vector<PtrObstacle> obstacles, double velLinear, double velAngular)
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
