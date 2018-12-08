#include "collisions/robot.h"

Robot::Robot(double width, double height):
    width_(width), height_(height), velocity_(width, height), pathCheckZone_(width, height, CollisionLevel::LEVEL_DANGER)
{
}

std::vector<Robot::ShapePtr> Robot::getMainShapes()
{
    return velocity_.getShapes(pos_, getMaxMainDist());
}


std::vector<Robot::ShapePtr> Robot::getPathShapes()
{
    return pathCheckZone_.getShapes(pos_);
}

std::vector<Collision> Robot::checkCollisions(std::vector<ObstaclePtr> obstacles)
{
    auto collisionsVel = velocity_.checkCollisions(pos_, obstacles);
    auto collisionsPath = pathCheckZone_.checkCollisions(pos_, obstacles);
    collisionsVel.insert(collisionsVel.end(), collisionsPath.begin(), collisionsPath.end());
    return collisionsVel;
}


double Robot::getMaxMainDist() const
{
    if (pathCheckZone_.hasWaypoints())
        return pos_.norm2Dist(pathCheckZone_.getFirstWaypoint());
    else
        return -1.0;
}
