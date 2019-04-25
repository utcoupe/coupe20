#include "collisions/robot.h"

#include "collisions/shapes/rectangle.h"

#include <memory>

using namespace std;

Robot::Robot(double width, double height):
    Obstacle(
        make_shared<CollisionsShapes::Rectangle>(Position(), width, height),
        make_unique<ObstacleVelocity>(width, height)
    ),
    m_pathCheckZone(width, height, CollisionLevel::LEVEL_DANGER)
{
}

std::vector<Robot::ShapePtr> Robot::getMainShapes()
{
    // TODO + static shape ?
    return m_velocity->getShapes(getMaxMainDist());
}


std::vector<Robot::ShapePtr> Robot::getPathShapes()
{
    return m_pathCheckZone.getShapes(m_shape->getPos());
}

std::vector<Collision> Robot::checkCollisions(std::vector<ObstaclePtr> obstacles)
{
    // TODO
//     auto collisionsVel = velocity_.checkCollisions(pos_, obstacles);
    auto collisionsPath = m_pathCheckZone.checkCollisions(m_shape->getPos(), obstacles);
//     collisionsVel.insert(collisionsVel.end(), collisionsPath.begin(), collisionsPath.end());
//     return collisionsVel;
    return collisionsPath;
}


double Robot::getMaxMainDist() const
{
    if (m_pathCheckZone.hasWaypoints())
        return m_shape->getPos().norm2Dist(m_pathCheckZone.getFirstWaypoint());
    else
        return -1.0;
}
