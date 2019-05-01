#include "collisions/robot.h"

#include "collisions/shapes/rectangle.h"


#include <geometry_tools/position.h>

#include <memory>

using namespace std;

Robot::Robot(double width, double height):
    Obstacle(
        make_unique<CollisionsShapes::Rectangle>(Position(), width, height),
        make_unique<ObstacleVelocity>(width, height)
    ),
    m_pathCheckZone(CollisionLevel::POTENTIAL, m_shape->getPos(), width, height),
    m_velocityCheckZone(CollisionLevel::STOP, m_shape->getPos(), *m_velocity)
{
}

const std::vector<Robot::ShapePtr>& Robot::getMainShapes() const {
    // TODO + static shape ?
    return m_velocity->getShapes(getMaxMainDist());
}

std::vector<Collision> Robot::checkCollisions(const std::vector<ObstaclePtr>& obstacles) {
    auto&& collisionsVel = m_velocityCheckZone.checkCollisions(obstacles);
    auto&& collisionsPath = m_pathCheckZone.checkCollisions(obstacles);
    collisionsVel.insert(collisionsVel.end(), collisionsPath.begin(), collisionsPath.end());
    return collisionsVel;
}


double Robot::getMaxMainDist() const {
    return (
        m_pathCheckZone.hasWaypoints()
        ? m_shape->getPos().norm2Dist(m_pathCheckZone.getFirstWaypoint())
        : -1.0
    );
}
