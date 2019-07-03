#include "collisions/robot.h"

#include "collisions/engine/constants.h"
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

void Robot::checkCollisions(const std::vector<Obstacle*>& obstacles) const {
    m_velocityCheckZone.checkCollisions(obstacles);
    m_pathCheckZone.checkCollisions(obstacles);
}


double Robot::getMaxMainDist() const {
    return (
        m_pathCheckZone.hasWaypoints()
        ? m_shape->getPos().norm2Dist(m_pathCheckZone.getFirstWaypoint())
        : -1.0
    );
}
