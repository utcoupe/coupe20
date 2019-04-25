#include "collisions/obstacle.h"

#include <utility>

Obstacle::Obstacle(ShapePtr shape, VelocityPtr&& velocity):
    m_shape(shape), m_velocity(std::move(velocity))
{
    if (m_velocity) {
        m_velocity->setObjectPos(shape->getPos());
    }
}

std::vector<Obstacle::ShapePtr> Obstacle::getVelocityShapes (double maxDist) {
    if (m_velocity)
        return m_velocity->getShapes();
    return {};
}

std::chrono::duration<double> Obstacle::getAge() const {
    return std::chrono::system_clock::now() - m_spawnTime;
}
