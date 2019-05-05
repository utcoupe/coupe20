#include "collisions/obstacle.h"

#include <utility>

Obstacle::Obstacle(ShapePtr&& shape, VelocityPtr&& velocity):
    m_shape(std::move(shape)), m_velocity(std::move(velocity))
{
    if (m_velocity) {
        m_velocity->setObjectPos(m_shape->getPos());
    }
}

Obstacle::Obstacle(Obstacle::ShapePtr && shape):
    m_shape(std::move(shape))
{
}


const std::vector<Obstacle::ShapePtr>& Obstacle::getVelocityShapes (double maxDist) const {
    static const std::vector<ShapePtr> s_empty_vector;
    if (m_velocity)
        return m_velocity->getShapes();
    return s_empty_vector;
}

std::chrono::duration<double> Obstacle::getAge() const {
    return std::chrono::system_clock::now() - m_spawnTime;
}
