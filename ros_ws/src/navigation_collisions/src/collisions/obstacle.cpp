#include "collisions/obstacle.h"

Obstacle::Obstacle(ShapePtr shape, VelocityPtr velocity):
    shape_(shape), velocity_(velocity)
{
    velocity_->setObjectPos(shape->getPos());
}

std::vector<Obstacle:ShapePtr> Obstacle::getVelocityShapes (double maxDist) {
    if (velocity_)
        return velocity_->getShapes();
    return {};
}

std::chrono::duration<double, std::milli> Obstacle::getAge() const {
    return std::chrono::system_clock::now() - spawnTime_;
}
