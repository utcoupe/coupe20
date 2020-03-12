#ifndef COLLISIONS_OBSTACLE_H
#define COLLISIONS_OBSTACLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/obstacle_velocity.h"
#include "collisions/engine/collision.h"

#include <chrono>
#include <memory>

class Obstacle {
public:
    /**
     * Alias to manipulate shapes
     */
    using ShapePtr = std::unique_ptr<CollisionsShapes::AbstractShape>;

    /**
     * Alias to manipulate obstacle velocity
     */
    using VelocityPtr = std::unique_ptr<ObstacleVelocity>;

    /**
     * Main constructor of Obstacle
     * 
     * @param shape The shape of the obstacle
     * @param velocity The velocity of the obstacle
     */
    Obstacle(ShapePtr &&shape, VelocityPtr &&velocity);

    /**
     * Main constructor of Obstacle
     * 
     * @param shape The shape of the obstacle
     */
    Obstacle(ShapePtr &&shape);

    /**
     * Returns the static shape of the obstacle
     * 
     * @return The shape
     */
    const CollisionsShapes::AbstractShape &getShape() const noexcept { return *m_shape; }

    /**
     * Returns the velocity shapes of the obstacle.
     * 
     * The maxDist correspond to the maximum projection of the velocity shape. When maxDist = -1.0, it is deactivated.
     * 
     * @param maxDist The maximum projection distance
     * @return The shapes describing the theoretical object velocity
     */
    const std::vector<ShapePtr> &getVelocityShapes(double maxDist = -1.0) const;

    /**
     * Returns the object position
     * 
     * @return The object position
     */
    const Position &getPos() const { return m_shape->getPos(); }

    /**
     * Returns the object lifetime since its creation
     * 
     * @return The object age
     */
    std::chrono::duration<double> getAge() const;

    /**
     * Returns the latest collison data
     * @return Current collision data
     */
    Collision getCollisionData() const noexcept { return m_collisionData; }

    /**
     * Updates collision data.
     * @param collisionData The new collision data
     */
    void setCollisionData(Collision collisionData) noexcept { m_collisionData = collisionData; }

protected:
    /** The main shape of the object **/
    ShapePtr m_shape;
    /** The velocity of the object **/
    VelocityPtr m_velocity;

private:
    /** The object birthtime **/
    const std::chrono::system_clock::time_point m_spawnTime = std::chrono::system_clock::now();
    /** Collision data **/
    Collision m_collisionData;
};

using ObstacleRefList = std::vector<std::reference_wrapper<Obstacle>>;

#endif // COLLISIONS_OBSTACLE_H
