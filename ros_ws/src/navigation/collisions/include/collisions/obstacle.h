#ifndef COLLISIONS_OBSTACLE_H
#define COLLISIONS_OBSTACLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/obstacle_velocity.h"

#include <chrono>
#include <memory>

class Obstacle {
public:
    /**
     * Alias to manipulate shapes
     * TODO replace with reference
     */
    using ShapePtr = std::shared_ptr<CollisionsShapes::AbstractShape>;
    
    /**
     * Alias to manipulate obstacle velocity
     * TODO replace with reference
     */
    using VelocityPtr = std::shared_ptr<ObstacleVelocity>;
    
    /**
     * Main constructor of Obstacle
     * 
     * @param shape The shape of the obstacle
     * @param velocity The velocity of the obstacle
     */
    Obstacle(ShapePtr shape, VelocityPtr velocity = nullptr);
    
    /**
     * Returns the static shape of the obstacle
     * 
     * TODO As const reference
     * 
     * @return Obstacle::ShapePtr
     */
    ShapePtr getShape() const noexcept { return m_shape; }
    
    /**
     * Returns the velocity shapes of the obstacle.
     * 
     * The maxDist correspond to the maximum projection of the velocity shape. When maxDist = -1.0, it is deactivated.
     * 
     * @param maxDist The maximum projection distance
     * @return The shapes describing the theroretical object velocity
     */
    std::vector<ShapePtr> getVelocityShapes (double maxDist = -1.0);
    
    /**
     * Retuns the object position
     * 
     * @return The object position
     */
    Position getPos() const { return m_shape->getPos(); }
    
    /**
     * Returns the object lifetime since its creation
     * 
     * @return The object age
     */
    std::chrono::duration<double> getAge() const;
    
protected:
    /** The main shape of the object **/
    ShapePtr m_shape;
    /** The velocity of the object **/
    VelocityPtr m_velocity;
    /** The object birthtime **/
    const std::chrono::system_clock::time_point m_spawnTime = std::chrono::system_clock::now();
};

#endif // COLLISIONS_OBSTACLE_H
