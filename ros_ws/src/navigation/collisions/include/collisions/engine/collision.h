#ifndef COLLISIONS_ENGINE_COLLISION
#define COLLISIONS_ENGINE_COLLISION

#include "collisions/obstacle.h"
#include "collisions/engine/constants.h"

#include <memory>

class Collision {
public:
    /**
     * Alias to store Obstacle
     * TODO As reference or std::weak_ptr
     */
    using PtrObstacle = std::shared_ptr<Obstacle>;
    
    /**
     * Main constructor of Collision.
     * 
     * @param level The danger level of collision
     * @param obstacle The corresponding Obstacle
     * @param approxDistance Distance between the robot and the obstacle
     */
    Collision(CollisionLevel level, PtrObstacle obstacle, double approxDistance):
        m_level(level), m_obstacle(obstacle), m_approxDistance(approxDistance) {}
    
    /**
     * Returns the danger level of collision
     * 
     * @return The level of collision
     */
    CollisionLevel getLevel() const noexcept { return m_level; }
    
    /**
     * Returns the corresponding obstacle
     * 
     * @return The corresponding obstacle
     */
    const PtrObstacle& getObstacle() const noexcept { return m_obstacle; }
    
    /**
     * Returns the approximative distance between the robot and the obstacle.
     * 
     * @return The distance
     */
    double getDistance() const noexcept { return m_approxDistance; }
    
private:
    /** The danger level of collision **/
    CollisionLevel m_level;
    
    /** The corresponding obstacle **/
    PtrObstacle m_obstacle;
    
    /** The distance between the robot and the obstacle **/
    double m_approxDistance;
};

#endif // COLLISIONS_ENGINE_COLLISION
