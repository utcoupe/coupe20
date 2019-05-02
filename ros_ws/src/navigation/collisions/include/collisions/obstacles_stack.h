#ifndef COLLISIONS_OBSTACLES_STACK
#define COLLISIONS_OBSTACLES_STACK

#include "collisions/obstacle.h"

#include <chrono>
#include <forward_list>
#include <memory>
#include <mutex>
#include <vector>

/**
 * Stores received obstacles.
 */
class ObstaclesStack {
public:
    /** Alias to manipulate obstacle pointer */
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    
    /**
     * Initialize obstacles stack.
     */
    ObstaclesStack() : M_OBSTACLE_LIVESPAN(0.3) {}
    
    /**
     * Returns all managed obstacles
     * 
     * @return A list of obstacle
     */
    std::vector<ObstaclePtr> toList() const;
    
    /**
     * Updates obstacles detected by the belt.
     * 
     * @param new_obstacles An obstacle list
     */
    void updateBeltPoints(const std::vector<ObstaclePtr>& new_obstacles);
    
    /**
     * Updates obstacles detected by the lidar.
     * 
     * @param new_obstacles An obstacle list
     */
    void updateLidarObjects(const std::vector<ObstaclePtr>& new_obstacles);
    
    /**
     * Updates enemies known positions (may contain our second robot).
     * 
     * @param new_obstacles An obstacle list
     */
    void updateEnemies(const std::vector<ObstaclePtr>& new_obstacles);
    
    /**
     * Removes expired obstacles according to their lifetime.
     */
    void garbageCollect();
    
private:
    /** Maximum lifetime of an obstacle */
    const std::chrono::duration<double> M_OBSTACLE_LIVESPAN;
    
    /** Contains belt last send obstacle */
    std::forward_list<ObstaclePtr> m_beltPoints;
    /** Contains belt last send obstacle */
    std::forward_list<ObstaclePtr> m_lidarObjects;
    /** Contains belt last send obstacle */
    std::forward_list<ObstaclePtr> m_enemies;
    
    mutable std::mutex m_mutex;
    
    /**
     * Removes expired obstacles from the given foward list, according to their lifetime.
     * 
     * @param list The list to clean.
     */
    void garbageCollect(std::forward_list<ObstaclePtr>& list);
};

#endif // COLLISIONS_OBSTACLES_STACK
