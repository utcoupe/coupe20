#ifndef COLLISIONS_OBSTACLES_STACK
#define COLLISIONS_OBSTACLES_STACK

#include "collisions/obstacle.h"

#include <chrono>
#include <forward_list>
#include <mutex>
#include <vector>

/**
 * Stores received obstacles.
 */
class ObstaclesStack {
public:
    /**
     * Initialize obstacles stack.
     */
    ObstaclesStack() : M_OBSTACLE_LIVESPAN(0.3) {}
    
    /**
     * Returns all managed obstacles
     * 
     * @return A list of obstacle
     */
    std::vector<const Obstacle*> toList() const;
    
    /**
     * Updates obstacles detected by the belt.
     * 
     * @param new_obstacles An obstacle list
     */
    void updateBeltPoints(std::vector<Obstacle>&& new_obstacles);
    
    /**
     * Updates obstacles detected by the lidar.
     * 
     * @param new_obstacles An obstacle list
     */
    void updateLidarObjects(std::vector<Obstacle>&& new_obstacles);
    
    /**
     * Updates enemies known positions (may contain our second robot).
     * 
     * @param new_obstacles An obstacle list
     */
    void updateEnemies(std::vector<Obstacle>&& new_obstacles);
    
    /**
     * Removes expired obstacles according to their lifetime.
     */
    void garbageCollect();
    
private:
    /** Maximum lifetime of an obstacle */
    const std::chrono::duration<double> M_OBSTACLE_LIVESPAN;
    
    /** Contains belt last send obstacle */
    std::forward_list<Obstacle> m_beltPoints;
    /** Contains belt last send obstacle */
    std::forward_list<Obstacle> m_lidarObjects;
    /** Contains belt last send obstacle */
    std::forward_list<Obstacle> m_enemies;
    
    mutable std::mutex m_mutex;
    
    /**
     * Removes expired obstacles from the given foward list, according to their lifetime.
     * 
     * @param list The list to clean.
     */
    void garbageCollect(std::forward_list<Obstacle>& list);
};

#endif // COLLISIONS_OBSTACLES_STACK
