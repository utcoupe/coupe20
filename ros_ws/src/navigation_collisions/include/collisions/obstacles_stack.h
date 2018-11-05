#ifndef COLLISIONS_OBSTACLES_STACK
#define COLLISIONS_OBSTACLES_STACK

#include "collisions/obstacle.h"

#include <chrono>
#include <forward_list>
#include <memory>
#include <vector>

class ObstacleStack {
public:
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    ObstacleStack() = default;
    
    std::vector<ObstaclePtr> toList() const;
    
    void updateBeltPoints(const std::vector<ObstaclePtr>& new_obstacles);
    void updateLidarObjects(const std::vector<ObstaclePtr>& new_obstacles);
    void updateEnemies(const std::vector<ObstaclePtr>& new_obstacles);
    
    void garbageCollect();
    
private:
    const std::chrono::duration<double> OBSTACLE_LIVESPAN = 0.3;
    std::forward_list<ObstaclePtr> beltPoints_, lidarObjects_, enemies_;
    
    void garbageCollect(std::forward_list<ObstaclePtr> list);
};

#endif // COLLISIONS_OBSTACLES_STACK
