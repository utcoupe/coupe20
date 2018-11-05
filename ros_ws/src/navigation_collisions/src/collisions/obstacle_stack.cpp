#include "collisions/obstacles_stack.h"

std::vector<ObstacleStack::ObstaclePtr> ObstacleStack::toList() const {
    std::vector<ObstaclePtr> obstacles;
    obstacles.insert(obstacles.end(), beltPoints_.begin(), beltPoints_.end());
    obstacles.insert(obstacles.end(), lidarObjects_.begin(), lidarObjects_.end());
    obstacles.insert(obstacles.end(), enemies_.begin(), enemies_.end());
    return obstacles;
}

void ObstacleStack::updateBeltPoints(const std::vector<ObstaclePtr>& new_obstacles)
{
    beltPoints_.clear();
    beltPoints_.insert_after(beltPoints_.end(), new_obstacles.begin(), new_obstacles.end());
}

void ObstacleStack::updateLidarObjects(const std::vector<ObstaclePtr>& new_obstacles)
{
    lidarObjects_.clear();
    lidarObjects_.insert_after(lidarObjects_.end(), new_obstacles.begin(), new_obstacles.end());
}

void ObstacleStack::updateEnemies(const std::vector<ObstaclePtr>& new_obstacles)
{
    enemies_.clear();
    enemies_.insert_after(enemies_.end(), new_obstacles.begin(), new_obstacles.end());
}

void ObstacleStack::garbageCollect()
{
    
}

void ObstacleStack::garbageCollect(std::forward_list<ObstaclePtr> list)
{
    auto lifespan = OBSTACLE_LIVESPAN;
    list.remove_if([lifespan] (ObstaclePtr obstacle) {
        return obstacle->getAge() > lifespan;
    });
}

