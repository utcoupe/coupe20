#include "collisions/obstacles_stack.h"

std::vector<ObstaclesStack::ObstaclePtr> ObstaclesStack::toList() const {
    std::lock_guard<std::mutex> lock(m_mutex_);
    std::vector<ObstaclePtr> obstacles;
    obstacles.insert(obstacles.end(), beltPoints_.begin(), beltPoints_.end());
    obstacles.insert(obstacles.end(), lidarObjects_.begin(), lidarObjects_.end());
    obstacles.insert(obstacles.end(), enemies_.begin(), enemies_.end());
    return obstacles;
}

void ObstaclesStack::updateBeltPoints(const std::vector<ObstaclePtr>& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    beltPoints_.clear();
    beltPoints_.insert_after(beltPoints_.before_begin(), new_obstacles.begin(), new_obstacles.end());
}

void ObstaclesStack::updateLidarObjects(const std::vector<ObstaclePtr>& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    lidarObjects_.clear();
    lidarObjects_.insert_after(lidarObjects_.before_begin(), new_obstacles.begin(), new_obstacles.end());
}

void ObstaclesStack::updateEnemies(const std::vector<ObstaclePtr>& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    enemies_.clear();
    enemies_.insert_after(enemies_.before_begin(), new_obstacles.begin(), new_obstacles.end());
}

void ObstaclesStack::garbageCollect()
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    garbageCollect(beltPoints_);
    garbageCollect(lidarObjects_);
    garbageCollect(enemies_);
}

void ObstaclesStack::garbageCollect(std::forward_list<ObstaclePtr>& list)
{
    auto lifespan = OBSTACLE_LIVESPAN;
    list.remove_if([lifespan] (ObstaclePtr obstacle) {
        return obstacle->getAge() > lifespan;
    });
}

