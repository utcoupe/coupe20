#include "collisions/obstacles_stack.h"

std::vector<ObstaclesStack::ObstaclePtr> ObstaclesStack::toList() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<ObstaclePtr> obstacles;
    obstacles.insert(obstacles.end(), m_beltPoints.begin(), m_beltPoints.end());
    obstacles.insert(obstacles.end(), m_lidarObjects.begin(), m_lidarObjects.end());
    obstacles.insert(obstacles.end(), m_enemies.begin(), m_enemies.end());
    return obstacles; // uses move semantic
}

void ObstaclesStack::updateBeltPoints(const std::vector<ObstaclePtr>& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_beltPoints.clear();
    m_beltPoints.insert_after(m_beltPoints.before_begin(), new_obstacles.begin(), new_obstacles.end());
}

void ObstaclesStack::updateLidarObjects(const std::vector<ObstaclePtr>& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_lidarObjects.clear();
    m_lidarObjects.insert_after(m_lidarObjects.before_begin(), new_obstacles.begin(), new_obstacles.end());
}

void ObstaclesStack::updateEnemies(const std::vector<ObstaclePtr>& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_enemies.clear();
    m_enemies.insert_after(m_enemies.before_begin(), new_obstacles.begin(), new_obstacles.end());
}

void ObstaclesStack::garbageCollect()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    garbageCollect(m_beltPoints);
    garbageCollect(m_lidarObjects);
    garbageCollect(m_enemies);
}

void ObstaclesStack::garbageCollect(std::forward_list<ObstaclePtr>& list)
{
    auto lifespan = M_OBSTACLE_LIVESPAN; // C++ limitation (as of C++14) =(
    list.remove_if([lifespan] (ObstaclePtr obstacle) {
        return obstacle->getAge() > lifespan;
    });
}

