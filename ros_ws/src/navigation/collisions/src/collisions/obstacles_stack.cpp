#include "collisions/obstacles_stack.h"

std::vector<const Obstacle*> ObstaclesStack::toList() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<const Obstacle*> obstacles;
    auto copyObstPtrs = [&obstacles](const auto& obstList) {
        for (const auto& obst: obstList) {
            obstacles.push_back(&obst);
        }
    };
    copyObstPtrs(m_beltPoints);
    copyObstPtrs(m_lidarObjects);
    copyObstPtrs(m_enemies);
    return obstacles; // uses move semantic
}

void ObstaclesStack::updateBeltPoints(std::vector<Obstacle>&& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_beltPoints.clear();
    for (auto&& obst : new_obstacles) {
        m_beltPoints.insert_after(m_beltPoints.before_begin(), std::move(obst));
    }
}

void ObstaclesStack::updateLidarObjects(std::vector<Obstacle>&& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_lidarObjects.clear();
    for (auto&& obst : new_obstacles) {
        m_lidarObjects.insert_after(m_lidarObjects.before_begin(), std::move(obst));
    }
}

void ObstaclesStack::updateEnemies(std::vector<Obstacle>&& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_enemies.clear();
    for (auto&& obst : new_obstacles) {
        m_enemies.insert_after(m_enemies.before_begin(), std::move(obst));
    }
}

void ObstaclesStack::garbageCollect()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    garbageCollect(m_beltPoints);
    garbageCollect(m_lidarObjects);
    garbageCollect(m_enemies);
}

void ObstaclesStack::garbageCollect(std::forward_list<Obstacle>& list)
{
    auto lifespan = M_OBSTACLE_LIVESPAN; // C++ limitation (as of C++14) =(
    list.remove_if([lifespan] (const Obstacle& obstacle) {
        return obstacle.getAge() > lifespan;
    });
}

