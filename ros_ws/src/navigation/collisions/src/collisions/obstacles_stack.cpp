#include "collisions/obstacles_stack.h"

std::vector<Obstacle*> ObstaclesStack::toList() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<Obstacle*> obstacles;
    // TODO C++17 transform_copy
    auto copyObstPtrs = [&obstacles](const auto& obstList) {
        for (auto& obst: obstList) {
            // Ugly as hell =(
            obstacles.push_back(const_cast<Obstacle*>(&obst));
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
    std::move(begin(new_obstacles), end(new_obstacles), std::front_inserter(m_beltPoints));
}

void ObstaclesStack::updateLidarObjects(std::vector<Obstacle>&& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_lidarObjects.clear();
    std::move(begin(new_obstacles), end(new_obstacles), std::front_inserter(m_lidarObjects));
}

void ObstaclesStack::updateEnemies(std::vector<Obstacle>&& new_obstacles)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_enemies.clear();
    std::move(begin(new_obstacles), end(new_obstacles), std::front_inserter(m_enemies));
}

void ObstaclesStack::garbageCollect()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    garbageCollect(m_beltPoints);
    garbageCollect(m_lidarObjects);
    garbageCollect(m_enemies);
}

void ObstaclesStack::resetCollisionData() {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto resetObstacles = [](auto& obstacles) {
        for (auto& obstacle: obstacles) {
            obstacle.setCollisionData( {} );
        }
    };
    resetObstacles(m_beltPoints);
    resetObstacles(m_lidarObjects);
    resetObstacles(m_enemies);
}

void ObstaclesStack::garbageCollect(std::forward_list<Obstacle>& list)
{
    auto lifespan = M_OBSTACLE_LIVESPAN; // C++ limitation (as of C++14) =(
    list.remove_if([lifespan] (const Obstacle& obstacle) {
        return obstacle.getAge() > lifespan;
    });
}

