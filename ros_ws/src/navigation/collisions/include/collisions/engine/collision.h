#ifndef COLLISIONS_ENGINE_COLLISION
#define COLLISIONS_ENGINE_COLLISION

#include "collisions/obstacle.h"
#include "collisions/engine/constants.h"

#include <memory>

class Collision {
public:
    using PtrObstacle = std::shared_ptr<Obstacle>;
    
    Collision(CollisionLevel level, PtrObstacle obstacle, double approxDistance);
    
    CollisionLevel  getLevel()      const noexcept { return level_; }
              PtrObstacle     getObstacle()   const noexcept { return obstacle_; }
    double          getDistance()   const noexcept { return approxDistance_; }
    
private:
    CollisionLevel level_;
    double approxDistance_;
    PtrObstacle obstacle_;
};

#endif // COLLISIONS_ENGINE_COLLISION
