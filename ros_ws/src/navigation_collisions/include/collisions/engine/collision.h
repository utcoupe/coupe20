#ifndef COLLISIONS_ENGINE_COLLISION
#define COLLISIONS_ENGINE_COLLISION

#include "collisions/obstacle.h"
#include "collisions/engine/constants.h"

#include <memory>

class Collision {
public:
    using PtrObstacle = std::shared_ptr<Obstacle>;
    
    Collision(CollisionLevel level, PtrObstacle obstacle, double approxDistance);
    
    CollisionLevel  getLevel()      const { return level_; }
    PtrObstacle     getObstacle()   const { return obstacle_; }
    double          getDistance()   const { return approxDistance_; }
    
private:
    CollisionLevel level_;
    PtrObstacle obstacle_;
    double approxDistance_;
};

#endif // COLLISIONS_ENGINE_COLLISION
