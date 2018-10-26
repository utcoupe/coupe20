#ifndef COLLISIONS_ENGINE_VELOCITY
#define COLLISIONS_ENGINE_VELOCITY

#include "collisions/engine/velocity_check_zone.h"

class Velocity {
public:
    Velocity(double width, double height, double velLinear = 0.0, double velAngular = 0.0):
        checkZone_(width, height, CollisionLevel::LEVEL_STOP),
        velLinear_(velLinear),
        velAngular_(velAngular_)
    {}
    
    std::vector<PtrObstacle> getShapes(Position objectPos, double maxDist = -1);
    std::vector<Collision> checkCollisions(Position objectPos, std::vector<PtrObstacle> obstacles);
        
private:
    double velLinear_, velAngular_;
    VelocityCheckZone checkZone_;
};

#endif // COLLISIONS_ENGINE_VELOCITY
