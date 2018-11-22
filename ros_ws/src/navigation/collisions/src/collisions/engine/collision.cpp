#include "collisions/engine/collision.h"

Collision::Collision(CollisionLevel level, PtrObstacle obstacle, double approxDistance):
    level_(level), obstacle_(obstacle), approxDistance_(approxDistance)
{}
