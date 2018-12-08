#include "collisions/obstacle_velocity.h"

#include "collisions/engine/constants.h"
#include "collisions/shapes/rectangle.h"

ObstacleVelocity::ObstacleVelocity(double width, double height, double velLinear, double velAngular, Position objectPos) noexcept:
    width_(width), height_(height), velLinear_(velLinear), velAngular_(velAngular),
    objectPos_(objectPos)
{
}

std::vector<ObstacleVelocity::ShapePtr> ObstacleVelocity::getShapes(double maxDist)
{
    if (needUpdate_ || lastMaxDist_ != maxDist) {
        generateVelShapes(maxDist);
        lastMaxDist_ = maxDist;
        needUpdate_ = false;
    }
    return velShapes_;
}

void ObstacleVelocity::generateVelShapes(double maxDist)
{
    velShapes_.clear();
    if (abs(velLinear_) < CollisionThresholds::VEL_MIN) {
        return;
    }
    
    auto expansionDist = CollisionThresholds::getStopDistance(velLinear_);
    if (maxDist != -1) {
        // if set, reduce the expansion to the provided limit
        expansionDist = std::min(expansionDist, maxDist);
    }
    
    double width = height_ + expansionDist;
    double height = width_;
    double len = width / 2.0 - height_ / 2.0;
    double sideAngle = velLinear_ < 0 ? M_PI : 0;
    Position pos(
        objectPos_.getX() + len * std::cos(objectPos_.getAngle() + sideAngle),
        objectPos_.getY() + len * std::sin(objectPos_.getAngle() + sideAngle),
        objectPos_.getAngle()
    );
    velShapes_.emplace_back( std::make_shared<CollisionsShapes::Rectangle>(pos, width, height) );
}
