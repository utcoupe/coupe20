#ifndef COLLISIONS_ENGINE_CONSTANTS_H
#define COLLISIONS_ENGINE_CONSTANTS_H

#include <algorithm>

enum class CollisionLevel { SAFE, POTENTIAL, STOP, DANGER };

enum class CollisionType {TYPE_STATIC, TYPE_DYNAMIC };

namespace CollisionThresholds {
    /** Minimum linear speed (m/s) before creating a stop rect. **/
    constexpr const double VEL_MIN     = 0.05;
    /** Minimum distance when linear_speed != 0. **/
    constexpr const double STOP_MIN    = 0.1;
    /** Distance in meters when the robot is at 1 m/sec (linear coefficient). **/
    constexpr const double STOP_GAIN   = 0.6;
    /** Maximum distance when linear_speed != 0. **/
    constexpr const double STOP_MAX    = 1.0;
    
    /** Radius around the robot where obstacles are considered at LEVEL_DANGER vs. LEVEL_POTENTIAL. **/
    constexpr const double DANGER_RADIUS = 0.8;
    
    constexpr double getStopDistance(double linearSpeed) noexcept {
        if (!linearSpeed)
            return 0.0; // If robot stopped, don't create stop rect distance.
        return std::min({STOP_MIN, STOP_GAIN * linearSpeed, STOP_MAX});
    }
}

#endif // COLLISIONS_ENGINE_CONSTANTS_H
