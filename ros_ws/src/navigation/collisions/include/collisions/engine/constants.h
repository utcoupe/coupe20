#ifndef COLLISIONS_ENGINE_CONSTANTS_H
#define COLLISIONS_ENGINE_CONSTANTS_H

#include <algorithm>

/**
 * Represents a level of dangerousness of a collision between an object and the robot.
 */
enum class CollisionLevel {
    SAFE, /// The robot won't collide with the object.
    POTENTIAL, /// There is a low probability of collision.
    STOP, /// Collision is inevitable.
    DANGER /// There is a high probability of collision.
};

/**
 * Enables the engine to discriminate the danger level between moving or static objects.
 */
enum class CollisionType {
    TYPE_STATIC, TYPE_DYNAMIC
};

namespace CollisionThresholds {
    /** Minimum linear speed (m/s) before creating a stop rect. **/
    constexpr const double VEL_MIN = 0.05;
    /** Minimum distance when linear_speed != 0. **/
    constexpr const double STOP_MIN = 0.1;
    /** Distance in meters when the robot is at 1 m/sec (linear coefficient). **/
    constexpr const double STOP_GAIN = 0.6;
    /** Maximum distance when linear_speed != 0. **/
    constexpr const double STOP_MAX = 1.0;

    /** Radius around the robot where obstacles are considered at LEVEL_DANGER vs. LEVEL_POTENTIAL. **/
    constexpr const double DANGER_RADIUS = 0.8;

    /**
     * Returns the minimum distance needed to stop the robot from its current linear speed.
     *
     * This function will return 0.0 if linearSpeed is null.
     * Else, the minimum distance is given between STOP_MIN and STOP_MAX.
     *
     * @param linearSpeed The current linear speed of the robot.
     * @return The current minimum stop distance.
     */
    constexpr double getStopDistance(double linearSpeed) noexcept {
        if (linearSpeed == 0.0)
            return 0.0; // If robot stopped, don't create stop rect distance.
        return std::max(STOP_MIN, std::min(STOP_GAIN * linearSpeed, STOP_MAX));
    }
}

#endif // COLLISIONS_ENGINE_CONSTANTS_H
