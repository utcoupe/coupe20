#ifndef COLLISIONS_ROBOT_H
#define COLLISIONS_ROBOT_H

#include "collisions/engine/path_check_zone.h"
#include "collisions/engine/collision.h"
#include "collisions/engine/velocity_check_zone.h"
#include "collisions/shapes/abstract_shape.h"
#include "collisions/obstacle.h"

#include <functional>
#include <memory>
#include <utility>
#include <vector>

class Position;

class Robot : protected Obstacle {
public:
    /**
     * Alias to manipulate shapes
     */
    using ShapePtr = std::unique_ptr<CollisionsShapes::AbstractShape>;

    /**
     * Status of the robot
     */
    enum class NavStatus {
        IDLE, NAVIGATING
    };

    /**
     * Initialize the shapes of the robot.
     * 
     * Only Rectangle shape is available for now, it may change in the future.
     * 
     * @param width The biggest width of the robot
     * @param height The biggest height of the robot
     */
    Robot(double width, double height);

    /**
     * Updates robot position.
     * 
     * @param pos The new position
     */
    void setPos(const Position &pos) noexcept {
        m_shape->setPos(pos);
        m_velocity->setObjectPos(pos);
    }

    /**
     * Updates robot velocity.
     * 
     * @param velLinear The linear velocity
     * @param velAngular The angular velovity
     */
    void updateVelocity(double velLinear, double velAngular) noexcept {
        m_velocity->setVelocity(velLinear, velAngular);
    }

    /**
     * Updates robot status.
     * 
     * @param status The new status
     */
    void updateStatus(NavStatus status) noexcept { m_navStatus = status; }

    /**
     * Updates robot path waypoints.
     * 
     * @param newWaypoints The new waypoints
     */
    void updateWaypoints(const std::vector<Position> &newWaypoints) {
        m_pathCheckZone.updateWaypoints(newWaypoints);
    }

    /**
     * Computes and returns robot velocity shapes.
     * 
     * @return The computed shapes.
     */
    const std::vector<ShapePtr> &getMainShapes() const;

    /**
     * Computes and returns robot path shapes.
     * 
     * @return The computed shapes.
     */
    const std::vector<ShapePtr> &getPathShapes() const {
        return m_pathCheckZone.getShapes();
    }

    /**
     * Check all possible collisions between the robot and all other obstacles.
     *
     * Obstacles are updated.
     */
    void checkCollisions(const ObstacleRefList &obstacles) const;

private:
    /** Robot status **/
    NavStatus m_navStatus = NavStatus::IDLE;
    /** Collision path checker **/
    PathCheckZone m_pathCheckZone;
    /** Velocity shapes checker **/
    VelocityCheckZone m_velocityCheckZone;

    /**
     * Returns the distance between the robot and the path first waypoint if it exists, else return -1.0.
     *
     * Its role is to set the maximum distance to check in front of the robot.
     * 
     * @return The maximum distance
     */
    double getMaxMainDist() const;
};

#endif // COLLISIONS_ROBOT_H
