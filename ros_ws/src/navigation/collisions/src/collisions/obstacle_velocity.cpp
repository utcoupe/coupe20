#include "collisions/obstacle_velocity.h"

#include "collisions/engine/constants.h"
#include "collisions/shapes/rectangle.h"

ObstacleVelocity::ObstacleVelocity(double width, double height, double velLinear, double velAngular,
                                   const Position& objectPos) noexcept:
        m_width(width), m_height(height), m_velLinear(velLinear), m_velAngular(velAngular),
        m_objectPos(objectPos) {
}

const std::vector<ObstacleVelocity::ShapePtr> &ObstacleVelocity::getShapes(double maxDist) const {
    if (m_needUpdate || m_lastMaxDist != maxDist) {
        m_generateVelShapes(maxDist);
        m_lastMaxDist = maxDist;
        m_needUpdate = false;
    }
    return m_velShapes;
}

void ObstacleVelocity::m_generateVelShapes(double maxDist) const {
    m_velShapes.clear();
    if (abs(m_velLinear) < CollisionThresholds::VEL_MIN) {
        return;
    }

    auto expansionDist = CollisionThresholds::getStopDistance(m_velLinear);
    if (maxDist != EXPANSION_DIST_DISABLED) {
        // if set, reduce the expansion to the provided limit
        expansionDist = std::min(expansionDist, maxDist);
    }

    const double width = m_height + expansionDist;
    const double height = m_width;
    const double len = width / 2.0 - m_height / 2.0;
    const double sideAngle = m_velLinear < 0 ? M_PI : 0;
    const Position pos(
            m_objectPos.getX() + len * std::cos(m_objectPos.getAngle() + sideAngle),
            m_objectPos.getY() + len * std::sin(m_objectPos.getAngle() + sideAngle),
            m_objectPos.getAngle()
    );
    m_velShapes.emplace_back(
            std::make_unique<CollisionsShapes::Rectangle>(pos, width, height)
    );
}
