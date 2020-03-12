#include "collisions/engine/path_check_zone.h"

#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/circle.h"
#include "collisions/engine/engine.h"
#include "collisions/engine/constants.h"
#include "collisions/obstacle.h"

using namespace CollisionsShapes;

const std::vector<PathCheckZone::ShapePtr> &PathCheckZone::getShapes() const {
    m_shapes.clear();
    if (!m_waypoints.empty()) {
        auto prevPos = m_robotPos;
        for (unsigned idPos = 0; idPos < m_waypoints.size(); idPos++) {
            const auto &curPos = m_waypoints[idPos];

            if (prevPos == curPos)
                continue;

            const double dist = prevPos.norm2Dist(curPos.toPoint());
            const double angle = std::atan(
                    (curPos.getY() - prevPos.getY())
                    / (curPos.getX() - prevPos.getX())
            );
            Position pos{(curPos + prevPos.toPoint()) / 2.0, angle};

            m_shapes.push_back(std::make_unique<Rectangle>(pos, dist, m_width));
            if (idPos + 1 == m_waypoints.size()) {
                m_shapes.push_back(std::make_unique<Rectangle>(
                        Position(curPos.toPoint(), angle),
                        m_height,
                        m_width
                ));
            } else {
                const double ray = std::hypot(m_width, m_height) / 2.0;
                m_shapes.push_back(std::make_unique<Circle>(
                        curPos,
                        ray
                ));
            }
            prevPos = curPos;
        }
    }
    return m_shapes;
}

void PathCheckZone::checkCollisions(const ObstacleRefList& obstacles) const
{
    for (const auto &obstacle: CollisionResolver::findCollisions(getShapes(), obstacles)) {
        const double approxDist = m_robotPos.norm2Dist(obstacle.get().getPos().toPoint());
        auto level = (
                approxDist < CollisionThresholds::DANGER_RADIUS ?
                CollisionLevel::DANGER :
                m_level
        );
        if (level > obstacle.get().getCollisionData().getLevel()) {
            obstacle.get().setCollisionData(Collision(level, approxDist));
        }
    }
}
