#include "collisions/engine/path_check_zone.h"

#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/circle.h"
#include "collisions/engine/engine.h"
#include "collisions/engine/constants.h"
#include "collisions/obstacle.h"

using namespace CollisionsShapes;

const std::vector<PathCheckZone::ShapePtr>& PathCheckZone::getShapes() const {
    m_shapes.clear();
    if (!m_waypoints.empty()) {
        auto prevPos = m_robotPos;
        for (unsigned idPos = 0; idPos < m_waypoints.size(); idPos++) {
            const auto& curPos = m_waypoints[idPos];
            
            if (prevPos == curPos)
                continue;
            
            double dist = prevPos.norm2Dist(curPos);
            double angle = std::atan(
                (curPos.getY() - prevPos.getY())
                / (curPos.getX() - prevPos.getX())
            );
            Position pos {(curPos + prevPos) / 2, angle};
            
            m_shapes.push_back(std::make_unique<Rectangle>(pos, dist, m_width));
            if (idPos + 1 == m_waypoints.size()) {
                m_shapes.push_back(std::make_unique<Rectangle>(
                    Position(curPos, angle),
                    m_height,
                    m_width
                ));
            }
            else {
                double ray = std::hypot(m_width, m_height) / 2.0;
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

std::vector<Collision> PathCheckZone::checkCollisions(const std::vector<const Obstacle*>& obstacles) const {
    std::vector<Collision> collisions;
    for (const auto& obst: CollisionResolver::findCollisions(getShapes(), obstacles)) {
        double approxDist = m_robotPos.norm2Dist(obst->getPos());
        auto level = (
            approxDist < CollisionThresholds::DANGER_RADIUS ?
            CollisionLevel::DANGER:
            m_level
        );
        collisions.emplace_back(level, obst, approxDist);
    }
    return collisions;
}
