#include "collisions/shapes/segment.h"

#include <cmath>

using namespace CollisionsShapes;

Segment::Segment(const Point &firstPoint, const Point &lastPoint) noexcept:
        m_firstPoint(firstPoint), m_lastPoint(lastPoint) {
    m_length = m_firstPoint.norm2Dist(m_lastPoint);
    m_pos = Position(
            (m_firstPoint + m_lastPoint) / 2,
            std::atan2(m_lastPoint.getY() - m_firstPoint.getY(), m_lastPoint.getX() - m_firstPoint.getX())
    );
}

bool Segment::isCollidingWith(const CollisionsShapes::AbstractShape &otherShape) const {
    if (otherShape.getShapeType() != ShapeType::SEGMENT)
        return otherShape.isCollidingWith(*this);

    const auto &otherSeg = dynamic_cast<const CollisionsShapes::Segment &>(otherShape);
    // https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
    auto ccw = [](Point a, Point b, Point c) {
        return (
                (c.getY() - a.getY()) * (b.getX() - a.getX())
                > (b.getY() - a.getY()) * (c.getX() - a.getX())
        );
    };
    const bool cond1 = (
            ccw(m_firstPoint, otherSeg.getFirstPoint(), otherSeg.getLastPoint())
            != ccw(m_lastPoint, otherSeg.getFirstPoint(), otherSeg.getLastPoint())
    );
    const bool cond2 = (
            ccw(m_firstPoint, m_lastPoint, otherSeg.getFirstPoint())
            != ccw(m_firstPoint, m_lastPoint, otherSeg.getLastPoint())
    );
    return cond1 && cond2;
}
