#include "collisions/shapes/rectangle.h"

#include <cmath>

using namespace CollisionsShapes;

bool Rectangle::isCollidingWith(const CollisionsShapes::AbstractShape* otherShape) const
{
    bool collides;
    switch (otherShape->getShapeType()) {
    case ShapeType::SEGMENT:
        collides = m_isCollidingWithSegment(dynamic_cast<const Segment*>(otherShape));
        break;
    case ShapeType::RECTANGLE:
        collides = m_isCollidingWithRectangle(dynamic_cast<const Rectangle*>(otherShape));
        break;
    default:
        collides = otherShape->isCollidingWith(this);
    }
    return collides;
}

std::vector<Segment> Rectangle::toSegments() const
{
    std::vector<Segment> segments;
    auto corners = m_getCorners();
    for (unsigned idCorner = 0; idCorner < corners.size(); idCorner++) {
        segments.emplace_back(corners[idCorner], corners[idCorner + 1]);
    }
    return segments;
}

bool Rectangle::isInRect(Position pos) const {
    double phi = std::atan2(
        pos.getY() - m_pos.getY(),
        pos.getX() - m_pos.getX()
    );
    if (phi < 0) {
        phi += 2 * M_PI;
    }
    double angle = m_pos.getAngle();
    double dist = pos.norm2Dist(m_pos);
    Point localPoint(
        dist * std::cos(phi - angle),
        dist * std::sin(phi - angle)
    );
    return (
           std::abs(localPoint.getX()) * 2.0 <= m_width
        && std::abs(localPoint.getY()) * 2.0 <= m_height
    );
}

std::vector<Point> CollisionsShapes::Rectangle::m_getCorners() const
{
    std::vector<Point> corners;
    double len = std::sqrt(std::pow(m_width / 2.0, 2) + std::pow(m_height / 2.0, 2));
    double cornerPhi = std::atan2(m_height, m_width);
    for (double anglePhi : {0.0, M_PI}) {
        for (auto i: {0, 1}) {
            int factor = ( i + 1) % 2 == 0 ? 1 : -1;
            double phi = anglePhi + factor * cornerPhi;
            corners.emplace_back(
                m_pos.getX() + len * std::cos(phi + m_pos.getAngle()),
                m_pos.getY() + len * std::sin(phi + m_pos.getAngle())
            );
        }
    }
    return corners;
}

bool Rectangle::m_isCollidingWithSegment(const CollisionsShapes::Segment* otherSeg) const
{
    if (isInRect(otherSeg->getPos()))
        return true;
    for (const auto seg: toSegments())
        if (seg.isCollidingWith(otherSeg))
            return true;
    return false;
}

bool Rectangle::m_isCollidingWithRectangle(const CollisionsShapes::Rectangle* otherRect) const
{
    if (isInRect(otherRect->getPos()))
        return true;
    for (const auto seg: otherRect->toSegments())
        if (m_isCollidingWithSegment(&seg))
            return true;
    return false;
}

