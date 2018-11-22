#include "collisions/shapes/rectangle.h"

#include <cmath>

using namespace CollisionsShapes;

bool Rectangle::isCollidingWith(const CollisionsShapes::AbstractShape* otherShape) const
{
    bool collides;
    switch (otherShape->getShapeType()) {
    case ShapeType::SEGMENT:
        collides = isCollidingWithSegment(dynamic_cast<const Segment*>(otherShape));
        break;
    case ShapeType::RECTANGLE:
        collides = isCollidingWithRectangle(dynamic_cast<const Rectangle*>(otherShape));
        break;
    default:
        collides = otherShape->isCollidingWith(this);
    }
    return collides;
}

std::vector<Segment> Rectangle::toSegments() const
{
    std::vector<Segment> segments;
    auto corners = getCorners();
    for (unsigned idCorner = 0; idCorner < corners.size(); idCorner++) {
        segments.emplace_back(corners[idCorner], corners[idCorner + 1]);
    }
    return segments;
}

bool Rectangle::isInRect(Position pos) const {
    double phi = std::atan2(
        pos.getY() - pos_.getY(),
        pos.getX() - pos_.getX()
    );
    if (phi < 0) {
        phi += 2 * M_PI;
    }
    double angle = pos_.getAngle();
    double dist = pos.norm2Dist(pos_);
    Point localPoint(
        dist * std::cos(phi - angle),
        dist * std::sin(phi - angle)
    );
    return (
           std::abs(localPoint.getX()) * 2.0 <= width_
        && std::abs(localPoint.getY()) * 2.0 <= height_
    );
}

std::vector<Point> CollisionsShapes::Rectangle::getCorners() const
{
    std::vector<Point> corners;
    double len = std::sqrt(std::pow(width_ / 2.0, 2) + std::pow(height_ / 2.0, 2));
    double cornerPhi = std::atan2(height_, width_);
    for (double anglePhi : {0.0, M_PI}) {
        for (auto i: {0, 1}) {
            int factor = ( i + 1) % 2 == 0 ? 1 : -1;
            double phi = anglePhi + factor * cornerPhi;
            corners.emplace_back(
                pos_.getX() + len * std::cos(phi + pos_.getAngle()),
                pos_.getY() + len * std::sin(phi + pos_.getAngle())
            );
        }
    }
    return corners;
}

bool Rectangle::isCollidingWithSegment(const CollisionsShapes::Segment* otherSeg) const
{
    if (isInRect(otherSeg->getPos()))
        return true;
    for (const auto seg: toSegments())
        if (seg.isCollidingWith(otherSeg))
            return true;
    return false;
}

bool Rectangle::isCollidingWithRectangle(const CollisionsShapes::Rectangle* otherRect) const
{
    if (isInRect(otherRect->getPos()))
        return true;
    for (const auto seg: otherRect->toSegments())
        if (isCollidingWithSegment(&seg))
            return true;
    return false;
}

