#include "collisions/shapes/segment.h"

#include <cmath>

using namespace CollisionsShapes;
Segment::Segment(Point firstPoint, Point lastPoint) noexcept:
    firstPoint_(firstPoint), lastPoint_(lastPoint)
{
    length_ = firstPoint_.norm2Dist(lastPoint_);
    pos_ = Position(
        (firstPoint_ + lastPoint_) / 2,
        std::atan2(lastPoint_.getY() - firstPoint_.getY(), lastPoint_.getX() - firstPoint_.getX())
    );
}

bool Segment::isCollidingWith(const CollisionsShapes::AbstractShape* otherShape) const
{
    if (otherShape->getShapeType() != ShapeType::SEGMENT)
        return otherShape->isCollidingWith(this);
    
    const Segment* otherSeg = dynamic_cast<const CollisionsShapes::Segment*>(otherShape);
    // https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
    auto ccw = [](Point a, Point b,  Point c) {
        return (
            (c.getY() - a.getY()) * (b.getX() - a.getX())
            > (b.getY() - a.getY()) * (c.getX() - a.getX())
        );
    };
    bool cond1 = (
           ccw(firstPoint_, otherSeg->getFirstPoint(), otherSeg->getLastPoint())
        != ccw(lastPoint_, otherSeg->getFirstPoint(), otherSeg->getLastPoint())
    );
    bool cond2 = (
           ccw(firstPoint_, lastPoint_, otherSeg->getFirstPoint())
        != ccw(firstPoint_, lastPoint_, otherSeg->getLastPoint())
    );
    return cond1 && cond2;
}
