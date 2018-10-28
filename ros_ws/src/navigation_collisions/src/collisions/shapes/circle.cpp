#include "collisions/shapes/circle.h"

using namespace CollisionsShapes;

Circle::Circle(Position pos, double radius):
    AbstractShape(pos), radius_(radius)
{}

bool Circle::isCollidingWith(const AbstractShape* otherShape) const
{
    bool collides = false;
    switch (otherShape->getShapeType()) {
        case ShapeType::SEGMENT:
            collides = isCollidingWithSegment(dynamic_cast<const Segment*>(otherShape));
            break;
        case ShapeType::RECTANGLE:
            collides = isCollidingWithRectangle(dynamic_cast<const Rectangle*>(otherShape));
            break;
        case ShapeType::CIRCLE:
            collides = isCollidingWithCircle(dynamic_cast<const Circle*>(otherShape));
            break;
        default: // should not happen
            collides = otherShape->isCollidingWith(this);
    }
    return collides;
}

bool Circle::isCollidingWithSegment(const Segment* otherSeg) const
{
    Rectangle newRect(
        otherSeg->getPos(),
        radius_ * 2,
        otherSeg->getLength() + 2 * radius_
    );
    return isCollidingWithRectangle(&newRect);
}

bool Circle::isCollidingWithRectangle(const Rectangle* otherRect) const
{
    Rectangle newRect(
        otherRect->getPos(),
        otherRect->getWidth() + radius_ * 2,
        otherRect->getHeight() + radius_ * 2
    );
    return newRect.isInRect(pos_);
}

bool Circle::isCollidingWithCircle(const Circle* otherCirc) const
{
    double dist = otherCirc->getPos().norm2Dist(pos_);
    return dist <= radius_ + otherCirc->getRadius();
}

