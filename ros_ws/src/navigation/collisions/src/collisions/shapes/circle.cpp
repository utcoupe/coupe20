#include "collisions/shapes/circle.h"

using namespace CollisionsShapes;

bool Circle::isCollidingWith(const AbstractShape& otherShape) const
{
    bool collides = false;
    switch (otherShape.getShapeType()) {
        case ShapeType::SEGMENT:
            collides = m_isCollidingWithSegment(dynamic_cast<const Segment&>(otherShape));
            break;
        case ShapeType::RECTANGLE:
            collides = m_isCollidingWithRectangle(dynamic_cast<const Rectangle&>(otherShape));
            break;
        case ShapeType::CIRCLE:
            collides = m_isCollidingWithCircle(dynamic_cast<const Circle&>(otherShape));
            break;
        default: // should not happen
            collides = otherShape.isCollidingWith(*this);
    }
    return collides;
}

bool Circle::m_isCollidingWithSegment(const Segment& otherSeg) const noexcept
{
    Rectangle newRect(
        otherSeg.getPos(),
        m_radius * 2,
        otherSeg.getLength() + 2 * m_radius
    );
    return m_isCollidingWithRectangle(newRect);
}

bool Circle::m_isCollidingWithRectangle(const Rectangle& otherRect) const noexcept
{
    Rectangle newRect(
        otherRect.getPos(),
        otherRect.getWidth() + m_radius * 2,
        otherRect.getHeight() + m_radius * 2
    );
    return newRect.isInRect(m_pos);
}

