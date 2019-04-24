#ifndef COLLISIONS_SHAPES_CIRCLE_H
#define COLLISIONS_SHAPES_CIRCLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/shapes/segment.h"
#include "collisions/shapes/rectangle.h"

namespace CollisionsShapes {

class Circle: public AbstractShape {
public:
    Circle(Position pos, double radius) noexcept:
        AbstractShape(pos), m_radius(radius)
    {}
    ~Circle() override = default;
    
    bool isCollidingWith(const AbstractShape* otherShape) const override;
    ShapeType getShapeType() const override { return ShapeType::CIRCLE; }
    
    double getRadius() const noexcept { return m_radius; }
    
private:
    double m_radius;
    
    bool m_isCollidingWithSegment(const Segment* otherSeg)      const noexcept;
    bool m_isCollidingWithRectangle(const Rectangle* otherRect) const noexcept;
    bool m_isCollidingWithCircle(const Circle* otherCirc)       const noexcept {
        double dist = otherCirc->getPos().norm2Dist(m_pos);
        return dist <= m_radius + otherCirc->getRadius();
    }
};
    
} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_CIRCLE_H
