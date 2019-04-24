#ifndef COLLISIONS_SHAPES_SEGMENT_H
#define COLLISIONS_SHAPES_SEGMENT_H

#include "collisions/shapes/abstract_shape.h"
#include "geometry_tools/point.h"

namespace CollisionsShapes {

class Segment: public AbstractShape {
public:
    Segment(Point firstPoint, Point lastPoint) noexcept;
    ~Segment() override = default;
    
    bool isCollidingWith(const AbstractShape* otherShape) const override;
    ShapeType getShapeType() const override { return ShapeType::SEGMENT; }
    
    Point  getFirstPoint() const noexcept { return m_firstPoint; }
    Point  getLastPoint()  const noexcept { return m_lastPoint; }
    double getLength()    const noexcept { return m_length; }
    
protected:
    Point m_firstPoint, m_lastPoint;
    double m_length;
};

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_SEGMENT_H
