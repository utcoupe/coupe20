#ifndef COLLISIONS_SHAPES_SEGMENT_H
#define COLLISIONS_SHAPES_SEGMENT_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/point.h"

namespace CollisionsShapes {

class Segment: public AbstractShape {
public:
    Segment(Point firstPoint, Point lastPoint) noexcept;
    ~Segment() override = default;
    
    bool isCollidingWith(const AbstractShape* otherShape) const override;
    ShapeType getShapeType() const override { return ShapeType::SEGMENT; }
    
    constexpr Point  getFirstPoint() const noexcept { return firstPoint_; }
    constexpr Point  getLastPoint()  const noexcept { return lastPoint_; }
    constexpr double getLength()    const noexcept { return length_; }
    
protected:
    Point firstPoint_, lastPoint_;
    double length_;
};

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_SEGMENT_H
