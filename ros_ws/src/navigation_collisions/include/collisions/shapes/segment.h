#ifndef COLLISIONS_SHAPES_SEGMENT_H
#define COLLISIONS_SHAPES_SEGMENT_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/point.h"

namespace CollisionsShapes {

class Segment: public AbstractShape {
public:
    Segment(Point firstPoint, Point lastPoint);
    
    bool isCollidingWith(const AbstractShape* otherShape) const override;
    ShapeType getShapeType() const override { return ShapeType::SEGMENT; }
    
    Point getFirstPoint() const { return firstPoint_; }
    Point getLastPoint() const { return lastPoint_; }
    double getLength() const { return length_; }
    
protected:
    Point firstPoint_, lastPoint_;
    double length_;
};

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_SEGMENT_H
