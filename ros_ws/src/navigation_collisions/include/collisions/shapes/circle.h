#ifndef COLLISIONS_SHAPES_CIRCLE_H
#define COLLISIONS_SHAPES_CIRCLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/shapes/segment.h"
#include "collisions/shapes/rectangle.h"

namespace CollisionsShapes {

class Circle: public AbstractShape {
public:
    Circle(Position pos, double radius);
    
    bool isCollidingWith(const AbstractShape* otherShape) const override;
    ShapeType getShapeType() const override { return ShapeType::CIRCLE; }
    
    double getRadius() const { return radius_; }
    
private:
    double radius_;
    
    bool isCollidingWithSegment(const Segment* otherSeg) const;
    bool isCollidingWithRectangle(const Rectangle* otherRect) const;
    bool isCollidingWithCircle(const Circle* otherCirc) const;
};
    
} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_CIRCLE_H
