#ifndef COLLISIONS_SHAPES_ABSTRACT_SHAPE_H
#define COLLISIONS_SHAPES_ABSTRACT_SHAPE_H

#include "collisions/position.h"

namespace CollisionsShapes {
    
enum class ShapeType { UNDEFINED, SEGMENT, RECTANGLE, CIRCLE };

class AbstractShape {
public:
    AbstractShape(Position pos = {}): pos_(pos) {}
    
    virtual bool isCollidingWith(const AbstractShape* otherShape) const = 0;
    virtual ShapeType getShapeType() const = 0;
    
    Position getPos() const { return pos_; };
    
protected:
    Position pos_;
};

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_ABSTRACT_SHAPE_H
