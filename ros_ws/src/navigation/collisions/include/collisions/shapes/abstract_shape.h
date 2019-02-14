#ifndef COLLISIONS_SHAPES_ABSTRACT_SHAPE_H
#define COLLISIONS_SHAPES_ABSTRACT_SHAPE_H

#include "geometry_tools/position.h"

namespace CollisionsShapes {
    
enum class ShapeType { UNDEFINED, SEGMENT, RECTANGLE, CIRCLE };

class AbstractShape {
public:
    AbstractShape(Position pos = {}) noexcept: pos_(pos) {}
    virtual ~AbstractShape() = default;
    
    virtual bool isCollidingWith(const AbstractShape* otherShape) const = 0;
    virtual ShapeType getShapeType() const = 0;
    
    Position getPos() const noexcept { return pos_; };
    
protected:
    Position pos_;
};

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_ABSTRACT_SHAPE_H
