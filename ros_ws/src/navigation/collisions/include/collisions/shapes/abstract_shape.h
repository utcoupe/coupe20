#ifndef COLLISIONS_SHAPES_ABSTRACT_SHAPE_H
#define COLLISIONS_SHAPES_ABSTRACT_SHAPE_H

#include "geometry_tools/position.h"

namespace CollisionsShapes {
    
enum class ShapeType { UNDEFINED, SEGMENT, RECTANGLE, CIRCLE };

class AbstractShape {
public:
    AbstractShape(const Position& pos = {}) noexcept: m_pos(pos) {}
    virtual ~AbstractShape() = default;
    
    virtual bool isCollidingWith(const AbstractShape* otherShape) const = 0;
    virtual ShapeType getShapeType() const = 0;
    
    void setPos(const Position& pos) noexcept { m_pos = pos; }
    const Position& getPos() const noexcept { return m_pos; };
    
protected:
    Position m_pos;
};

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_ABSTRACT_SHAPE_H
