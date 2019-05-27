#ifndef COLLISIONS_SHAPES_RECTANGLE_H
#define COLLISIONS_SHAPES_RECTANGLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/shapes/segment.h"

#include <vector>

namespace CollisionsShapes {
    
class Rectangle: public AbstractShape {
public:
    Rectangle(const Position& pos, double width, double height) noexcept:
        AbstractShape(pos), m_width(width), m_height(height)
    {}
    ~Rectangle() override = default;
    
    bool isCollidingWith(const AbstractShape& otherShape) const override;
    ShapeType getShapeType() const override { return ShapeType::RECTANGLE; }
    
    std::vector<Segment> toSegments() const;
    bool isInRect(Position pos) const;
    
    double getWidth()  const noexcept { return m_width; }
    double getHeight() const noexcept { return m_height; }
    
private:
    double m_width, m_height;
    
    std::vector<Point> m_getCorners() const;
    bool m_isCollidingWithSegment(const Segment& otherSeg) const;
    bool m_isCollidingWithRectangle(const Rectangle& otherRect) const;
};
    
} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_RECTANGLE_H
