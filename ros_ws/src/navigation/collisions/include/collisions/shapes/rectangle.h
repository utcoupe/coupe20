#ifndef COLLISIONS_SHAPES_RECTANGLE_H
#define COLLISIONS_SHAPES_RECTANGLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/shapes/segment.h"

#include <vector>

namespace CollisionsShapes {
    
class Rectangle: public AbstractShape {
public:
    Rectangle(Position pos, double width, double height) noexcept:
        AbstractShape(pos), width_(width), height_(height)
    {}
    ~Rectangle() override = default;
    
    bool isCollidingWith(const AbstractShape* otherShape) const override;
    ShapeType getShapeType() const override { return ShapeType::RECTANGLE; }
    
    std::vector<Segment> toSegments() const;
    bool isInRect(Position pos) const;
    
    double getWidth()  const noexcept { return width_; }
    double getHeight() const noexcept { return height_; }
    
private:
    double width_, height_;
    
    std::vector<Point> getCorners() const;
    bool isCollidingWithSegment(const Segment* otherSeg) const;
    bool isCollidingWithRectangle(const Rectangle* otherRect) const;
};
    
} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_RECTANGLE_H
