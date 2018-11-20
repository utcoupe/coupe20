#ifndef COLLISIONS_SHAPES_CIRCLE_H
#define COLLISIONS_SHAPES_CIRCLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/shapes/segment.h"
#include "collisions/shapes/rectangle.h"

namespace CollisionsShapes {

class Circle: public AbstractShape {
public:
    constexpr Circle(Position pos, double radius) noexcept:
        AbstractShape(pos), radius_(radius)
    {}
    ~Circle() override = default;
    
    bool isCollidingWith(const AbstractShape* otherShape) const override;
    ShapeType getShapeType() const override { return ShapeType::CIRCLE; }
    
    constexpr double getRadius() const noexcept { return radius_; }
    
private:
    double radius_;
    
              bool isCollidingWithSegment(const Segment* otherSeg)      const noexcept;
              bool isCollidingWithRectangle(const Rectangle* otherRect) const noexcept;
    
    constexpr bool isCollidingWithCircle(const Circle* otherCirc)       const noexcept {
        double dist = otherCirc->getPos().norm2Dist(pos_);
        return dist <= radius_ + otherCirc->getRadius();
    }
};
    
} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_CIRCLE_H
