#ifndef COLLISIONS_SHAPES_CIRCLE_H
#define COLLISIONS_SHAPES_CIRCLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/shapes/segment.h"
#include "collisions/shapes/rectangle.h"

namespace CollisionsShapes {

    /**
     * Represents a shape as a circle (or disc).
     */
    class Circle : public AbstractShape {
    public:
        /**
         * Initializes a circle.
         *
         * It is assumed that radius is greater or equal to 0!
         *
         * @param pos Center position of the circle.
         * @param radius Radius of the circle.
         */
        Circle(const Position &pos, double radius) noexcept:
                AbstractShape(pos), m_radius(radius) {}

        /**
         * We leave the default destructor.
         */
        ~Circle() override = default;

        /**
         * Checks if the circle is colliding with another shape.
         *
         * In the case otherShape.getShapeType() is neither ShapeType::CIRCLE, ShapeType::SEGMENT, nor
         * ShapeType::Rectangle, AbstractShape::isCollidingWith is called on the other shape with the current circle in
         * parameter.
         *
         * @param otherShape The shape to check any collisions
         * @return true if they are colliding, else false.
         */
        bool isCollidingWith(const AbstractShape &otherShape) const override;

        /**
         * Returns ShapeType::CIRCLE (constant).
         *
         * @return Circle shape type.
         */
        ShapeType getShapeType() const override { return ShapeType::CIRCLE; }

        /**
         * Returns circle's radius.
         *
         * @return Circle's radius.
         */
        double getRadius() const noexcept { return m_radius; }

    private:
        /** Circle's radius. **/
        double m_radius;

        /**
         * Checks if a segment is colliding with this circle.
         *
         * @param otherSeg The segment to check collisions.
         * @return true if they are colliding, else false.
         */
        bool m_isCollidingWithSegment(const Segment &otherSeg) const noexcept;

        /**
         * Checks if a rectangle is colliding with this circle.
         *
         * @param otherRect The rectangle to check collisions.
         * @return true if they are colliding, else false.
         */
        bool m_isCollidingWithRectangle(const Rectangle &otherRect) const noexcept;

        /**
         * Checks if another circle is colliding with this circle.
         *
         * @param otherCircle The circle to check collisions.
         * @return true if they are colliding, else false.
         */
        bool m_isCollidingWithCircle(const Circle &otherCircle) const noexcept {
            return otherCircle.getPos().norm2Dist(m_pos.toPoint()) <= m_radius + otherCircle.getRadius();
        }
    };

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_CIRCLE_H
