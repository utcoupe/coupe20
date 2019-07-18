#ifndef COLLISIONS_SHAPES_RECTANGLE_H
#define COLLISIONS_SHAPES_RECTANGLE_H

#include "collisions/shapes/abstract_shape.h"
#include "collisions/shapes/segment.h"

#include <vector>

namespace CollisionsShapes {

    /**
     * Represents a shape as a rectangle.
     */
    class Rectangle : public AbstractShape {
    public:
        /**
         * Initializes a rectangle.
         *
         * At an angle of 0.0rad, the width and the height of a rectangle is defined respectively as the distance on the
         * x and y axes. The position is the rectangle's center.
         *
         * @param pos The center of the rectangle.
         * @param width The width of the rectangle.
         * @param height The height of the rectangle.
         */
        Rectangle(const Position &pos, double width, double height) noexcept:
                AbstractShape(pos), m_width(width), m_height(height) {}

        /**
         * We leave the default destructor.
         */
        ~Rectangle() override = default;

        /**
         * Checks if the rectangle is colliding with another shape.
         *
         * If otherShape.getShapeType() is neither ShapeType::RECTANGLE, nor ShapeType::SEGMENT,
         * AbstractShape::isCollidingWith is called on otherShape with this rectangle passed in parameter.
         *
         * @param otherShape The shape to test against any collisions.
         * @return true if they are colliding, else false.
         */
        bool isCollidingWith(const AbstractShape &otherShape) const override;

        /**
         * Returns ShapeType::RRECTANGLE (constant).
         * @return ShapeType::RRECTANGLE
         */
        ShapeType getShapeType() const override { return ShapeType::RECTANGLE; }

        /**
         * Converts this rectangle to 4 segments (its sides).
         *
         * @return a list containing 4 segments describing the rectangle.
         */
        std::vector<Segment> toSegments() const;

        /**
         * Tests is a point is inside the rectangle.
         *
         * @param pos
         * @return
         */
        bool isInRect(const Point &point) const;

        /**
         * Returns the rectangle's width
         *
         * @return Rectangle's width.
         */
        double getWidth() const noexcept { return m_width; }


        /**
         * Returns the rectangle's height
         *
         * @return Rectangle's height.
         */
        double getHeight() const noexcept { return m_height; }

    private:
        /** Rectangle's width. **/
        double m_width;
        /** Rectangle's height. **/
        double m_height;

        /**
         * Returns the 4 corners of the rectangle.
         *
         * @return 4 points representing the rectangle's corners.
         */
        std::vector<Point> m_getCorners() const;

        /**
         * Checks if this rectangle is colliding with a segment.
         *
         * @param otherSeg A segment to check any collisions.
         * @return true if they are colliding, else false.
         */
        bool m_isCollidingWithSegment(const Segment &otherSeg) const;

        /**
         * Checks if this rectangle is colliding with another rectangle.
         *
         * @param otherSeg Another rectangle to check any collisions.
         * @return true if they are colliding, else false.
         */
        bool m_isCollidingWithRectangle(const Rectangle &otherRect) const;
    };

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_RECTANGLE_H
