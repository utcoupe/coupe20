#ifndef COLLISIONS_SHAPES_SEGMENT_H
#define COLLISIONS_SHAPES_SEGMENT_H

#include "collisions/shapes/abstract_shape.h"

#include <geometry_tools/point.h>

namespace CollisionsShapes {

    /**
     * Represents a shape as a segment.
     */
    class Segment : public AbstractShape {
    public:
        /**
         * Initializes a segment from 2 points.
         *
         * @param firstPoint One segment's extremities.
         * @param lastPoint The other segment's extremities.
         */
        Segment(const Point &firstPoint, const Point &lastPoint) noexcept;

        /**
         * We leave the default destructor.
         */
        ~Segment() override = default;

        /**
         * Checks if the segment is colliding with another shape.
         *
         * If otherShape.getShapeType() isn't ShapeType::SEGMENT, it will call  AbstractShape::isCollidingWith on
         * otherShape with this segment in parameter.
         *
         * @param otherShape The other shape to test any collisions.
         * @return true if they are colliding, else false.
         */
        bool isCollidingWith(const AbstractShape &otherShape) const override;

        /**
         * Returns ShapeType::SEGMENT (constant).
         * @return ShapeType::SEGMENT.
         */
        ShapeType getShapeType() const override { return ShapeType::SEGMENT; }

        /**
         * Returns one segment's extremities.
         * @return The first segment's extremity.
         */
        Point getFirstPoint() const noexcept { return m_firstPoint; }

        /**
         * Returns another segment's extremities.
         * @return The second segment's extremity.
         */
        Point getLastPoint() const noexcept { return m_lastPoint; }

        /**
         * Return the segment's length.
         * @return The distance between the two extremities.
         */
        double getLength() const noexcept { return m_length; }

    protected:
        /** A segment extremity. **/
        Point m_firstPoint;
        /** Another segment extremity. **/
        Point m_lastPoint;
        /** Distance between the two extremities. **/
        double m_length;
    };

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_SEGMENT_H
