#ifndef COLLISIONS_SHAPES_ABSTRACT_SHAPE_H
#define COLLISIONS_SHAPES_ABSTRACT_SHAPE_H

#include <geometry_tools/position.h>

namespace CollisionsShapes {

    /** Way to discriminate shapes in order to downcast them. */
    enum class ShapeType {
        UNDEFINED, /// There is something not right with the shape...
        SEGMENT, /// Shape is a segment.
        RECTANGLE, /// Shape is a rectangle.
        CIRCLE /// Shape is a circle.
    };

    /**
     * Defines the basic structure of a shape.
     */
    class AbstractShape {
    public:
        /**
         * Initialize a shape.
         *
         * @param pos The position of the object; it most cases it is its center.
         */
        AbstractShape(const Position &pos = {}) noexcept: m_pos(pos) {}

        /**
         * We leave the default destructor.
         */
        virtual ~AbstractShape() = default;

        /**
         * Tests if this shape collides with another shape.
         *
         * If this shape cannot compare itself to the other one, it calls isCollidingWith on the other shape with itself
         * in parameter. WARNING: if not done in the right way, it may lead to infinite, recursive calls !
         *
         * @param otherShape Shape to test any collisions.
         * @return true if shapes collide, else false.
         */
        virtual bool isCollidingWith(const AbstractShape &otherShape) const = 0;

        /**
         * Allow discrimination of shapes from their types in order to downcast them.
         *
         * See ShapeType for all available shape types.
         *
         * @return The type of the shape.
         */
        virtual ShapeType getShapeType() const = 0;

        /**
         * Sets shape's position.
         *
         * @param pos The new position.
         */
        void setPos(const Position &pos) noexcept { m_pos = pos; }

        /**
         * Returns shape's position.
         *
         * @return The current position.
         */
        const Position &getPos() const noexcept { return m_pos; };

    protected:
        /** Shape's current position **/
        Position m_pos;
    };

} // namespace CollisionsShapes

#endif // COLLISIONS_SHAPES_ABSTRACT_SHAPE_H
