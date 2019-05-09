// Don"t need to put main(), it is defined in main_unit_tests.cpp
#include "catch.hpp"

#include "collisions/shapes/segment.h"
#include "collisions/shapes/circle.h"
#include "collisions/shapes/rectangle.h"

SCENARIO( "First shape is a rectangle", "[shape][rectangle]" ) {
    GIVEN( "Rectangle is a square" ) {
        Position posRect {0, 0, 0};
        CollisionsShapes::Rectangle rect(posRect, 1.0, 1.0);
        
        REQUIRE( rect.getWidth() == rect.getHeight() );
        REQUIRE( rect.getPos() == posRect );
        REQUIRE( rect.isCollidingWith(rect) );
        
        WHEN( "There is a circle inside" ) {
            CollisionsShapes::Circle circ(posRect, 1.0);
            
            THEN( "They are colliding" ) {
                REQUIRE( rect.isCollidingWith(circ) );
                REQUIRE( circ.isCollidingWith(rect) );
            }
        }
    }
}
