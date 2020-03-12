// Don"t need to put main(), it is defined in main_unit_tests.cpp
#include "catch.hpp"

#include "collisions/shapes/segment.h"
#include "collisions/shapes/circle.h"
#include "collisions/shapes/rectangle.h"

#include <cmath>
#include <vector>
#include <utility>

SCENARIO( "First shape is a rectangle", "[shape][rectangle]" ) {
    GIVEN( "Rectangle is a square" ) {
        Position posRect {0.0, 0.0, 0.0};
        CollisionsShapes::Rectangle rect(posRect, 1.0, 1.0);
        
        REQUIRE( rect.getWidth() == rect.getHeight() );
        REQUIRE( rect.getPos() == posRect );
        REQUIRE( rect.isCollidingWith(rect) );
        
        WHEN( "There is a circle inside" ) {
            std::vector<CollisionsShapes::Circle> collidingCircles {
                CollisionsShapes::Circle(posRect, 1.0),
                CollisionsShapes::Circle(posRect, std::sqrt(2) / 2.0),
                CollisionsShapes::Circle(posRect, 2.0),
                CollisionsShapes::Circle(posRect, 0.5),
                CollisionsShapes::Circle(posRect + Point(0.5, 0.5), 1.0),
                CollisionsShapes::Circle(posRect + Point(0.5, 0.5), 0.5),
                CollisionsShapes::Circle(posRect + Point(-0.5, -0.5), 0.25),
                CollisionsShapes::Circle(posRect + Point(0.0, 1.0), 1.0),
                CollisionsShapes::Circle(posRect + Point(1.0, 1.0), std::sqrt(2) / 2.0)
            };
            
            THEN( "They are colliding" ) {
                for (auto& circ: collidingCircles) {
                    REQUIRE( rect.isCollidingWith(circ) );
                    REQUIRE( circ.isCollidingWith(rect) );
                }
            }
        }
        
        WHEN( "There is a circle outside not touching it" ) {
            // TODO generator
            THEN( "They are not colliding" ) {
                CollisionsShapes::Circle circ(posRect + Point(0.0, 2.0), 0.9);
                REQUIRE( !rect.isCollidingWith(circ) );
                REQUIRE( !circ.isCollidingWith(rect) );
            }
        }
    }
}
