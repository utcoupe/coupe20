#ifndef COLLISIONS_POINT_H
#define COLLISIONS_POINT_H

#include <cmath>
#include <ostream>

#include "geometry_msgs/Pose2D.h"

/**
 * Class representing a position or a vector.
 * Have somme basic operators.
 */
class Point
{
public:
    constexpr Point (double x, double y) noexcept : _x(x), _y(y) {}
    constexpr Point () noexcept : Point(0,0) {}
    
    constexpr double norm1Dist (const Point& other) const noexcept {
        return std::abs (_x - other.getX()) + std::abs (_y - other.getY());
    }
    
    constexpr double norm2Dist (const Point& other) const noexcept {
        return std::hypot(other.getX(), other.getY());
    }
    
    // Operators
    constexpr Point operator+ (const Point& other) const noexcept {
        return {_x + other.getX(), _y + other.getY()};
    }

    constexpr Point operator/ (double divisor)     const noexcept {
        return {_x / divisor, _y / divisor};
    }
    
    constexpr bool operator== (const Point& other) const noexcept {
        return _x == other.getX() && _y == other.getY();
    }
    
    constexpr bool operator!= (const Point& other) const noexcept {
        return !operator==(other);
    }
    
    friend std::ostream& operator<< (std::ostream& os, const Point& pos) {
        os << "(" << pos._x << "," << pos._y << ")";
        return os;
    }
    
    // Getters & Setters
    constexpr double getX () const          noexcept { return _x; }
    constexpr void   setX (const double& x) noexcept { _x = x; }
    constexpr double getY () const          noexcept { return _y; }
    constexpr void   setY (const double& y) noexcept { _y = y; }
    
protected:
    double _x, _y;
};

#endif // COLLISIONS_POINT_H
