#ifndef COLLISIONS_POINT_H
#define COLLISIONS_POINT_H

#include <cmath>
#include <ostream>

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"

/**
 * Class representing a position or a vector.
 * Have somme basic operators.
 */
class Point
{
public:
    constexpr Point (double x, double y) noexcept: _x(x), _y(y) {}
    constexpr Point () noexcept: Point(0,0) {}
    constexpr Point (const Point& other) noexcept:
        Point(other.getX(),other.getY())
    {}
    constexpr Point (const geometry_msgs::Pose2D& pos) noexcept:
        Point(pos.x, pos.y)
    {}
    constexpr Point (const geometry_msgs::Point& pos) noexcept:
        Point(pos.x, pos.y)
    {}
    
    constexpr double norm1Dist (Point other) const noexcept {
        return std::abs (_x - other.getX()) + std::abs (_y - other.getY());
    }
    
    double norm2Dist (Point other) const noexcept {
        return std::hypot(_x - other.getX(), _y - other.getY());
    }
    
    Point floor () const noexcept {
        return { std::floor(_x), std::floor(_y) };
    }
    
    constexpr double scalarProduct(Point other) const noexcept {
        return _x * other.getX() + _y * other.getY();
    }
    
    constexpr double vectorProduct(Point other) const noexcept {
        return _x * other.getY() - _y * other.getX();
    }
    
    // Operators
    constexpr Point operator+ (Point other) const noexcept {
        return {_x + other.getX(), _y + other.getY()};
    }
    
    constexpr Point operator- (Point other) const noexcept {
        return {_x - other.getX(), _y - other.getY()};
    }

    constexpr Point operator/ (double divisor)     const noexcept {
        return {_x / divisor, _y / divisor};
    }

    constexpr Point operator* (double multiplier)  const noexcept {
        return {_x * multiplier, _y * multiplier};
    }
    
    constexpr bool operator== (Point other) const noexcept {
        return _x == other.getX() && _y == other.getY();
    }
    
    constexpr bool operator!= (Point other) const noexcept {
        return !operator==(other);
    }
    
    friend std::ostream& operator<< (std::ostream& os, Point pos);
    
    // Getters & Setters
    constexpr double getX () const          noexcept { return _x; }
    constexpr void   setX (const double& x) noexcept { _x = x; }
    constexpr double getY () const          noexcept { return _y; }
    constexpr void   setY (const double& y) noexcept { _y = y; }
    
    geometry_msgs::Pose2D toPose2D() const;
    geometry_msgs::Point toGeoPoint() const;
    
protected:
    double _x, _y;
};

#endif // COLLISIONS_POINT_H
