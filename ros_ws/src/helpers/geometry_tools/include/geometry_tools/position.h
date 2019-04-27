#ifndef COLLISIONS_POSITION_H
#define COLLISIONS_POSITION_H

#include "geometry_tools/point.h"

#include <ostream>

class Position: public Point {
public:
    constexpr Position (double x, double y, double angle) noexcept:
        Position({x, y}, angle)
    {}
    
    constexpr Position (Point pos = {0, 0}, double angle = 0.0) noexcept:
        Point(pos),
        _a(angle)
    {}
    
    constexpr Position(const Position& other) noexcept:
        Position(other.toPoint(), other.getAngle())
    {}
    
    constexpr Position(const geometry_msgs::Pose2D& pos) noexcept:
        Position(Point(pos), pos.theta)
    {}
    
    constexpr Position(const geometry_msgs::Point& pos, double theta = 0.0) noexcept:
        Position(Point(pos), theta)
    {}
    
    constexpr Point toPoint() const noexcept {
        return {_x, _y};
    }
    
    geometry_msgs::Pose2D toPose2D() const;
    
    // Operators
    constexpr bool operator== (Position other) const noexcept {
        return this->toPoint() == other.toPoint() && _a == other.getAngle();
    }
    
    constexpr bool operator!= (Position other) const noexcept{
        return !this->operator==(other);
    }

    friend std::ostream& operator<<(std::ostream& os, Position pos);
    
    // Getters & setters
    constexpr double getAngle() const           noexcept { return _a; }
    constexpr void setAngle(const double angle) noexcept { _a = angle; }
    
protected:
    double _a;
};

#endif // COLLISIONS_POSITION_H
