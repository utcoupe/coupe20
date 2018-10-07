#ifndef COLLISIONS_POSITION_H
#define COLLISIONS_POSITION_H

#include "collisions/point.h"

#include <ostream>

class Position: public Point {
public:
    Position (double x, double y, double angle):
        Position({x, y}, angle)
    {}
    
    Position (Point pos = {0, 0}, double angle = 0.0):
        Point(pos),
        _a(angle)
    {}
    
    Point toPoint() const;
    
    // Operators
    bool operator== (const Position& other) const;
    bool operator!= (const Position& other) const;
    friend std::ostream& operator<<(std::ostream& os, const Position& pos);
    
    // Getters & setters
    double getAngle() const { return _a; }
    void setAngle(const double angle) { _a = angle; }
    
protected:
    double _a;
};

#endif // COLLISIONS_POSITION_H
