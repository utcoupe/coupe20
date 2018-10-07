#ifndef COLLISIONS_POINT_H
#define COLLISIONS_POINT_H

#include <ostream>

#include "geometry_msgs/Pose2D.h"

/**
 * Class representing a position or a vector.
 * Have somme basic operators.
 */
class Point
{
public:
    Point (double x, double y) : _x(x), _y(y) {}
    Point () : Point(0,0) {}
    
    double norm1Dist (const Point& other) const;
    double norm2Dist(const Point& other) const;
    
    // Operators
    Point operator+ (const Point& other) const;
    Point operator/ (double divisor) const;
    bool operator== (const Point& other) const;
    bool operator!= (const Point& other) const;
    friend std::ostream& operator<< (std::ostream& os, const Point& pos);
    
    // Getters & Setters
    double getX () const { return _x; }
    void setX (const double& x) { _x = x; }
    double getY () const { return _y; }
    void setY (const double& y) { _y = y; }
    
protected:
    double _x, _y;
};

#endif // COLLISIONS_POINT_H
